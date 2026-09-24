//! C ABI v1 (core/README.md, "C ABI, version 1"). Every export is
//! `extern "C"` with `#[no_mangle]`; this file only marshals — parsing lives
//! in `program`, physics in `engine`. No panics ever cross this boundary on
//! bad input: an error is a negative return code plus a message in
//! `LAST_ERROR`, per the contract's error table.

use crate::engine::Engine;
use crate::error::BoError;
use crate::program::Program;
use std::cell::UnsafeCell;

const ABI_VERSION: u32 = 1;

/// One slot per handle ever issued by [`bo_engine_create`]. Indices are
/// 1-based (handle `h` lives at `handles[h - 1]`) so `0` is never a valid
/// handle, matching the contract. `bo_engine_create` only ever pushes a new
/// slot — it never reuses one left behind by `bo_engine_destroy` — so a
/// destroyed handle's numeric value can never be reissued.
enum HandleState {
    /// Created, no program loaded yet — `bo_engine_load` has not succeeded.
    Empty,
    Loaded(Engine),
    /// `bo_engine_destroy` was called; the slot is permanently dead.
    Destroyed,
}

/// A `Sync` wrapper so a `static` can hold interior-mutable state without
/// `static mut`. This target (`wasm32-unknown-unknown`, no `atomics`
/// feature) never has more than one thread, and every export here runs to
/// completion before the host's single JS thread can call another export —
/// wasm has no reentrancy without an explicit callback into the host and
/// back, which this module never does. So there is never more than one live
/// `&mut` to the inner value, which is what actually makes `Sync` sound
/// here; it has nothing to do with real concurrent access.
struct GlobalCell<T> {
    inner: UnsafeCell<T>,
}

unsafe impl<T> Sync for GlobalCell<T> {}

impl<T> GlobalCell<T> {
    const fn new(value: T) -> Self {
        GlobalCell { inner: UnsafeCell::new(value) }
    }

    /// # Safety (upheld crate-wide, not per-call)
    /// Callers must not hold two live references from the same `GlobalCell`
    /// at once. Every function below calls this exactly once and finishes
    /// with it before returning, which is what keeps that true.
    #[allow(clippy::mut_from_ref)]
    fn get(&self) -> &mut T {
        unsafe { &mut *self.inner.get() }
    }
}

static HANDLES: GlobalCell<Vec<HandleState>> = GlobalCell::new(Vec::new());
static LAST_ERROR: GlobalCell<Vec<u8>> = GlobalCell::new(Vec::new());

fn set_error(message: &str) {
    let buf = LAST_ERROR.get();
    buf.clear();
    buf.extend_from_slice(message.as_bytes());
}

/// Records `err`'s message as the last error and returns its code — the
/// shape every fallible export here needs to return.
fn fail(err: &BoError) -> i32 {
    set_error(&err.message);
    err.code
}

fn slot_mut(handles: &mut [HandleState], h: u32) -> Option<&mut HandleState> {
    if h == 0 {
        return None;
    }
    handles.get_mut((h - 1) as usize)
}

/// Runs `f` against the loaded engine at `h`, or returns `missing` if `h` is
/// unknown, destroyed, or has no program loaded — the getters below have no
/// channel to report *why*, per the contract's own ABI table (only
/// `bo_engine_load` / `bo_engine_advance` carry an error code), so an
/// unusable handle just reads back a sentinel (`0`, `NaN`, or a null
/// pointer) instead of the real value.
fn with_engine<R>(h: u32, missing: R, f: impl FnOnce(&mut Engine) -> R) -> R {
    let handles = HANDLES.get();
    match slot_mut(handles, h) {
        Some(HandleState::Loaded(engine)) => f(engine),
        _ => missing,
    }
}

#[no_mangle]
pub extern "C" fn bo_abi_version() -> u32 {
    ABI_VERSION
}

/// Scratch f64 buffer, zeroed. Freed with [`bo_free`], passing the same
/// `len_f64`.
///
/// A boxed slice, not a leaked `Vec`: `bo_free` only knows the length, and a
/// `Vec`'s capacity is not promised to equal it, whereas a boxed slice's
/// allocation is exactly `len` elements by construction.
#[no_mangle]
pub extern "C" fn bo_alloc(len_f64: u32) -> *mut f64 {
    let buf: Box<[f64]> = vec![0.0; len_f64 as usize].into_boxed_slice();
    Box::into_raw(buf) as *mut f64
}

#[no_mangle]
pub extern "C" fn bo_free(ptr: *mut f64, len_f64: u32) {
    if ptr.is_null() {
        return;
    }
    // SAFETY: contract requires `ptr`/`len_f64` to be exactly what a prior
    // `bo_alloc` returned/was given, so this rebuilds the same boxed slice.
    unsafe {
        drop(Box::from_raw(std::ptr::slice_from_raw_parts_mut(ptr, len_f64 as usize)));
    }
}

/// New engine handle. `0` is never issued; handles are never reused, even
/// after `bo_engine_destroy` (see [`HandleState`]).
#[no_mangle]
pub extern "C" fn bo_engine_create() -> u32 {
    let handles = HANDLES.get();
    handles.push(HandleState::Empty);
    handles.len() as u32
}

/// Frees the engine. Unknown or already-destroyed handles are ignored.
#[no_mangle]
pub extern "C" fn bo_engine_destroy(h: u32) {
    let handles = HANDLES.get();
    if let Some(slot) = slot_mut(handles, h) {
        *slot = HandleState::Destroyed;
    }
}

/// Loads a program: parse, derive fixed nodes, partition, solve at t = 0.
/// `0` on success, or a negative error code with the message in
/// [`bo_error_ptr`] / [`bo_error_len`]. A failed load leaves the handle
/// exactly as it was — an `Empty` handle stays `Empty`, and a previously
/// `Loaded` one keeps its old program rather than being torn down by a bad
/// reload.
#[no_mangle]
pub extern "C" fn bo_engine_load(h: u32, ptr: *const f64, len_f64: u32) -> i32 {
    let handles = HANDLES.get();
    let slot = match slot_mut(handles, h) {
        Some(slot) => slot,
        None => return fail(&BoError::bad_handle(format!("unknown engine handle {h}"))),
    };
    if matches!(slot, HandleState::Destroyed) {
        return fail(&BoError::bad_handle(format!("engine handle {h} was destroyed")));
    }
    if ptr.is_null() {
        // A null pointer is never valid even for len_f64 == 0 (a program is
        // never zero-length: the header alone is 4 f64s), so this is always
        // a malformed-program report, not a special case.
        return fail(&BoError::malformed("program pointer is null"));
    }
    // SAFETY: the contract requires `ptr` to point to `len_f64` live,
    // initialized f64s in this instance's memory for the duration of this
    // call. The TS glue re-derives the view from `memory.buffer` on every
    // call rather than caching one, which is what keeps this true even
    // after a call that grew memory.
    let data = unsafe { std::slice::from_raw_parts(ptr, len_f64 as usize) };
    match Program::parse(data).and_then(|program| Engine::load(&program)) {
        Ok(engine) => {
            *slot = HandleState::Loaded(engine);
            0
        }
        Err(err) => fail(&err),
    }
}

/// Advances to `t_end` (absolute seconds).
#[no_mangle]
pub extern "C" fn bo_engine_advance(h: u32, t_end: f64) -> i32 {
    let handles = HANDLES.get();
    match slot_mut(handles, h) {
        Some(HandleState::Loaded(engine)) => match engine.advance(t_end) {
            Ok(()) => 0,
            Err(err) => fail(&err),
        },
        _ => fail(&BoError::bad_handle(format!("engine handle {h}: unknown or no program loaded"))),
    }
}

#[no_mangle]
pub extern "C" fn bo_engine_time(h: u32) -> f64 {
    with_engine(h, f64::NAN, |e| e.time())
}

#[no_mangle]
pub extern "C" fn bo_engine_node_count(h: u32) -> u32 {
    with_engine(h, 0, |e| e.node_count())
}

/// `N` volts, node 0 = 0. Valid until the next call on this engine.
#[no_mangle]
pub extern "C" fn bo_engine_node_voltages(h: u32) -> *const f64 {
    with_engine(h, std::ptr::null(), |e| e.voltages().as_ptr())
}

#[no_mangle]
pub extern "C" fn bo_engine_device_count(h: u32) -> u32 {
    with_engine(h, 0, |e| e.device_count())
}

/// `D` amps, one per device record, in record order. Valid until the next
/// call on this engine.
#[no_mangle]
pub extern "C" fn bo_engine_device_currents(h: u32) -> *const f64 {
    with_engine(h, std::ptr::null(), |e| e.currents().as_ptr())
}

/// `0` partitions, `1` static solves, `2` largest partition size, `3` fixed
/// nodes. Unknown `i`, or an unusable handle, gives `NaN`.
#[no_mangle]
pub extern "C" fn bo_engine_stat(h: u32, i: u32) -> f64 {
    with_engine(h, f64::NAN, |e| e.stat(i))
}

/// UTF-8 message for the most recent non-zero return from `bo_engine_load`
/// or `bo_engine_advance`.
#[no_mangle]
pub extern "C" fn bo_error_ptr() -> *const u8 {
    LAST_ERROR.get().as_ptr()
}

#[no_mangle]
pub extern "C" fn bo_error_len() -> u32 {
    LAST_ERROR.get().len() as u32
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Mutex, MutexGuard};

    // `HANDLES` / `LAST_ERROR` are process-global by design (see
    // `GlobalCell`'s doc comment) — sound on the actual target,
    // `wasm32-unknown-unknown`, which never runs more than one thread. But
    // `cargo test` runs this same code natively, where the test harness
    // does use real OS threads, so these tests must serialize themselves
    // against each other or they race on that shared state (handle numbers
    // colliding, one test's program clobbering another's). This lock is a
    // test-only concession to that; it has no production counterpart.
    static TEST_LOCK: Mutex<()> = Mutex::new(());

    fn lock() -> MutexGuard<'static, ()> {
        TEST_LOCK.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// Builds a well-formed one-resistor-and-a-source divider program, the
    /// same shape `program::tests` uses, as a plain `Vec<f64>` ready for
    /// `bo_engine_load`.
    fn divider_program() -> Vec<f64> {
        use crate::program::{KIND_RESISTOR, KIND_VOLTAGE_SOURCE, MAGIC, VERSION};
        vec![
            MAGIC, VERSION, 3.0, 3.0, //
            KIND_VOLTAGE_SOURCE as f64, 0.0, 2.0, 1.0, 0.0, 1.0, 10.0, 0.0, //
            KIND_RESISTOR as f64, 1.0, 2.0, 1.0, 2.0, 1.0, 1000.0, 0.0, //
            KIND_RESISTOR as f64, 2.0, 2.0, 2.0, 0.0, 1.0, 1000.0, 0.0, //
            MAGIC,
        ]
    }

    fn load(h: u32, data: &[f64]) -> i32 {
        bo_engine_load(h, data.as_ptr(), data.len() as u32)
    }

    #[test]
    fn abi_version_is_1() {
        assert_eq!(bo_abi_version(), 1);
    }

    #[test]
    fn alloc_then_free_round_trips() {
        let ptr = bo_alloc(4);
        assert!(!ptr.is_null());
        unsafe {
            for i in 0..4 {
                assert_eq!(*ptr.add(i), 0.0);
                *ptr.add(i) = i as f64;
            }
        }
        bo_free(ptr, 4);
    }

    #[test]
    fn unknown_handle_is_bad_handle_everywhere() {
        let _guard = lock();
        let bogus = 999_999u32;
        let data = divider_program();
        assert_eq!(load(bogus, &data), -5);
        assert_eq!(bo_engine_advance(bogus, 1.0), -5);
        assert!(bo_engine_time(bogus).is_nan());
        assert_eq!(bo_engine_node_count(bogus), 0);
        assert_eq!(bo_engine_device_count(bogus), 0);
        assert!(bo_engine_node_voltages(bogus).is_null());
        assert!(bo_engine_device_currents(bogus).is_null());
        assert!(bo_engine_stat(bogus, 0).is_nan());
    }

    #[test]
    fn handle_zero_is_never_valid() {
        let _guard = lock();
        assert_eq!(load(0, &divider_program()), -5);
    }

    #[test]
    fn destroyed_handle_is_bad_handle_and_is_never_reused() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &divider_program()), 0);
        bo_engine_destroy(h);
        assert_eq!(load(h, &divider_program()), -5, "destroyed handle rejects a reload");
        assert_eq!(bo_engine_advance(h, 1.0), -5);

        // A fresh create() must not reissue the destroyed handle's number.
        let h2 = bo_engine_create();
        assert_ne!(h, h2, "handles are never reused after destroy");
        assert_eq!(load(h2, &divider_program()), 0);
    }

    #[test]
    fn destroying_an_unknown_handle_is_ignored_not_an_error() {
        let _guard = lock();
        bo_engine_destroy(123_456);
    }

    #[test]
    fn a_handle_with_no_program_loaded_reads_back_sentinels() {
        let _guard = lock();
        let h = bo_engine_create();
        assert!(bo_engine_time(h).is_nan());
        assert_eq!(bo_engine_node_count(h), 0);
        assert!(bo_engine_node_voltages(h).is_null());
        assert_eq!(bo_engine_advance(h, 1.0), -5);
    }

    #[test]
    fn load_solves_and_queries_report_the_divider() {
        let _guard = lock();
        let h = bo_engine_create();
        let data = divider_program();
        assert_eq!(load(h, &data), 0);
        assert_eq!(bo_engine_node_count(h), 3);
        assert_eq!(bo_engine_device_count(h), 3);
        let v_ptr = bo_engine_node_voltages(h);
        assert!(!v_ptr.is_null());
        let v = unsafe { std::slice::from_raw_parts(v_ptr, 3) };
        assert!((v[2] - 5.0).abs() < 1e-6);
        assert_eq!(bo_engine_advance(h, 5.0), 0);
        assert_eq!(bo_engine_time(h), 5.0);
        assert_eq!(bo_engine_advance(h, 1.0), -6);
    }

    #[test]
    fn a_malformed_program_reports_error_and_message() {
        let _guard = lock();
        let h = bo_engine_create();
        let mut data = divider_program();
        data[0] = 0.0; // corrupt the header magic
        let code = load(h, &data);
        assert_eq!(code, -1);
        let ptr = bo_error_ptr();
        let len = bo_error_len() as usize;
        let message = unsafe { std::str::from_utf8(std::slice::from_raw_parts(ptr, len)).unwrap() };
        assert!(message.contains("header magic"), "message was: {message}");
    }

    #[test]
    fn a_failed_reload_leaves_the_previous_program_intact() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &divider_program()), 0);
        let mut bad = divider_program();
        bad[1] = 999.0; // corrupt the version
        assert_eq!(load(h, &bad), -1);
        // The original, successfully loaded program must still answer.
        assert_eq!(bo_engine_node_count(h), 3);
        assert!(!bo_engine_node_voltages(h).is_null());
    }
}
