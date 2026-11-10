//! C ABI v2 (core/README.md, "C ABI, version 2"). Every export is
//! `extern "C"` with `#[no_mangle]`; this file only marshals — parsing lives
//! in `program`, physics in `engine`. No panics ever cross this boundary on
//! bad input: an error is a negative return code plus a message in
//! `LAST_ERROR`, per the contract's error table.
//!
//! W1a adds the exports marked "v2 (W1a)" in the contract's ABI table:
//! `bo_engine_set_option`, `bo_engine_next_event_time`, the digital/channel
//! slot getters, and the harness trace. `bo_engine_set_params` (W1b),
//! `bo_engine_seed_memory`/`bo_engine_take_eeprom_updates`/
//! `bo_engine_eeprom_updates_ptr` (W1d) and `bo_engine_device_state`/
//! `bo_engine_device_state_ptr` (W1f) and `bo_engine_reset_failures`/
//! `bo_engine_records`/`bo_engine_records_ptr` (W1g) are all implemented as of this file
//! — every export name here has a matching row in `scripts/build-core.mjs`'s
//! export audit list.

use crate::engine::Engine;
use crate::error::BoError;
use crate::program::Program;
use crate::snapshot::SnapshotTable;
use std::cell::UnsafeCell;

const ABI_VERSION: u32 = 2;

/// One slot per handle ever issued by [`bo_engine_create`]. Indices are
/// 1-based (handle `h` lives at `handles[h - 1]`) so `0` is never a valid
/// handle, matching the contract. `bo_engine_create` only ever pushes a new
/// slot — it never reuses one left behind by `bo_engine_destroy` — so a
/// destroyed handle's numeric value can never be reissued.
enum HandleState {
    /// Created, no program loaded yet — `bo_engine_load` has not succeeded.
    Empty,
    /// A loaded engine and its snapshots (W2a). A load replaces both, which
    /// is what invalidates every snapshot id taken before it.
    Loaded(Engine, SnapshotTable),
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
        Some(HandleState::Loaded(engine, _)) => f(engine),
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
            *slot = HandleState::Loaded(engine, SnapshotTable::default());
            0
        }
        Err(err) => {
            // A failed load keeps the old program but retires its snapshots,
            // as every load does (README "Snapshots"): the host asked for a
            // new program, and its facade has already dropped the old ones.
            if let HandleState::Loaded(_, table) = slot {
                *table = SnapshotTable::default();
            }
            fail(&err)
        }
    }
}

/// Advances to `t_end` (absolute seconds).
#[no_mangle]
pub extern "C" fn bo_engine_advance(h: u32, t_end: f64) -> i32 {
    let handles = HANDLES.get();
    match slot_mut(handles, h) {
        Some(HandleState::Loaded(engine, _)) => match engine.advance(t_end) {
            Ok(()) => 0,
            Err(err) => fail(&err),
        },
        _ => fail(&BoError::bad_handle(format!("engine handle {h}: unknown or no program loaded"))),
    }
}

// Ruling R86: the host's clock, in milliseconds (`performance.now()`), the
// one import of the module. Read only by `bo_engine_advance_for`, once at its
// start and end and once every `kernel::YIELD_CHECK_EVERY` instants; a
// reading that is not finite turns the budget off.
#[cfg(target_arch = "wasm32")]
#[link(wasm_import_module = "env")]
extern "C" {
    fn bo_host_now_ms() -> f64;
}

/// Advances toward `t_end` for at most about `budget_ms` of the host's wall
/// time (ruling R86): `0` once at `t_end`, `1` when it yielded short of it
/// between two instants (call again with the same `t_end` to go on), or a
/// negative error code, `-7` among them when the host step runs past its
/// event cap or the run stays under the progress floor. A `budget_ms` that
/// is not positive, or a build without the clock import, never yields; the
/// event cap holds either way.
#[no_mangle]
pub extern "C" fn bo_engine_advance_for(h: u32, t_end: f64, budget_ms: f64) -> i32 {
    let handles = HANDLES.get();
    match slot_mut(handles, h) {
        Some(HandleState::Loaded(engine, _)) => {
            #[cfg(target_arch = "wasm32")]
            let mut clock = || unsafe { bo_host_now_ms() };
            #[cfg(target_arch = "wasm32")]
            let budget = Some(crate::kernel::Budget { clock: &mut clock, budget_ms, check_every: crate::kernel::YIELD_CHECK_EVERY });
            #[cfg(not(target_arch = "wasm32"))]
            let budget = {
                let _ = budget_ms;
                None
            };
            match engine.advance_for(t_end, budget) {
                Ok(true) => 0,
                Ok(false) => 1,
                Err(err) => fail(&err),
            }
        }
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

/// Counters (core/README.md, "Stat indices"). Indices 0-3 are the W0 static
/// solve; 4-14 are the event kernel's, reserved at `0.0` until a later
/// dispatch wires it in. Unknown `i` (>= 15), or an unusable handle, gives
/// `NaN`.
#[no_mangle]
pub extern "C" fn bo_engine_stat(h: u32, i: u32) -> f64 {
    with_engine(h, f64::NAN, |e| e.stat(i))
}

/// Changes an option after load. Keys 2 and 3 take effect at the next event;
/// keys 1 and 4-7 are load-time only, and any other key is unknown — all
/// of those are `-8` (core/README.md, `bo_engine_set_option`).
#[no_mangle]
pub extern "C" fn bo_engine_set_option(h: u32, key: u32, value: f64) -> i32 {
    let handles = HANDLES.get();
    match slot_mut(handles, h) {
        Some(HandleState::Loaded(engine, _)) => match engine.set_option(key, value) {
            Ok(()) => 0,
            Err(err) => fail(&err),
        },
        _ => fail(&BoError::bad_handle(format!("engine handle {h}: unknown or no program loaded"))),
    }
}

/// Reads one index field (`deviceIndex`/`paramIndex`) out of a
/// `bo_engine_set_params` triple: must be finite, non-negative, whole and in
/// `u32` range, exactly `program.rs`'s own `Reader::bounded_index` rule for
/// an index field, but returning `Option` rather than a `program::BoError`
/// since this is a live ABI call (`ERR_INVALID_ARGUMENT`, not
/// `ERR_MALFORMED` — the contract's own distinction between a bad value at
/// load time and the same shape of bad value in a live call).
fn whole_index(v: f64) -> Option<u32> {
    if v.is_finite() && v >= 0.0 && v.fract() == 0.0 && v <= u32::MAX as f64 {
        Some(v as u32)
    } else {
        None
    }
}

/// `(deviceIndex, paramIndex, value)` triples for a switch-family kind's
/// live params (owner decision 1). All or nothing: every triple is
/// validated by `Engine::set_params` before any is applied. `-8` (with
/// nothing changed) for a malformed buffer (a length not a multiple of 3, a
/// null pointer with a nonzero length, or an index field that is not a
/// valid non-negative whole number) as well as for any triple
/// `Engine::set_params` itself rejects — both are "invalid argument to a
/// live ABI call" per the contract's error table, not `ERR_MALFORMED` (that
/// code is reserved for `bo_engine_load`'s own program bytes).
#[no_mangle]
pub extern "C" fn bo_engine_set_params(h: u32, ptr: *const f64, len_f64: u32) -> i32 {
    let handles = HANDLES.get();
    let engine = match slot_mut(handles, h) {
        Some(HandleState::Loaded(engine, _)) => engine,
        _ => return fail(&BoError::bad_handle(format!("engine handle {h}: unknown or no program loaded"))),
    };

    if len_f64 % 3 != 0 {
        return fail(&BoError::invalid_argument(format!(
            "bo_engine_set_params: buffer length {len_f64} is not a multiple of 3"
        )));
    }
    let triple_count = (len_f64 / 3) as usize;
    if triple_count == 0 {
        // An empty batch is trivially valid (`Engine::set_params` itself
        // treats it as a no-op) -- no need to touch `ptr` at all, matching
        // `bo_alloc(0)`'s own "a zero-length buffer is not an error" stance.
        return 0;
    }
    if ptr.is_null() {
        return fail(&BoError::invalid_argument("bo_engine_set_params: buffer pointer is null".to_string()));
    }
    // SAFETY: same contract as `bo_engine_load`'s pointer -- `len_f64` live,
    // initialized f64s in this instance's memory for the duration of this
    // call.
    let data = unsafe { std::slice::from_raw_parts(ptr, len_f64 as usize) };

    let mut triples = Vec::with_capacity(triple_count);
    for i in 0..triple_count {
        let (raw_device, raw_param, value) = (data[i * 3], data[i * 3 + 1], data[i * 3 + 2]);
        let device_index = match whole_index(raw_device) {
            Some(v) => v,
            None => {
                return fail(&BoError::invalid_argument(format!(
                    "bo_engine_set_params: triple {i} deviceIndex {raw_device} is not a valid index"
                )))
            }
        };
        let param_index = match whole_index(raw_param) {
            Some(v) => v,
            None => {
                return fail(&BoError::invalid_argument(format!(
                    "bo_engine_set_params: triple {i} paramIndex {raw_param} is not a valid index"
                )))
            }
        };
        triples.push((device_index, param_index, value));
    }

    match engine.set_params(&triples) {
        Ok(()) => 0,
        Err(err) => fail(&err),
    }
}

/// Replaces a 74ls189's 16 stored nibbles (W1d, owner decision 2). `-8` for
/// a device index out of range, a buffer length other than 16, a null
/// pointer with a nonzero length, or any reason the evaluator's own hook
/// declines (the wrong kind, or a bad value) — validated in full before
/// anything changes, per the contract; `-5` for an unknown/destroyed/
/// unloaded handle.
#[no_mangle]
pub extern "C" fn bo_engine_seed_memory(h: u32, device: u32, ptr: *const f64, len_f64: u32) -> i32 {
    let handles = HANDLES.get();
    let engine = match slot_mut(handles, h) {
        Some(HandleState::Loaded(engine, _)) => engine,
        _ => return fail(&BoError::bad_handle(format!("engine handle {h}: unknown or no program loaded"))),
    };
    if len_f64 != 16 {
        return fail(&BoError::invalid_argument(format!(
            "bo_engine_seed_memory: buffer length {len_f64} must be exactly 16"
        )));
    }
    if ptr.is_null() {
        return fail(&BoError::invalid_argument("bo_engine_seed_memory: buffer pointer is null".to_string()));
    }
    // SAFETY: same contract as `bo_engine_load`/`bo_engine_set_params`'s
    // pointer — `len_f64` (here always 16) live, initialized f64s in this
    // instance's memory for the duration of this call.
    let words = unsafe { std::slice::from_raw_parts(ptr, len_f64 as usize) };
    match engine.seed_memory(device, words) {
        Ok(()) => 0,
        Err(err) => fail(&err),
    }
}

/// Collects and clears every dirty EEPROM image, returning the update
/// buffer's length in f64 (`>= 0`, always a multiple of `2 + W` — one
/// `deviceIndex, W, word x W` run per dirty EEPROM); `-5` with no program
/// loaded.
#[no_mangle]
pub extern "C" fn bo_engine_take_eeprom_updates(h: u32) -> i32 {
    let handles = HANDLES.get();
    match slot_mut(handles, h) {
        Some(HandleState::Loaded(engine, _)) => engine.take_eeprom_updates().len() as i32,
        _ => fail(&BoError::bad_handle(format!("engine handle {h}: unknown or no program loaded"))),
    }
}

/// The buffer from the last [`bo_engine_take_eeprom_updates`] call. Valid
/// until the next call on this engine.
#[no_mangle]
pub extern "C" fn bo_engine_eeprom_updates_ptr(h: u32) -> *const f64 {
    with_engine(h, std::ptr::null(), |e| e.eeprom_updates().as_ptr())
}

/// Collects one device's runtime state into the engine's state buffer (W1f
/// Q11; core/README.md, `bo_engine_device_state`) and returns its length in
/// f64, which is the kind's layout length: a `bench_psu` with `iLimit = 0`
/// returns 0, a success with nothing to read. `-8` for an index out of range
/// or a kind with no device state, `-5` with no program loaded. The values
/// are projected to the current time and nothing is committed, so a host may
/// read as often as it likes without changing the run.
#[no_mangle]
pub extern "C" fn bo_engine_device_state(h: u32, device: u32) -> i32 {
    let handles = HANDLES.get();
    match slot_mut(handles, h) {
        Some(HandleState::Loaded(engine, _)) => match engine.device_state(device) {
            Ok(values) => values.len() as i32,
            Err(err) => fail(&err),
        },
        _ => fail(&BoError::bad_handle(format!("engine handle {h}: unknown or no program loaded"))),
    }
}

/// The buffer from the last [`bo_engine_device_state`] collect. Valid until
/// the next call on this engine.
#[no_mangle]
pub extern "C" fn bo_engine_device_state_ptr(h: u32) -> *const f64 {
    with_engine(h, std::ptr::null(), |e| e.device_state_buffer().as_ptr())
}

/// Classic's `resetFailures()` (W1g; core/README.md, `bo_engine_reset_failures`),
/// applied as ONE event at the current time: `0`; `-4` or `-7` when the
/// re-solve at that instant fails, and then the reset rolls back and the
/// engine is exactly as it was; `-5` with no program loaded.
#[no_mangle]
pub extern "C" fn bo_engine_reset_failures(h: u32) -> i32 {
    let handles = HANDLES.get();
    match slot_mut(handles, h) {
        Some(HandleState::Loaded(engine, _)) => match engine.reset_failures() {
            Ok(()) => 0,
            Err(err) => fail(&err),
        },
        _ => fail(&BoError::bad_handle(format!("engine handle {h}: unknown or no program loaded"))),
    }
}

/// Collects one record channel into the engine's record buffer (W1g;
/// core/README.md, `bo_engine_records`) and returns its length in f64, a
/// multiple of 7: channel `0` failures, `1` warnings, `2` tripped PTCs, `3`
/// the latch log (collecting it empties it). `-8` for any other channel,
/// `-5` with no program loaded.
#[no_mangle]
pub extern "C" fn bo_engine_records(h: u32, channel: u32) -> i32 {
    let handles = HANDLES.get();
    match slot_mut(handles, h) {
        Some(HandleState::Loaded(engine, _)) => match engine.records(channel) {
            Ok(values) => values.len() as i32,
            Err(err) => fail(&err),
        },
        _ => fail(&BoError::bad_handle(format!("engine handle {h}: unknown or no program loaded"))),
    }
}

/// The buffer from the last [`bo_engine_records`] collect. Valid until the
/// next call on this engine.
#[no_mangle]
pub extern "C" fn bo_engine_records_ptr(h: u32) -> *const f64 {
    with_engine(h, std::ptr::null(), |e| e.records_buffer().as_ptr())
}

/// Time of the earliest pending event, `+Infinity` when none. `NaN` for an
/// unusable handle, matching `bo_engine_time`'s sentinel.
#[no_mangle]
pub extern "C" fn bo_engine_next_event_time(h: u32) -> f64 {
    with_engine(h, f64::NAN, |e| e.next_event_time())
}

/// Time of the earliest pending driver commit (a live token; a commit held
/// by the power-loss hold is not pending), `+Infinity` when none. `NaN` for
/// an unusable handle. Classic's `nextScheduledEventTime()` counterpart, for
/// a host that clamps its own steps onto digital due times without
/// single-stepping through a stepped partition's own Step/Crossing/Bucket
/// bookkeeping (finding 10/8).
#[no_mangle]
pub extern "C" fn bo_engine_next_digital_due_time(h: u32) -> f64 {
    with_engine(h, f64::NAN, |e| e.next_digital_due_time())
}

#[no_mangle]
pub extern "C" fn bo_engine_digital_slot_count(h: u32) -> u32 {
    with_engine(h, 0, |e| e.digital_slot_count())
}

/// `S` values; `NaN` means the key is absent this pass. Valid until the next
/// call on this engine.
#[no_mangle]
pub extern "C" fn bo_engine_digital_slots(h: u32) -> *const f64 {
    with_engine(h, std::ptr::null(), |e| e.digital_slots().as_ptr())
}

#[no_mangle]
pub extern "C" fn bo_engine_channel_slot_count(h: u32) -> u32 {
    with_engine(h, 0, |e| e.channel_slot_count())
}

/// `C` per-channel currents, amps. Valid until the next call on this engine.
#[no_mangle]
pub extern "C" fn bo_engine_channel_slots(h: u32) -> *const f64 {
    with_engine(h, std::ptr::null(), |e| e.channel_slots().as_ptr())
}

/// Turns the harness instrumentation trace on or off (`on != 0`). Harness
/// instrumentation only, not W2's host traces.
#[no_mangle]
pub extern "C" fn bo_engine_trace_enable(h: u32, on: u32) -> i32 {
    let handles = HANDLES.get();
    match slot_mut(handles, h) {
        Some(HandleState::Loaded(engine, _)) => {
            engine.trace_enable(on != 0);
            0
        }
        _ => fail(&BoError::bad_handle(format!("engine handle {h}: unknown or no program loaded"))),
    }
}

/// Flat `(t, slot, value)` triples recorded while tracing was on. Valid
/// until the next call on this engine.
#[no_mangle]
pub extern "C" fn bo_engine_trace_ptr(h: u32) -> *const f64 {
    with_engine(h, std::ptr::null(), |e| e.trace().as_ptr())
}

/// `f64` count of the trace buffer; always a multiple of 3.
#[no_mangle]
pub extern "C" fn bo_engine_trace_len(h: u32) -> u32 {
    with_engine(h, 0, |e| e.trace().len() as u32)
}

/// Resets the trace buffer to empty. A no-op on an unusable handle — there
/// is nothing to report back on this call (the contract gives it no return
/// value), the same as `bo_engine_destroy` ignoring an unknown handle.
#[no_mangle]
pub extern "C" fn bo_engine_trace_clear(h: u32) {
    with_engine(h, (), |e| e.trace_clear());
}

/// Copies the whole engine into its snapshot table (W2a; core/README.md,
/// "Snapshots") and returns the snapshot's id, or `0` with the message in
/// [`bo_error_ptr`] for an unknown handle or one with no program loaded.
#[no_mangle]
pub extern "C" fn bo_engine_snapshot(h: u32) -> u32 {
    let handles = HANDLES.get();
    match slot_mut(handles, h) {
        Some(HandleState::Loaded(engine, table)) => table.take(engine),
        _ => {
            set_error(&format!("engine handle {h}: unknown or no program loaded"));
            0
        }
    }
}

/// Puts snapshot `id` back: `0`, `-5` for an unusable handle, `-8` for an id
/// this handle's table does not hold (never issued, dropped, from another
/// handle, or taken before the last load). The snapshot stays restorable.
#[no_mangle]
pub extern "C" fn bo_engine_restore(h: u32, id: u32) -> i32 {
    let handles = HANDLES.get();
    match slot_mut(handles, h) {
        Some(HandleState::Loaded(engine, table)) => {
            if table.restore(engine, id) {
                0
            } else {
                fail(&BoError::invalid_argument(format!("snapshot {id} is not held by engine handle {h}")))
            }
        }
        _ => fail(&BoError::bad_handle(format!("engine handle {h}: unknown or no program loaded"))),
    }
}

/// Drops snapshot `id`: `0`, `-5` for an unusable handle, `-8` for an id the
/// table does not hold, a second drop among them.
#[no_mangle]
pub extern "C" fn bo_engine_snapshot_drop(h: u32, id: u32) -> i32 {
    let handles = HANDLES.get();
    match slot_mut(handles, h) {
        Some(HandleState::Loaded(_, table)) => {
            if table.drop_snapshot(id) {
                0
            } else {
                fail(&BoError::invalid_argument(format!("snapshot {id} is not held by engine handle {h}")))
            }
        }
        _ => fail(&BoError::bad_handle(format!("engine handle {h}: unknown or no program loaded"))),
    }
}

/// The handle's snapshot table in bytes (each snapshot's copy, plus each
/// shared memory image once), `NaN` for an unusable handle.
#[no_mangle]
pub extern "C" fn bo_engine_snapshot_bytes(h: u32) -> f64 {
    let handles = HANDLES.get();
    match slot_mut(handles, h) {
        Some(HandleState::Loaded(_, table)) => table.bytes(),
        _ => f64::NAN,
    }
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

    /// Builds a well-formed v2 program from the pieces (core/README.md,
    /// "Program format, version 2"): header, options, readout slot counts,
    /// device records, trailer. Mirrors `program::tests`' builder, kept
    /// separate since it exercises the ABI's byte-level `bo_engine_load`
    /// rather than `Program::parse` directly.
    fn build_program(node_count: f64, options: &[(f64, f64)], digital_slots: f64, channel_slots: f64, devices: &[Vec<f64>]) -> Vec<f64> {
        use crate::program::{MAGIC, VERSION};
        let mut out = vec![MAGIC, VERSION, node_count, devices.len() as f64, options.len() as f64];
        for &(k, v) in options {
            out.push(k);
            out.push(v);
        }
        out.push(digital_slots);
        out.push(channel_slots);
        for d in devices {
            out.extend_from_slice(d);
        }
        out.push(MAGIC);
        out
    }

    /// A pin record with no flags and zero hash states.
    fn pin(node: f64) -> [f64; 4] {
        [node, 0.0, 0.0, 0.0]
    }

    fn resistor_record(component_index: f64, a: f64, b: f64, r: f64) -> Vec<f64> {
        let mut out = vec![crate::kinds::RESISTOR as f64, component_index, 2.0];
        out.extend_from_slice(&pin(a));
        out.extend_from_slice(&pin(b));
        out.push(1.0);
        out.push(r);
        out.push(0.0); // blobCount
        out
    }

    fn source_record(component_index: f64, pos: f64, neg: f64, v: f64) -> Vec<f64> {
        let mut out = vec![crate::kinds::VOLTAGE_SOURCE as f64, component_index, 2.0];
        out.extend_from_slice(&pin(pos));
        out.extend_from_slice(&pin(neg));
        out.push(1.0);
        out.push(v);
        out.push(0.0); // blobCount
        out
    }

    /// The same one-resistor-and-a-source divider `program::tests` and
    /// `engine::tests` use, as a plain `Vec<f64>` ready for `bo_engine_load`.
    fn divider_program() -> Vec<f64> {
        build_program(3.0, &[], 0.0, 0.0, &[
            source_record(0.0, 1.0, 0.0, 10.0),
            resistor_record(1.0, 1.0, 2.0, 1000.0),
            resistor_record(2.0, 2.0, 0.0, 1000.0),
        ])
    }

    fn switch_record(component_index: f64, a: f64, b: f64, closed: f64) -> Vec<f64> {
        let mut out = vec![crate::kinds::SWITCH as f64, component_index, 2.0];
        out.extend_from_slice(&pin(a));
        out.extend_from_slice(&pin(b));
        out.push(1.0);
        out.push(closed);
        out.push(0.0); // blobCount
        out
    }

    /// A source and one switch (open) across it, device 0 and device 1
    /// respectively — a minimal program for exercising
    /// `bo_engine_set_params`'s byte-level marshaling.
    fn switch_program() -> Vec<f64> {
        build_program(3.0, &[], 0.0, 0.0, &[
            source_record(0.0, 1.0, 0.0, 10.0),
            switch_record(1.0, 1.0, 2.0, 0.0),
        ])
    }

    fn load(h: u32, data: &[f64]) -> i32 {
        bo_engine_load(h, data.as_ptr(), data.len() as u32)
    }

    #[test]
    fn abi_version_is_2() {
        assert_eq!(bo_abi_version(), 2);
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

    // --- v2 round trips of the W0 fixtures through the byte-level ABI -----

    #[test]
    fn v2_round_trip_of_chained_sources_matches_w0_currents() {
        // Same fixture as
        // `engine::tests::chained_sources_with_a_load_on_each_node_have_kcl_correct_currents`,
        // built as v2 bytes and driven through the ABI rather than a
        // hand-built `Program`, to prove the byte format round-trips to the
        // same physics.
        let _guard = lock();
        let data = build_program(3.0, &[], 0.0, 0.0, &[
            source_record(0.0, 1.0, 0.0, 5.0),
            source_record(1.0, 2.0, 1.0, 3.0),
            resistor_record(2.0, 1.0, 0.0, 1000.0),
            resistor_record(3.0, 2.0, 0.0, 1000.0),
        ]);
        let h = bo_engine_create();
        assert_eq!(load(h, &data), 0);
        let v = unsafe { std::slice::from_raw_parts(bo_engine_node_voltages(h), 3) };
        assert!((v[1] - 5.0).abs() < 1e-9);
        assert!((v[2] - 8.0).abs() < 1e-9);
        let i = unsafe { std::slice::from_raw_parts(bo_engine_device_currents(h), 4) };
        assert!((i[2] - 0.005).abs() < 1e-9, "load at A");
        assert!((i[3] - 0.008).abs() < 1e-9, "load at B");
    }

    #[test]
    fn v2_round_trip_of_an_isolated_net_reads_zero_volts() {
        // Same fixture as
        // `engine::tests::an_isolated_net_with_no_elements_reads_zero_volts_and_does_not_fail`.
        let _guard = lock();
        let data = build_program(2.0, &[], 0.0, 0.0, &[]);
        let h = bo_engine_create();
        assert_eq!(load(h, &data), 0);
        let v = unsafe { std::slice::from_raw_parts(bo_engine_node_voltages(h), 2) };
        assert_eq!(v[1], 0.0);
        assert_eq!(bo_engine_stat(h, 0), 1.0); // STAT_PARTITIONS
    }

    // --- v2 ABI exports, through the safe API ------------------------------

    #[test]
    fn next_event_time_is_infinity_for_a_loaded_static_program() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &divider_program()), 0);
        assert_eq!(bo_engine_next_event_time(h), f64::INFINITY);
    }

    #[test]
    fn next_event_time_is_nan_for_an_unusable_handle() {
        let _guard = lock();
        assert!(bo_engine_next_event_time(999_999).is_nan());
    }

    #[test]
    fn digital_and_channel_slots_are_sized_from_the_program_and_read_back_through_pointers() {
        let _guard = lock();
        let data = build_program(1.0, &[], 3.0, 2.0, &[]);
        let h = bo_engine_create();
        assert_eq!(load(h, &data), 0);
        assert_eq!(bo_engine_digital_slot_count(h), 3);
        assert_eq!(bo_engine_channel_slot_count(h), 2);
        let digital = unsafe { std::slice::from_raw_parts(bo_engine_digital_slots(h), 3) };
        assert!(digital.iter().all(|v| v.is_nan()), "{digital:?}");
        let channel = unsafe { std::slice::from_raw_parts(bo_engine_channel_slots(h), 2) };
        assert_eq!(channel, &[0.0, 0.0]);
    }

    #[test]
    fn slot_getters_are_sentinels_for_an_unusable_handle() {
        let _guard = lock();
        let bogus = 999_999u32;
        assert_eq!(bo_engine_digital_slot_count(bogus), 0);
        assert!(bo_engine_digital_slots(bogus).is_null());
        assert_eq!(bo_engine_channel_slot_count(bogus), 0);
        assert!(bo_engine_channel_slots(bogus).is_null());
    }

    #[test]
    fn trace_enable_ptr_len_and_clear_round_trip() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &divider_program()), 0);
        assert_eq!(bo_engine_trace_len(h), 0);
        assert_eq!(bo_engine_trace_enable(h, 1), 0);
        // W1a's static kernel never itself writes a trace entry (nothing
        // calls `Readout::record` yet), so this proves the plumbing —
        // enable, read back an empty buffer, clear — without a physics event
        // to record.
        assert_eq!(bo_engine_trace_len(h), 0);
        // `Vec::as_ptr` is never null, even for an empty `Vec`.
        assert!(!bo_engine_trace_ptr(h).is_null());
        bo_engine_trace_clear(h);
        assert_eq!(bo_engine_trace_len(h), 0);
        assert_eq!(bo_engine_trace_enable(h, 0), 0);
    }

    #[test]
    fn trace_enable_on_an_unusable_handle_is_bad_handle() {
        let _guard = lock();
        let bogus = 999_999u32;
        assert_eq!(bo_engine_trace_enable(bogus, 1), -5);
        assert_eq!(bo_engine_trace_len(bogus), 0);
        assert!(bo_engine_trace_ptr(bogus).is_null());
        bo_engine_trace_clear(bogus); // must not panic
    }

    /// W1f Q10: `NO_RAIL_SHORTCUT` (key 3) became load-time only, so after
    /// load only key 2 is accepted.
    #[test]
    fn set_option_accepts_key_2_and_rejects_key_3_after_load() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &divider_program()), 0);
        assert_eq!(bo_engine_set_option(h, 2, 1.0), 0);
        assert_eq!(bo_engine_set_option(h, 3, 0.0), -8);
        assert_eq!(bo_engine_set_option(h, 3, 1.0), -8);
    }

    /// Section 5 of the W1b review: `FORCE_STEPPED` (key 4) after load used
    /// to return success at the ABI layer too, with no physical effect
    /// (`partition::build` folds it into each partition's classification
    /// exactly once, at load). Now load-time only, like keys 1/5/6/7.
    #[test]
    fn set_option_rejects_force_stepped_after_load() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &divider_program()), 0);
        assert_eq!(bo_engine_set_option(h, 4, 1.0), -8);
    }

    #[test]
    fn set_option_rejects_load_time_only_and_unknown_keys_with_invalid_argument() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &divider_program()), 0);
        for key in [1, 4, 5, 6, 0, 7, 999] {
            assert_eq!(bo_engine_set_option(h, key, 0.0), -8, "key {key}");
        }
    }

    #[test]
    fn set_option_rejects_an_out_of_range_value_with_invalid_argument() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &divider_program()), 0);
        assert_eq!(bo_engine_set_option(h, 2, 2.0), -8);
    }

    #[test]
    fn set_option_on_an_unusable_handle_is_bad_handle_not_invalid_argument() {
        let _guard = lock();
        assert_eq!(bo_engine_set_option(999_999, 2, 1.0), -5);
    }

    // --- bo_engine_set_params (W1b, owner decision 1) -----------------------

    fn set_params(h: u32, triples: &[f64]) -> i32 {
        bo_engine_set_params(h, triples.as_ptr(), triples.len() as u32)
    }

    #[test]
    fn set_params_toggles_a_switch_and_the_node_voltage_updates_immediately() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &switch_program()), 0);
        let v = unsafe { std::slice::from_raw_parts(bo_engine_node_voltages(h), 3) };
        assert!(v[2].abs() < 1e-6, "open: got {}", v[2]);

        // device 1 (the switch), param 0 (`closed`), value 1.0.
        assert_eq!(set_params(h, &[1.0, 0.0, 1.0]), 0);
        let v = unsafe { std::slice::from_raw_parts(bo_engine_node_voltages(h), 3) };
        assert!((v[2] - 10.0).abs() < 1e-3, "closed: got {}", v[2]);
    }

    #[test]
    fn set_params_with_an_empty_buffer_is_a_no_op() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &switch_program()), 0);
        assert_eq!(set_params(h, &[]), 0);
    }

    #[test]
    fn set_params_rejects_a_length_not_a_multiple_of_three() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &switch_program()), 0);
        assert_eq!(set_params(h, &[1.0, 0.0]), -8);
    }

    #[test]
    fn set_params_rejects_a_non_whole_or_negative_index() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &switch_program()), 0);
        assert_eq!(set_params(h, &[1.5, 0.0, 1.0]), -8, "non-whole deviceIndex");
        assert_eq!(set_params(h, &[1.0, -1.0, 1.0]), -8, "negative paramIndex");
        assert_eq!(set_params(h, &[f64::NAN, 0.0, 1.0]), -8, "NaN deviceIndex");
    }

    #[test]
    fn set_params_rejects_an_invalid_triple_and_changes_nothing() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &switch_program()), 0);
        let before = unsafe { std::slice::from_raw_parts(bo_engine_node_voltages(h), 3) }.to_vec();
        // Device 0 (the source) has no live params.
        assert_eq!(set_params(h, &[0.0, 0.0, 1.0]), -8);
        let after = unsafe { std::slice::from_raw_parts(bo_engine_node_voltages(h), 3) };
        assert_eq!(after, before.as_slice());
    }

    #[test]
    fn advance_for_without_the_clock_import_reaches_t_end_like_advance() {
        // Ruling R86, addition (a): a native build has no clock import, so
        // the budget is off and `bo_engine_advance_for` is
        // `bo_engine_advance` (0 at `t_end`, never 1), its event cap the
        // kernel's; an unusable handle is -5.
        let _guard = lock();
        assert_eq!(bo_engine_advance_for(999_999, 1.0, 16.0), -5);
        let h = loaded(&[], 2.0, &[source_record(0.0, 1.0, 0.0, 5.0), resistor_record(1.0, 1.0, 0.0, 50.0)]);
        assert_eq!(bo_engine_advance_for(h, 0.5, 1e-9), 0);
        assert_eq!(bo_engine_time(h), 0.5);
        assert_eq!(bo_engine_advance_for(h, 0.25, 16.0), -6, "backwards");
    }

    #[test]
    fn set_params_on_an_unusable_handle_is_bad_handle() {
        let _guard = lock();
        assert_eq!(set_params(999_999, &[0.0, 0.0, 1.0]), -5);
    }

    #[test]
    fn set_params_stat_7_counts_cache_hits() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &switch_program()), 0);
        assert_eq!(bo_engine_stat(h, 7), 0.0);
        assert_eq!(set_params(h, &[1.0, 0.0, 0.0]), 0); // toggle open -> open: no real change.
        assert_eq!(set_params(h, &[1.0, 0.0, 1.0]), 0); // open -> closed: a real change.
        assert_eq!(set_params(h, &[1.0, 0.0, 0.0]), 0); // closed -> open: a real change.
        assert_eq!(set_params(h, &[1.0, 0.0, 0.0]), 0); // open -> open again: repeats a prior state.
        assert!(bo_engine_stat(h, 7) >= 1.0, "expected at least one cache hit, got {}", bo_engine_stat(h, 7));
    }

    // --- bo_engine_device_state (W1f Q11) -----------------------------------

    /// A `battery_pack` record (core/README.md, "Battery record"): `e` volts
    /// full and `r` ohm fresh, 1 Ah, full, the generic depletion shape and
    /// table `[(0, 0), (1, e)]`. `ocv_scale` and `temp_factor` are arguments
    /// so a test can tell the scaled OCV from the unscaled resistance.
    fn battery_record(component_index: f64, pos: f64, neg: f64, e: f64, r: f64, ocv_scale: f64, temp_factor: f64) -> Vec<f64> {
        use crate::kinds::battery_param::*;
        let mut params = [0.0; COUNT];
        params[OCV_SCALE] = ocv_scale;
        params[R_BASE] = r;
        params[TEMP_FACTOR] = temp_factor;
        params[DEP_A] = 0.5;
        params[DEP_P] = 2.0;
        params[DEP_K] = 0.05;
        params[CAPACITY_AH] = 1.0;
        params[SOC0] = 1.0;
        let table = [2.0, 0.0, 0.0, 1.0, e];
        let mut out = vec![crate::kinds::BATTERY_PACK as f64, component_index, 2.0];
        out.extend_from_slice(&pin(pos));
        out.extend_from_slice(&pin(neg));
        out.push(COUNT as f64);
        out.extend_from_slice(&params);
        out.push(table.len() as f64);
        out.extend_from_slice(&table);
        out
    }

    fn psu_record(component_index: f64, pos: f64, neg: f64, v: f64, i_limit: f64) -> Vec<f64> {
        let mut out = vec![crate::kinds::BENCH_PSU as f64, component_index, 2.0];
        out.extend_from_slice(&pin(pos));
        out.extend_from_slice(&pin(neg));
        out.push(2.0);
        out.push(v);
        out.push(i_limit);
        out.push(0.0); // blobCount
        out
    }

    /// Exact mode (`NO_RAIL_SHORTCUT`, option key 3) must return the hub's
    /// layouts (Rails point 9), so every layout test runs in both.
    fn rail_options(exact: bool) -> Vec<(f64, f64)> {
        if exact {
            vec![(3.0, 1.0)]
        } else {
            Vec::new()
        }
    }

    /// One collect, read back through the pointer export as a host reads it,
    /// or the negative code.
    fn device_state(h: u32, device: u32) -> Result<Vec<f64>, i32> {
        let len = bo_engine_device_state(h, device);
        if len < 0 {
            return Err(len);
        }
        let ptr = bo_engine_device_state_ptr(h);
        assert!(!ptr.is_null(), "a successful collect leaves a readable buffer, even an empty one");
        Ok(unsafe { std::slice::from_raw_parts(ptr, len as usize) }.to_vec())
    }

    fn last_error() -> String {
        let bytes = unsafe { std::slice::from_raw_parts(bo_error_ptr(), bo_error_len() as usize) };
        String::from_utf8(bytes.to_vec()).unwrap()
    }

    fn loaded(options: &[(f64, f64)], node_count: f64, devices: &[Vec<f64>]) -> u32 {
        let h = bo_engine_create();
        assert_eq!(load(h, &build_program(node_count, options, 0.0, 0.0, devices)), 0, "{}", last_error());
        h
    }

    #[test]
    fn device_state_returns_each_w1f_layout_in_both_rail_modes() {
        let _guard = lock();
        for exact in [false, true] {
            let options = rail_options(exact);
            // A generic {5 V, 0.9 ohm} cell into 200 ohm, read at load: no time
            // has passed (owner decision 8), so soc 1, OCV 5, R 0.9, 3600 C
            // left, nothing discharged or rejected (the README's six values).
            let h = loaded(&options, 2.0, &[battery_record(0.0, 1.0, 0.0, 5.0, 0.9, 1.0, 1.0), resistor_record(1.0, 1.0, 0.0, 200.0)]);
            assert_eq!(device_state(h, 0), Ok(vec![1.0, 5.0, 0.9, 3600.0, 0.0, 0.0]), "battery at load, exact {exact}");
            // Drained for a second it is still six values, and a second read
            // is the same read (a collect commits nothing).
            assert_eq!(bo_engine_advance(h, 1.0), 0);
            let drained = device_state(h, 0).unwrap();
            assert_eq!(drained.len(), 6, "exact {exact}");
            assert!(drained[0] < 1.0 && drained[4] > 0.0, "the cell drained, exact {exact}: {drained:?}");
            assert_eq!(device_state(h, 0), Ok(drained), "exact {exact}");

            // Probe P5's limited PSU, {5 V, 0.1 A} into 10 ohm: 0.5 A wanted,
            // so CC from load, `reg` 2.
            let h = loaded(&options, 2.0, &[psu_record(0.0, 1.0, 0.0, 5.0, 0.1), resistor_record(1.0, 1.0, 0.0, 10.0)]);
            assert_eq!(device_state(h, 0), Ok(vec![2.0]), "limited PSU, exact {exact}");

            // An unlimited PSU is an ideal source: an empty layout, which is a
            // success (length 0), not -8.
            let h = loaded(&options, 2.0, &[psu_record(0.0, 1.0, 0.0, 5.0, 0.0), resistor_record(1.0, 1.0, 0.0, 10.0)]);
            assert_eq!(bo_engine_device_state(h, 0), 0, "unlimited PSU, exact {exact}");
            assert_eq!(device_state(h, 0), Ok(Vec::new()), "unlimited PSU, exact {exact}");
        }
    }

    #[test]
    fn device_state_reports_the_scaled_ocv_and_the_unscaled_resistance() {
        // ocvScale 0.8 and tempFactor 2. Index 1 is OCV(soc) * ocvScale and
        // index 2 is R(soc) = (rBase * dep) * tempFactor, which ocvScale never
        // touches: at soc 1, where dep is exactly 1, 5 * 0.8 = 4 V and 0.9 * 2 =
        // 1.8 ohm. Were the two swapped or both scaled, neither would match.
        let _guard = lock();
        for exact in [false, true] {
            let h = loaded(&rail_options(exact), 2.0, &[battery_record(0.0, 1.0, 0.0, 5.0, 0.9, 0.8, 2.0), resistor_record(1.0, 1.0, 0.0, 200.0)]);
            assert_eq!(device_state(h, 0), Ok(vec![1.0, 4.0, 1.8, 3600.0, 0.0, 0.0]), "exact {exact}");
        }
    }

    #[test]
    fn device_state_is_minus_8_for_a_kind_with_none_or_an_index_out_of_range() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &divider_program()), 0);
        // An ideal source has no layout. A resistor has W1g's thermal one,
        // empty without a code 91 record: a success with nothing to read.
        assert_eq!(bo_engine_device_state(h, 0), -8);
        assert!(last_error().contains("has no device state"), "{}", last_error());
        assert_eq!(bo_engine_device_state(h, 1), 0, "a resistor without a thermal record");
        for device in [3, u32::MAX] {
            assert_eq!(bo_engine_device_state(h, device), -8, "device {device}");
            assert!(last_error().contains("out of range"), "device {device}: {}", last_error());
        }
    }

    #[test]
    fn device_state_empties_the_buffer_on_either_minus_8() {
        // A host that ignores the code and reads the pointer must not get the
        // last device's values back as this call's: a full collect first, so
        // there is something stale to leave behind.
        let _guard = lock();
        let h = loaded(&[], 2.0, &[battery_record(0.0, 1.0, 0.0, 5.0, 0.9, 1.0, 1.0), resistor_record(1.0, 1.0, 0.0, 200.0)]);
        let buffer_len = |h: u32| with_engine(h, usize::MAX, |e| e.device_state_buffer().len());
        for (device, why) in [(2, "out of range"), (u32::MAX, "out of range")] {
            assert_eq!(device_state(h, 0).map(|v| v.len()), Ok(6), "device {device}");
            assert_eq!(buffer_len(h), 6, "device {device}");
            assert_eq!(bo_engine_device_state(h, device), -8, "device {device}");
            assert!(last_error().contains(why), "device {device}: {}", last_error());
            assert_eq!(buffer_len(h), 0, "device {device}: a failed collect leaves no values");
        }
    }

    #[test]
    fn device_state_is_minus_5_without_a_program_and_its_pointer_is_null() {
        let _guard = lock();
        let empty = bo_engine_create();
        let destroyed = bo_engine_create();
        assert_eq!(load(destroyed, &divider_program()), 0);
        bo_engine_destroy(destroyed);
        for h in [empty, destroyed, 999_999] {
            assert_eq!(bo_engine_device_state(h, 0), -5, "handle {h}");
            assert!(last_error().contains("unknown or no program loaded"), "handle {h}: {}", last_error());
            assert!(bo_engine_device_state_ptr(h).is_null(), "handle {h}");
        }
    }

    // --- bo_engine_records / bo_engine_reset_failures (W1g) ------------------

    fn records(h: u32, channel: u32) -> Result<Vec<f64>, i32> {
        let len = bo_engine_records(h, channel);
        if len < 0 {
            return Err(len);
        }
        let ptr = bo_engine_records_ptr(h);
        // SAFETY: the buffer from the collect just made, `len` f64 long, valid
        // until the next call on this engine.
        Ok(unsafe { std::slice::from_raw_parts(ptr, len as usize) }.to_vec())
    }

    fn rating_record(device: f64, p_max: f64, r_power: f64) -> Vec<f64> {
        vec![crate::kinds::RATING as f64, device, 0.0, 3.0, device, p_max, r_power, 0.0]
    }

    #[test]
    fn records_and_reset_failures_round_trip_a_latch() {
        // 5 V across 50 ohm rated 0.25 W latches at 1 s (W1g point 6).
        let _guard = lock();
        let h = loaded(&[], 2.0, &[source_record(0.0, 1.0, 0.0, 5.0), resistor_record(1.0, 1.0, 0.0, 50.0), rating_record(1.0, 0.25, 50.0)]);
        assert_eq!(records(h, 0), Ok(vec![]));
        assert_eq!(bo_engine_advance(h, 1.5), 0);
        let r = records(h, 0).unwrap();
        assert_eq!(r.len(), 7);
        assert_eq!(&r[..3], &[1.0, 1.0, -1.0]);
        assert!((r[3] - 1.0).abs() < 1e-12);
        assert_eq!(records(h, 1), Ok(vec![]), "no Rust warning in W1");
        assert_eq!(records(h, 2), Ok(vec![]), "no PTC");
        assert_eq!(records(h, 3).map(|v| v.len()), Ok(7));
        assert_eq!(records(h, 3), Ok(vec![]), "the log was taken");
        assert_eq!(bo_engine_reset_failures(h), 0);
        assert_eq!(records(h, 0), Ok(vec![]), "the reset cleared the latch");
        assert_eq!(bo_engine_time(h), 1.5, "and did not advance time");
    }

    #[test]
    fn records_refuse_an_unknown_channel_and_both_exports_need_a_program() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &divider_program()), 0);
        assert_eq!(bo_engine_records(h, 4), -8);
        assert!(last_error().contains("unknown channel"), "{}", last_error());
        assert_eq!(with_engine(h, usize::MAX, |e| e.records_buffer().len()), 0, "a refused collect leaves no records");
        let empty = bo_engine_create();
        for h in [empty, 999_999] {
            assert_eq!(bo_engine_records(h, 0), -5, "handle {h}");
            assert_eq!(bo_engine_reset_failures(h), -5, "handle {h}");
            assert!(bo_engine_records_ptr(h).is_null(), "handle {h}");
        }
    }

    // ---- W2a: snapshots --------------------------------------------------

    fn node2(h: u32) -> f64 {
        unsafe { std::slice::from_raw_parts(bo_engine_node_voltages(h), 3)[2] }
    }

    #[test]
    fn a_snapshot_restores_any_number_of_times_and_only_on_its_own_handle() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &switch_program()), 0);
        let open = bo_engine_snapshot(h);
        assert_ne!(open, 0);
        assert_eq!(set_params(h, &[1.0, 0.0, 1.0]), 0);
        assert!((node2(h) - 10.0).abs() < 1e-3, "closed");
        for _ in 0..2 {
            assert_eq!(bo_engine_restore(h, open), 0);
            assert!(node2(h).abs() < 1e-6, "the open switch is back");
            assert_eq!(set_params(h, &[1.0, 0.0, 1.0]), 0);
        }
        let other = bo_engine_create();
        assert_eq!(load(other, &switch_program()), 0);
        assert_eq!(bo_engine_restore(other, open), -8, "another handle's id");
        assert!(last_error().contains("not held"), "{}", last_error());
        assert_eq!(bo_engine_snapshot_drop(other, open), -8);
        assert_eq!(bo_engine_restore(h, 0), -8, "0 is never issued");
        assert_eq!(bo_engine_restore(h, u32::MAX), -8, "never issued");
    }

    #[test]
    fn a_drop_or_a_load_retires_an_id_and_the_bytes_go_back_to_zero() {
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &switch_program()), 0);
        assert_eq!(bo_engine_snapshot_bytes(h), 0.0);
        let a = bo_engine_snapshot(h);
        let b = bo_engine_snapshot(h);
        assert!(a != 0 && b != 0 && a != b, "{a} {b}");
        assert!(bo_engine_snapshot_bytes(h) > 0.0);
        assert_eq!(bo_engine_snapshot_drop(h, a), 0);
        assert_eq!(bo_engine_snapshot_drop(h, a), -8, "a second drop");
        assert_eq!(bo_engine_restore(h, a), -8, "a dropped id");
        assert_eq!(load(h, &switch_program()), 0);
        assert_eq!(bo_engine_restore(h, b), -8, "an id from before the load");
        assert_eq!(bo_engine_snapshot_drop(h, b), -8);
        assert_eq!(bo_engine_snapshot_bytes(h), 0.0, "the load emptied the table");
        let c = bo_engine_snapshot(h);
        assert!(c > b, "ids are never reused: {c} after {b}");
    }

    #[test]
    fn a_failed_load_retires_every_snapshot_too() {
        // README "Snapshots": a load, failed ones included, retires the ids
        // taken before it, even though a failed load keeps the old program.
        let _guard = lock();
        let h = bo_engine_create();
        assert_eq!(load(h, &switch_program()), 0);
        let id = bo_engine_snapshot(h);
        assert_ne!(id, 0);
        let malformed = vec![1.0, 2.0, 3.0];
        assert_eq!(load(h, &malformed), -1);
        assert!(node2(h).abs() < 1e-6, "the old program is still loaded");
        assert_eq!(bo_engine_restore(h, id), -8, "an id from before the failed load");
        assert_eq!(bo_engine_snapshot_bytes(h), 0.0);
    }

    #[test]
    fn the_snapshot_exports_need_a_loaded_handle() {
        let _guard = lock();
        let empty = bo_engine_create();
        let destroyed = bo_engine_create();
        assert_eq!(load(destroyed, &switch_program()), 0);
        let id = bo_engine_snapshot(destroyed);
        bo_engine_destroy(destroyed);
        for h in [0, empty, destroyed, u32::MAX] {
            assert_eq!(bo_engine_snapshot(h), 0, "handle {h}");
            assert!(last_error().contains("unknown or no program loaded"), "{}", last_error());
            assert_eq!(bo_engine_restore(h, id), -5, "handle {h}");
            assert_eq!(bo_engine_snapshot_drop(h, id), -5, "handle {h}");
            assert!(bo_engine_snapshot_bytes(h).is_nan(), "handle {h}");
        }
    }
}
