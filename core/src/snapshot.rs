//! W2a: whole-engine snapshots behind opaque ids (W2 design 2.3, ruling D7
//! (a); core/README.md, "Snapshots").
//!
//! A snapshot is a deep copy of the [`Engine`] ([`Engine::snapshot`]), held
//! in its handle's [`SnapshotTable`] until the host drops it. The core sets
//! no count or byte cap: WASM memory never shrinks, so the facade owns the
//! lifetime (`CoreSnapshot.dispose()` and a `FinalizationRegistry`), and
//! [`SnapshotTable::bytes`] lets the host watch the total.
//!
//! Ids come from one counter for the whole module and are never reused, so
//! an id from another handle, from before a load (which starts a new table),
//! or one already dropped is unknown here and reads `-8`.
//!
//! **Bytes.** A snapshot's size is what its copy allocated: the module's
//! allocator keeps a net count of the bytes it hands out, and a snapshot
//! records the change across its copy. The copy-on-write EEPROM images are
//! shared, not copied, so they are counted on their own: once per distinct
//! image any snapshot in the table holds.

use crate::engine::Engine;
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::collections::HashMap;
use std::sync::atomic::{AtomicU32, Ordering};

/// The next snapshot id. `0` is never issued (`bo_engine_snapshot`'s error).
static NEXT_ID: AtomicU32 = AtomicU32::new(1);

struct Entry {
    engine: Box<Engine>,
    bytes: usize,
}

/// One handle's snapshots. A load replaces the table with an empty one.
#[derive(Default)]
pub struct SnapshotTable {
    entries: HashMap<u32, Entry>,
}

impl SnapshotTable {
    /// Copies `engine` into the table and returns its id.
    pub fn take(&mut self, engine: &Engine) -> u32 {
        let before = net_allocated();
        let copy = Box::new(engine.snapshot());
        let bytes = (net_allocated() - before).max(0) as usize;
        let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
        assert!(id != 0, "snapshot ids ran out");
        self.entries.insert(id, Entry { engine: copy, bytes });
        id
    }

    /// Puts snapshot `id` back into `engine`; `false` for an id this table
    /// does not hold. The snapshot stays in the table, so it can be restored
    /// again.
    pub fn restore(&self, engine: &mut Engine, id: u32) -> bool {
        match self.entries.get(&id) {
            Some(entry) => {
                engine.restore(&entry.engine);
                true
            }
            None => false,
        }
    }

    /// Drops snapshot `id`; `false` for an id this table does not hold.
    pub fn drop_snapshot(&mut self, id: u32) -> bool {
        self.entries.remove(&id).is_some()
    }

    /// What snapshot `id`'s copy allocated, without the images it shares.
    #[cfg(test)]
    pub(crate) fn copy_bytes(&self, id: u32) -> Option<usize> {
        self.entries.get(&id).map(|entry| entry.bytes)
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The table's total in bytes: each snapshot's copy, plus each distinct
    /// memory image the snapshots hold, once.
    pub fn bytes(&self) -> f64 {
        let mut images = Vec::new();
        let mut total = 0usize;
        for entry in self.entries.values() {
            total += entry.bytes;
            entry.engine.memory_images(&mut images);
        }
        images.sort_unstable();
        images.dedup();
        total += images.iter().map(|&(_, len)| len).sum::<usize>();
        total as f64
    }
}

// The net bytes this thread's allocator calls have handed out. A
// const-initialised `Cell` has no destructor, so reading it inside the
// allocator never allocates or touches a torn-down slot.
thread_local! {
    static NET: Cell<isize> = const { Cell::new(0) };
}

fn note(delta: isize) {
    let _ = NET.try_with(|n| n.set(n.get() + delta));
}

/// The net bytes allocated on this thread so far.
pub fn net_allocated() -> isize {
    NET.try_with(Cell::get).unwrap_or(0)
}

/// The system allocator with a net byte count, so a snapshot can measure its
/// own copy.
struct Counting;

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let p = System.alloc(layout);
        if !p.is_null() {
            note(layout.size() as isize);
        }
        p
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let p = System.alloc_zeroed(layout);
        if !p.is_null() {
            note(layout.size() as isize);
        }
        p
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout);
        note(-(layout.size() as isize));
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let p = System.realloc(ptr, layout, new_size);
        if !p.is_null() {
            note(new_size as isize - layout.size() as isize);
        }
        p
    }
}

#[global_allocator]
static ALLOCATOR: Counting = Counting;
