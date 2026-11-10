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
//! **Bytes.** [`SnapshotTable::bytes`] walks the snapshots ([`HeapBytes`]):
//! each one's own heap, plus each allocation they share (a
//! [`crate::shared::Shared`] table, an `Rc`, a copy-on-write EEPROM image)
//! once for the whole table, whether the live engine shares it too or not.
//! It allocates nothing on the advance path: an earlier build counted every
//! allocation through a wrapping global allocator, which cost about 1 % on
//! the SAP-1. The tests still count allocations, to hold the walk to what a
//! copy really allocates.

use crate::engine::Engine;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::sync::atomic::{AtomicU32, Ordering};

/// The next snapshot id. `0` is never issued (`bo_engine_snapshot`'s error).
static NEXT_ID: AtomicU32 = AtomicU32::new(1);

struct Entry {
    engine: Box<Engine>,
}

/// One handle's snapshots. A load replaces the table with an empty one.
///
/// A `BTreeMap`, not a `HashMap`: hashing a key here added call sites to
/// std's `DefaultHasher::write`, and LLVM stopped inlining it into the hot
/// `HashMap` reads of the advance path (about 1.5 % on the SAP-1, W2a).
#[derive(Default)]
pub struct SnapshotTable {
    entries: BTreeMap<u32, Entry>,
}

impl SnapshotTable {
    /// Copies `engine` into the table and returns its id.
    pub fn take(&mut self, engine: &Engine) -> u32 {
        let copy = Box::new(engine.snapshot());
        let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
        assert!(id != 0, "snapshot ids ran out");
        self.entries.insert(id, Entry { engine: copy });
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

    /// Snapshot `id` alone, walked as if it were the only one.
    #[cfg(test)]
    pub(crate) fn walk_one(&self, id: u32) -> Option<usize> {
        self.entries.get(&id).map(|entry| entry.engine.heap_bytes(&mut Walk::default()) + std::mem::size_of::<Engine>())
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The table's total in bytes: each snapshot (the engine itself and its
    /// own heap), and each allocation the snapshots share, once.
    pub fn bytes(&self) -> f64 {
        let mut walk = Walk::default();
        let mut total = 0usize;
        for entry in self.entries.values() {
            total += std::mem::size_of::<Engine>() + entry.engine.heap_bytes(&mut walk);
        }
        total as f64
    }
}

/// One size walk: the shared allocations already counted (a `BTreeSet`,
/// for the reason [`SnapshotTable`] gives).
#[derive(Default)]
pub struct Walk {
    seen: BTreeSet<usize>,
}

impl Walk {
    /// Whether the shared allocation at `addr` is new to this walk (and so
    /// is counted now).
    pub fn first(&mut self, addr: usize) -> bool {
        self.seen.insert(addr)
    }
}

/// The heap bytes a value owns: what a clone of it allocates, with each
/// shared allocation counted once per [`Walk`]. The inline size is the
/// container's. Every engine type implements it: most impls are generated
/// at the end of each module by `core/scripts/gen_heap_bytes.py` (rerun it
/// with `--write` after changing an engine type's fields; `--check` reports a
/// stale section), and `the_walk_counts_what_a_copy_allocates` holds them to
/// a counting allocator.
pub trait HeapBytes {
    fn heap_bytes(&self, w: &mut Walk) -> usize;
}

macro_rules! no_heap {
    ($($t:ty),*) => { $(impl HeapBytes for $t { #[inline] fn heap_bytes(&self, _: &mut Walk) -> usize { 0 } })* };
}
no_heap!(bool, char, u8, u16, u32, u64, u128, usize, i8, i16, i32, i64, i128, isize, f32, f64, ());

impl<T: HeapBytes> HeapBytes for Option<T> {
    fn heap_bytes(&self, w: &mut Walk) -> usize {
        self.as_ref().map_or(0, |x| x.heap_bytes(w))
    }
}

impl<T: HeapBytes> HeapBytes for Vec<T> {
    fn heap_bytes(&self, w: &mut Walk) -> usize {
        self.len() * std::mem::size_of::<T>() + self.iter().map(|x| x.heap_bytes(w)).sum::<usize>()
    }
}

impl<T: HeapBytes> HeapBytes for std::collections::VecDeque<T> {
    fn heap_bytes(&self, w: &mut Walk) -> usize {
        self.len() * std::mem::size_of::<T>() + self.iter().map(|x| x.heap_bytes(w)).sum::<usize>()
    }
}

impl<T: HeapBytes, const N: usize> HeapBytes for [T; N] {
    fn heap_bytes(&self, w: &mut Walk) -> usize {
        self.iter().map(|x| x.heap_bytes(w)).sum()
    }
}

impl<T: HeapBytes + ?Sized> HeapBytes for Box<T> {
    fn heap_bytes(&self, w: &mut Walk) -> usize {
        std::mem::size_of_val::<T>(self) + (**self).heap_bytes(w)
    }
}

impl<T: HeapBytes> HeapBytes for [T] {
    fn heap_bytes(&self, w: &mut Walk) -> usize {
        self.iter().map(|x| x.heap_bytes(w)).sum()
    }
}

impl<T: HeapBytes + ?Sized> HeapBytes for std::rc::Rc<T> {
    fn heap_bytes(&self, w: &mut Walk) -> usize {
        if w.first(std::rc::Rc::as_ptr(self) as *const u8 as usize) {
            2 * std::mem::size_of::<usize>() + std::mem::size_of_val::<T>(self) + (**self).heap_bytes(w)
        } else {
            0
        }
    }
}

impl<T: HeapBytes + ?Sized> HeapBytes for crate::shared::Shared<T> {
    fn heap_bytes(&self, w: &mut Walk) -> usize {
        if w.first(crate::shared::Shared::addr(self)) {
            2 * std::mem::size_of::<usize>() + std::mem::size_of_val::<T>(self) + (**self).heap_bytes(w)
        } else {
            0
        }
    }
}

/// Empty between uses, so a copy holds nothing.
impl<T> HeapBytes for crate::shared::Scratch<T> {
    fn heap_bytes(&self, _: &mut Walk) -> usize {
        0
    }
}

impl<T: Copy> HeapBytes for std::cell::Cell<T> {
    fn heap_bytes(&self, _: &mut Walk) -> usize {
        0
    }
}

impl HeapBytes for String {
    fn heap_bytes(&self, _: &mut Walk) -> usize {
        self.len()
    }
}

impl<K: HeapBytes, V: HeapBytes, S> HeapBytes for HashMap<K, V, S> {
    fn heap_bytes(&self, w: &mut Walk) -> usize {
        // hashbrown: one control byte and one (K, V) slot a bucket; a clone
        // keeps the bucket count.
        let buckets = if self.capacity() == 0 { 0 } else { (self.capacity() * 8 / 7).next_power_of_two() };
        buckets * (1 + std::mem::size_of::<(K, V)>()) + self.iter().map(|(k, v)| k.heap_bytes(w) + v.heap_bytes(w)).sum::<usize>()
    }
}

impl<K: HeapBytes, S> HeapBytes for HashSet<K, S> {
    fn heap_bytes(&self, w: &mut Walk) -> usize {
        let buckets = if self.capacity() == 0 { 0 } else { (self.capacity() * 8 / 7).next_power_of_two() };
        buckets * (1 + std::mem::size_of::<K>()) + self.iter().map(|k| k.heap_bytes(w)).sum::<usize>()
    }
}

impl<K: HeapBytes, V: HeapBytes> HeapBytes for std::collections::BTreeMap<K, V> {
    fn heap_bytes(&self, w: &mut Walk) -> usize {
        self.len() * std::mem::size_of::<(K, V)>() + self.iter().map(|(k, v)| k.heap_bytes(w) + v.heap_bytes(w)).sum::<usize>()
    }
}

impl<K: HeapBytes> HeapBytes for std::collections::BTreeSet<K> {
    fn heap_bytes(&self, w: &mut Walk) -> usize {
        self.len() * std::mem::size_of::<K>() + self.iter().map(|k| k.heap_bytes(w)).sum::<usize>()
    }
}

macro_rules! tuple_heap {
    ($($n:ident $i:tt),+) => {
        impl<$($n: HeapBytes),+> HeapBytes for ($($n,)+) {
            fn heap_bytes(&self, w: &mut Walk) -> usize {
                0 $(+ self.$i.heap_bytes(w))+
            }
        }
    };
}
tuple_heap!(A 0);
tuple_heap!(A 0, B 1);
tuple_heap!(A 0, B 1, C 2);
tuple_heap!(A 0, B 1, C 2, D 3);
tuple_heap!(A 0, B 1, C 2, D 3, E 4);
tuple_heap!(A 0, B 1, C 2, D 3, E 4, F 5);

impl<T: HeapBytes + Ord> HeapBytes for std::collections::BinaryHeap<T> {
    fn heap_bytes(&self, w: &mut Walk) -> usize {
        self.len() * std::mem::size_of::<T>() + self.iter().map(|x| x.heap_bytes(w)).sum::<usize>()
    }
}

impl<T: HeapBytes> HeapBytes for std::cmp::Reverse<T> {
    fn heap_bytes(&self, w: &mut Walk) -> usize {
        self.0.heap_bytes(w)
    }
}

/// A borrow owns nothing.
impl<T: ?Sized> HeapBytes for &T {
    fn heap_bytes(&self, _: &mut Walk) -> usize {
        0
    }
}

/// A threshold closure owns nothing a walk can see beyond its box.
impl HeapBytes for dyn Fn(&[f64]) -> Vec<crate::solve::Threshold> {
    fn heap_bytes(&self, _: &mut Walk) -> usize {
        0
    }
}

/// Counts the bytes allocated on this thread, for the tests that hold the
/// walk to what a copy really allocates. Test builds only: the module built
/// for a host keeps the system allocator untouched.
#[cfg(test)]
mod counting {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::cell::Cell;

    thread_local! {
        static NET: Cell<isize> = const { Cell::new(0) };
    }

    fn note(delta: isize) {
        let _ = NET.try_with(|n| n.set(n.get() + delta));
    }

    pub fn net_allocated() -> isize {
        NET.try_with(Cell::get).unwrap_or(0)
    }

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
}

#[cfg(test)]
pub(crate) use counting::net_allocated;
