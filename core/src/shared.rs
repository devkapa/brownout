//! [`Shared`]: a table that W2a's snapshots share instead of copying.
//!
//! Most of an engine's bytes are tables built at load and never changed
//! after it (stamps, IC records, maps from nodes to readers, the slow
//! model). A snapshot is a deep copy, so without this each one copied them
//! all: about 600 KiB on the SAP-1. A `Shared` field clones as a reference
//! count; reading it is a plain `&T` (a `Shared<[T]>` reads through one
//! pointer, as a `Vec<T>` does), and a write through it while a snapshot
//! also holds it copies it first (copy-on-write, [`Rc::make_mut`]), so the
//! engine and the snapshot each go on with their own. A table that is never
//! written after load is never copied at all.
//!
//! Only tables fixed at load, or ones only a host edit changes, go behind
//! it. A table written on every instant would be copied at the first
//! instant after each snapshot, which saves nothing.
//!
//! [`Scratch`] is the other half: a work buffer whose contents never outlive
//! one use. It is empty between uses (`put` clears it, `take` asserts it in
//! debug builds), so a snapshot copies nothing for it, and a restored engine
//! starts each use exactly as the original did.

use std::ops::{Deref, DerefMut};
use std::rc::Rc;

/// A copy-on-write table (see the module doc).
#[derive(Debug)]
pub struct Shared<T: ?Sized>(Rc<T>);

impl<T: ?Sized> Shared<T> {
    /// The allocation's address, so a size walk counts a table its
    /// snapshots share once.
    pub fn addr(this: &Self) -> usize {
        Rc::as_ptr(&this.0) as *const u8 as usize
    }

    /// Whether `a` and `b` are the same allocation.
    pub fn ptr_eq(a: &Self, b: &Self) -> bool {
        Rc::ptr_eq(&a.0, &b.0)
    }
}

impl<T: ?Sized> Clone for Shared<T> {
    fn clone(&self) -> Self {
        Shared(Rc::clone(&self.0))
    }
}

impl<T: Default> Default for Shared<T> {
    fn default() -> Self {
        Shared(Rc::new(T::default()))
    }
}

impl<T> Default for Shared<[T]> {
    fn default() -> Self {
        Shared(Rc::from(Vec::new()))
    }
}

impl<T: ?Sized + PartialEq> PartialEq for Shared<T> {
    fn eq(&self, other: &Self) -> bool {
        *self.0 == *other.0
    }
}

impl<T> From<T> for Shared<T> {
    fn from(value: T) -> Self {
        Shared(Rc::new(value))
    }
}

impl<T> From<Vec<T>> for Shared<[T]> {
    fn from(value: Vec<T>) -> Self {
        Shared(Rc::from(value))
    }
}

impl<T: ?Sized> Deref for Shared<T> {
    type Target = T;

    fn deref(&self) -> &T {
        &self.0
    }
}

impl<T: Clone> DerefMut for Shared<T> {
    fn deref_mut(&mut self) -> &mut T {
        Rc::make_mut(&mut self.0)
    }
}

impl<T: Clone> DerefMut for Shared<[T]> {
    fn deref_mut(&mut self) -> &mut [T] {
        Rc::make_mut(&mut self.0)
    }
}

/// A work buffer, empty between uses (see the module doc). The only access
/// is [`Scratch::take`] at a use's start and [`Scratch::put`] at its end, so
/// nothing reads it across two uses.
#[derive(Debug)]
pub struct Scratch<T>(Vec<T>);

impl<T> Default for Scratch<T> {
    fn default() -> Self {
        Scratch(Vec::new())
    }
}

impl<T> Scratch<T> {
    /// The buffer for one use, empty, with the capacity the last use left.
    pub fn take(&mut self) -> Vec<T> {
        debug_assert!(self.0.is_empty(), "a scratch buffer was not cleared after its last use");
        std::mem::take(&mut self.0)
    }

    /// Hands the buffer back, cleared, for the next use.
    pub fn put(&mut self, mut buffer: Vec<T>) {
        buffer.clear();
        self.0 = buffer;
    }
}

/// A copy is empty: what the buffer holds between uses.
impl<T> Clone for Scratch<T> {
    fn clone(&self) -> Self {
        Scratch(Vec::new())
    }
}

impl<'a, T> IntoIterator for &'a Shared<[T]> {
    type Item = &'a T;
    type IntoIter = std::slice::Iter<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.iter()
    }
}

impl<'a, T: Clone> IntoIterator for &'a mut Shared<[T]> {
    type Item = &'a mut T;
    type IntoIter = std::slice::IterMut<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        Rc::make_mut(&mut self.0).iter_mut()
    }
}

impl<T: PartialEq> PartialEq<Vec<T>> for Shared<[T]> {
    fn eq(&self, other: &Vec<T>) -> bool {
        *self.0 == other[..]
    }
}

impl<T: PartialEq> PartialEq<Shared<[T]>> for Vec<T> {
    fn eq(&self, other: &Shared<[T]>) -> bool {
        self[..] == *other.0
    }
}

impl<T> FromIterator<T> for Shared<[T]> {
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        Shared(iter.into_iter().collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_clone_shares_and_a_write_copies_once() {
        let a: Shared<[u32]> = vec![1, 2, 3].into();
        let mut b = a.clone();
        assert!(Shared::ptr_eq(&a, &b));
        b[0] = 9;
        assert!(!Shared::ptr_eq(&a, &b));
        assert_eq!((&a[..], &b[..]), (&[1, 2, 3][..], &[9, 2, 3][..]));
        let before = Shared::addr(&b);
        b[1] = 8;
        assert_eq!(Shared::addr(&b), before, "unshared, a write goes in place");
    }
}
