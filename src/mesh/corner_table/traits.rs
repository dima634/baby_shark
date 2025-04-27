use std::ops::{Deref, DerefMut};
use super::flags;

pub trait Flags {
    #[inline]
    fn is_deleted(&self) -> bool {
        self.flags().contains(flags::Flags::IS_DELETED)
    }

    #[inline]
    fn set_deleted(& self, deleted: bool) -> &Self {
        self.flags_mut().set(flags::Flags::IS_DELETED, deleted);
        self
    }

    #[inline]
    fn is_marked_1(&self) -> bool {
        self.flags().contains(flags::Flags::IS_MARKED_1)
    }

    #[inline]
    fn set_marked_1(&self, marked: bool) -> &Self {
        self.flags_mut().set(flags::Flags::IS_MARKED_1, marked);
        self
    }

    #[inline]
    fn is_marked_2(&self) -> bool {
        self.flags().contains(flags::Flags::IS_MARKED_2)
    }

    #[inline]
    fn set_marked_2(&self, marked: bool) -> &Self {
        self.flags_mut().set(flags::Flags::IS_MARKED_2, marked);
        self
    }

    fn flags(&self) -> impl Deref<Target = flags::Flags>;
    fn flags_mut(&self) -> impl DerefMut<Target = flags::Flags>;
}
