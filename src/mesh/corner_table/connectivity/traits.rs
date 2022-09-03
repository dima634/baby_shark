use std::cell::UnsafeCell;
use super::flags;

pub trait Flags {
    #[inline]
    fn is_deleted(&self) -> bool {
        unsafe {
            return (*self.get_flags().get()).contains(flags::Flags::IS_DELETED);
        }
    }

    #[inline]
    fn set_deleted(& self, deleted: bool) -> &Self {
        unsafe {
            (*self.get_flags().get()).set(flags::Flags::IS_DELETED, deleted);
            return self;
        }
    }

    #[inline]
    fn is_visited(&self) -> bool {
        unsafe {
            return (*self.get_flags().get()).contains(flags::Flags::IS_VISITED);
        }
    }

    #[inline]
    fn set_visited(&self, visited: bool) -> &Self {
        unsafe {
            (*self.get_flags().get()).set(flags::Flags::IS_VISITED, visited);
            return self;
        }
    }

    fn get_flags(&self) -> &UnsafeCell<flags::Flags>;
}
