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
            self
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
            self
        }
    }

    #[inline]
    fn is_marked_1(&self) -> bool {
        unsafe {
            return (*self.get_flags().get()).contains(flags::Flags::IS_MARKED_1);
        }
    }

    #[inline]
    fn set_marked_1(&self, marked: bool) -> &Self {
        unsafe {
            (*self.get_flags().get()).set(flags::Flags::IS_MARKED_1, marked);
            self
        }
    }

    #[inline]
    fn is_marked_2(&self) -> bool {
        unsafe {
            return (*self.get_flags().get()).contains(flags::Flags::IS_MARKED_2);
        }
    }

    #[inline]
    fn set_marked_2(&self, marked: bool) -> &Self {
        unsafe {
            (*self.get_flags().get()).set(flags::Flags::IS_MARKED_2, marked);
            self
        }
    }

    #[inline]
    fn is_marked_3(&self) -> bool {
        unsafe {
            return (*self.get_flags().get()).contains(flags::Flags::IS_MARKED_3);
        }
    }

    #[inline]
    fn set_marked_3(&self, marked: bool) -> &Self {
        unsafe {
            (*self.get_flags().get()).set(flags::Flags::IS_MARKED_3, marked);
            self
        }
    }

    fn get_flags(&self) -> &UnsafeCell<flags::Flags>;
}
