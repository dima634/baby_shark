use std::cell::UnsafeCell;

use tabled::Tabled;
use crate::helpers::display::{display_option, display_unsafecell};
use super::{traits::{Flags, Corner}, flags};

///
/// Default implementation for Corner trait
/// 
#[derive(Debug, Tabled)]
pub struct DefaultCorner {
    next_corner_index: usize,
    #[tabled(display_with = "display_option")]
    opposite_corner_index: Option<usize>,
    vertex_index: usize,

    #[tabled(display_with = "display_unsafecell")]
    flags: UnsafeCell<flags::Flags>
}

impl DefaultCorner {
    pub fn new(
        next_corner_index: usize, 
        opposite_corner_index: Option<usize>,
        vertex_index: usize,
        flags: flags::Flags
    ) -> Self { 
        return Self { 
            next_corner_index, 
            opposite_corner_index, 
            vertex_index, 
            flags: UnsafeCell::new(flags) 
        };
    }
}

impl Default for DefaultCorner {
    fn default() -> Self {
        return Self { 
            next_corner_index:      usize::max_value(), 
            opposite_corner_index:  None,
            vertex_index:           usize::max_value(),
            flags:                  Default::default() 
        };
    }
}

impl Flags for DefaultCorner {
    #[inline]
    fn get_flags(&self) -> &UnsafeCell<super::flags::Flags> {
        return &self.flags;
    }
}

impl Corner for DefaultCorner {
    #[inline]
    fn get_next_corner_index(&self) -> usize {
        return self.next_corner_index;
    }

    #[inline]
    fn set_next_corner_index(&mut self, index: usize) -> &Self {
        self.next_corner_index = index;
        return self;
    }

    #[inline]
    fn get_opposite_corner_index(&self) -> Option<usize> {
        return self.opposite_corner_index;
    }

    #[inline]
    fn set_opposite_corner_index(&mut self, index: usize) -> &mut Self {
        self.opposite_corner_index = Some(index);
        return self;
    }

    #[inline]
    fn get_vertex_index(&self) -> usize {
        return self.vertex_index;
    }

    #[inline]
    fn set_vertex_index(&mut self, index: usize) -> &mut Self {
        self.vertex_index = index;
        return self;
    }
}

impl PartialEq for DefaultCorner {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        let flags_equal: bool;
        unsafe {
            flags_equal = (*self.flags.get()) == (*other.flags.get());
        }

        return 
            self.next_corner_index     == other.next_corner_index &&   
            self.opposite_corner_index == other.opposite_corner_index &&
            self.vertex_index          == other.vertex_index && 
            flags_equal;
    }
}
impl Eq for DefaultCorner {}
