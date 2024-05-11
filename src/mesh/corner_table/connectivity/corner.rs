use std::cell::UnsafeCell;

use tabled::Tabled;
use crate::helpers::display::{display_option, display_unsafecell};
use super::{traits::Flags, flags};

///
/// Default implementation for Corner trait
/// 
#[derive(Debug, Tabled)]
pub struct Corner {
    #[tabled(display_with = "display_option")]
    opposite_corner_index: Option<usize>,
    vertex_index: usize,

    #[tabled(display_with = "display_unsafecell")]
    flags: UnsafeCell<flags::Flags>
}

impl Corner {
    pub fn new(
        opposite_corner_index: Option<usize>,
        vertex_index: usize,
        flags: flags::Flags
    ) -> Self { 
        Self { 
            opposite_corner_index, 
            vertex_index, 
            flags: UnsafeCell::new(flags) 
        }
    }

    #[inline]
    pub fn get_opposite_corner_index(&self) -> Option<usize> {
        self.opposite_corner_index
    }

    #[inline]
    pub fn set_opposite_corner_index(&mut self, index: Option<usize>) -> &mut Self {
        self.opposite_corner_index = index;
        self
    }

    #[inline]
    pub fn get_vertex_index(&self) -> usize {
        self.vertex_index
    }

    #[inline]
    pub fn set_vertex_index(&mut self, index: usize) -> &mut Self {
        self.vertex_index = index;
        self
    }
}

impl Default for Corner {
    fn default() -> Self {
        Self {
            opposite_corner_index:  None,
            vertex_index:           usize::max_value(),
            flags:                  Default::default() 
        }
    }
}

impl Flags for Corner {
    #[inline]
    fn get_flags(&self) -> &UnsafeCell<super::flags::Flags> {
        &self.flags
    }
}

impl PartialEq for Corner {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.opposite_corner_index == other.opposite_corner_index &&
            self.vertex_index          == other.vertex_index
    }
}
impl Eq for Corner {}


#[inline]
pub fn next(corner: usize) -> usize {
    if (corner % 3) == 2 { corner - 2 } else { corner + 1 }
}

#[inline]
pub fn previous(corner: usize) -> usize {
    if (corner % 3) == 0 { corner + 2 } else { corner - 1 }
} 

#[inline]
pub fn face(corner: usize) -> usize {
    corner / 3
}

#[inline]
pub fn first_corner(face: usize) -> usize {
    face * 3
}

#[inline]
pub fn first_corner_from_corner(corner: usize) -> usize {
    first_corner(face(corner))
}

/// Returns true when `corner` is part `face`
#[inline]
pub fn face_contains_corner(face: usize, corner: usize) -> bool {
    self::face(corner) == face
}
