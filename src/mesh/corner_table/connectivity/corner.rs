use super::{flags, traits::Flags};
use crate::helpers::display::{display_option, display_refcell};
use std::cell::{Ref, RefCell, RefMut};
use std::fmt::Debug;
use std::ops::{Deref, DerefMut};
use tabled::Tabled;

///
/// Default implementation for Corner trait
///
#[derive(Tabled)]
pub struct Corner {
    #[tabled(display_with = "display_option")]
    opposite_corner_index: Option<usize>, // TODO: use usize::MAX as invalid value
    vertex_index: usize,

    #[tabled(display_with = "display_refcell")]
    flags: RefCell<flags::Flags>,
}

impl Corner {
    pub fn new(
        opposite_corner_index: Option<usize>,
        vertex_index: usize,
        flags: flags::Flags,
    ) -> Self {
        Self {
            opposite_corner_index,
            vertex_index,
            flags: RefCell::new(flags),
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
            opposite_corner_index: None,
            vertex_index: usize::max_value(),
            flags: Default::default(),
        }
    }
}

impl Flags for Corner {
    #[inline]
    fn flags(&self) -> impl Deref<Target = flags::Flags> {
        Ref::map(self.flags.borrow(), |flags| flags)
    }

    #[inline]
    fn flags_mut(&self) -> impl DerefMut<Target = flags::Flags> {
        RefMut::map(self.flags.borrow_mut(), |flags| flags)
    }
}

impl PartialEq for Corner {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.opposite_corner_index == other.opposite_corner_index
            && self.vertex_index == other.vertex_index
    }
}
impl Eq for Corner {}

impl Debug for Corner {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Corner")
            .field("opposite_corner_index", &self.opposite_corner_index)
            .field("vertex_index", &self.vertex_index)
            .field("flags", &self.flags.borrow().bits())
            .finish()
    }
}

#[inline]
pub fn next(corner: usize) -> usize {
    if (corner % 3) == 2 {
        corner - 2
    } else {
        corner + 1
    }
}

#[inline]
pub fn previous(corner: usize) -> usize {
    if (corner % 3) == 0 {
        corner + 2
    } else {
        corner - 1
    }
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
