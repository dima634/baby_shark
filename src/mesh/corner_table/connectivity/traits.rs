use std::cell::UnsafeCell;

use crate::mesh;
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

pub trait Vertex: Default + Flags + mesh::traits::Vertex {
    fn get_corner_index(&self) -> usize;
    fn set_corner_index(&mut self, index: usize) -> &mut Self;
}

pub trait Corner: Default + Flags {
    fn get_next_corner_index(&self) -> usize;
    fn set_next_corner_index(&mut self, index: usize) -> &Self;

    fn get_opposite_corner_index(&self) -> Option<usize>;
    fn set_opposite_corner_index(&mut self, index: usize) -> &mut Self;

    fn get_vertex_index(&self) -> usize;
    fn set_vertex_index(&mut self, index: usize) -> &mut Self;
}
