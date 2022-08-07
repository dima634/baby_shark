use crate::mesh;
use super::flags;

pub trait TopologyFlags {
    #[inline]
    fn is_deleted(&self) -> bool {
        return self.get_flags().contains(flags::TopologyFlags::IS_DELETED);
    }

    #[inline]
    fn set_deleted(&mut self, deleted: bool) -> &mut Self {
        self.get_flags_mut().set(flags::TopologyFlags::IS_DELETED, deleted);
        return self;
    }

    #[inline]
    fn is_visited(&self) -> bool {
        return self.get_flags().contains(flags::TopologyFlags::IS_VISITED);
    }

    #[inline]
    fn set_visited(&mut self, visited: bool) -> &mut Self {
        self.get_flags_mut().set(flags::TopologyFlags::IS_VISITED, visited);
        return self;
    }

    fn get_flags_mut(&mut self) -> &mut flags::TopologyFlags;
    fn get_flags(&self) -> &flags::TopologyFlags;
}

pub trait TopologyPrimitive: Default + TopologyFlags {
    fn get_index(&self) -> usize;
    fn set_index(&mut self, index: usize) -> &mut Self;
}

pub trait Vertex: TopologyPrimitive + mesh::traits::Vertex {
    fn get_corner_index(&self) -> usize;
    fn set_corner_index(&mut self, index: usize) -> &mut Self;
}

pub trait Corner: TopologyPrimitive {
    fn get_next_corner_index(&self) -> usize;
    fn set_next_corner_index(&mut self, index: usize) -> &Self;

    fn get_opposite_corner_index(&self) -> Option<usize>;
    fn set_opposite_corner_index(&mut self, index: usize) -> &mut Self;

    fn get_face_index(&self) -> usize;

    fn get_vertex_index(&self) -> usize;
    fn set_vertex_index(&mut self, index: usize) -> &mut Self;
}
