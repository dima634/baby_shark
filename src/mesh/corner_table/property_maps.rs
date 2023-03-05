use std::ops::{Index, IndexMut};

use crate::{mesh::traits::{PropertyMap, VertexProperties}, geometry::traits::RealNumber};

use super::corner_table::CornerTable;

/// Property map for corner table vertices
pub struct VertexPropertyMap<TProperty: Default> {
    props: Vec<TProperty>
}

impl<TProperty: Default> VertexPropertyMap<TProperty> {
    pub fn new(vertices_count: usize) -> Self {
        let mut props = Vec::new();
        props.resize_with(vertices_count, Default::default);
        return Self { props };
    }
}

impl<TProperty: Default> Index<usize> for VertexPropertyMap<TProperty> {
    type Output = TProperty;

    #[inline]
    fn index(&self, index: usize) -> &Self::Output {
        return &self.props[index];
    }
}

impl<TProperty: Default> IndexMut<usize> for VertexPropertyMap<TProperty> {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        return &mut self.props[index];
    }
}

impl<TProperty: Default> PropertyMap<usize, TProperty> for VertexPropertyMap<TProperty> {
    #[inline]
    fn get(&self, key: &usize) -> Option<&TProperty> {
        return self.props.get(*key);
    }

    #[inline]
    fn get_mut(&mut self, key: &usize) -> Option<&mut TProperty> {
        return self.props.get_mut(*key);
    }
}

/// Implementation of vertex property maps for corner table
impl<TScalar: RealNumber> VertexProperties for CornerTable<TScalar> {
    type VertexPropertyMap<TProperty: Default> = VertexPropertyMap<TProperty>;

    #[inline]
    fn create_vertex_properties_map<TProperty: Default>(&self) -> Self::VertexPropertyMap<TProperty> {
        return VertexPropertyMap::new(self.vertices.len());
    }
}
