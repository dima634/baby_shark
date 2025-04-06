use std::ops::{Index, IndexMut};
use crate::{mesh::traits::{PropertyMap, VertexProperties}, geometry::traits::RealNumber};
use super::{vertex::VertexId, CornerTable};

/// Property map for corner table vertices
pub struct VertexPropertyMap<TProperty: Default> {
    props: Vec<TProperty>
}

impl<TProperty: Default> VertexPropertyMap<TProperty> {
    pub fn new(vertices_count: usize) -> Self {
        let mut props = Vec::new();
        props.resize_with(vertices_count, Default::default);
        Self { props }
    }
}

impl<TProperty: Default> Index<VertexId> for VertexPropertyMap<TProperty> {
    type Output = TProperty;

    #[inline]
    fn index(&self, index: VertexId) -> &Self::Output {
        &self.props[index.index()]
    }
}

impl<TProperty: Default> IndexMut<VertexId> for VertexPropertyMap<TProperty> {
    #[inline]
    fn index_mut(&mut self, index: VertexId) -> &mut Self::Output {
        &mut self.props[index.index()]
    }
}

impl<TProperty: Default> PropertyMap<VertexId, TProperty> for VertexPropertyMap<TProperty> {
    #[inline]
    fn get(&self, key: &VertexId) -> Option<&TProperty> {
        return self.props.get(key.index());
    }

    #[inline]
    fn get_mut(&mut self, key: &VertexId) -> Option<&mut TProperty> {
        return self.props.get_mut(key.index());
    }
}

/// Implementation of vertex property maps for corner table
impl<TScalar: RealNumber> VertexProperties for CornerTable<TScalar> {
    type VertexPropertyMap<TProperty: Default> = VertexPropertyMap<TProperty>;

    #[inline]
    fn create_vertex_properties_map<TProperty: Default>(&self) -> Self::VertexPropertyMap<TProperty> {
        VertexPropertyMap::new(self.vertices.len())
    }
}
