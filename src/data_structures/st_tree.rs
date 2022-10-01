use std::marker::PhantomData;

use crate::geometry::traits::RealNumber;


pub type Vertex = usize;

pub struct STTree<TCostType: RealNumber> {
    a: PhantomData<TCostType>
}

impl<TCostType: RealNumber> STTree<TCostType> {
    pub fn parent(&self, v: &Vertex) -> Option<&Vertex> {
        todo!()
    }

    pub fn root(&self, v: &Vertex) -> &Vertex {
        todo!()
    }

    pub fn cost(&self, v: &Vertex) -> TCostType {
        todo!()
    }

    pub fn min_cost(&self, v: &Vertex) -> TCostType {
        todo!()
    }
    
    pub fn update(&mut self, v: &Vertex, cost: TCostType) -> TCostType {
        todo!()
    }

    pub fn link(&mut self, v1: &Vertex, v2: &Vertex, cost: TCostType) -> TCostType {
        todo!()
    }

    pub fn cut(&mut self, v: &Vertex) {
        todo!()
    }

    pub fn evert(&mut self, v: &Vertex) {
        todo!()
    }
}
