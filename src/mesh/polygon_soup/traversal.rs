use crate::mesh::traits::Floating;
use super::polygon_soup::PolygonSoup;

/// Iterator over faces of polygon soup
pub struct FacesIter<'a, TScalar: Floating> {
    polygon_soup: &'a PolygonSoup<TScalar>,
    current_face: usize
}

impl<'a, TScalar: Floating> FacesIter<'a, TScalar> {
    pub fn new(polygon_soup: &'a PolygonSoup<TScalar>) -> Self { 
        return Self { 
            polygon_soup,
            current_face: 0,
        }; 
    }
}

impl<'a, TScalar: Floating> Iterator for FacesIter<'a, TScalar> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        let index = self.current_face;
        self.current_face += 3;

        if index < self.polygon_soup.vertices.len() {
            return Some(index);
        }

        return None;
    }
}

/// Iterator over vertices of polygon soup
pub struct VerticesIter<'a, TScalar: Floating> {
    polygon_soup: &'a PolygonSoup<TScalar>,
    current_vertex: usize
}

impl<'a, TScalar: Floating> VerticesIter<'a, TScalar> {
    pub fn new(polygon_soup: &'a PolygonSoup<TScalar>) -> Self { 
        return Self { 
            polygon_soup,
            current_vertex: 0,
        }; 
    }
}

impl<'a, TScalar: Floating> Iterator for VerticesIter<'a, TScalar> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        let index = self.current_vertex;
        self.current_vertex += 1;

        if index < self.polygon_soup.vertices.len() {
            return Some(index);
        }

        return None;
    }
}

/// Iterator over edges of polygon soup
pub struct EdgesIter<'a, TScalar: Floating> {
    polygon_soup: &'a PolygonSoup<TScalar>,
    current_edge: usize
}

impl<'a, TScalar: Floating> EdgesIter<'a, TScalar> {
    pub fn new(polygon_soup: &'a PolygonSoup<TScalar>) -> Self { 
        return Self { 
            polygon_soup,
            current_edge: 0,
        }; 
    }
}

impl<'a, TScalar: Floating> Iterator for EdgesIter<'a, TScalar> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        let index = self.current_edge;
        self.current_edge += 1;

        if index < self.polygon_soup.vertices.len() {
            return Some(index);
        }

        return None;
    }
}