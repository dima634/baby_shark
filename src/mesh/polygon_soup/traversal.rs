use crate::geometry::traits::RealNumber;

use super::data_structure::PolygonSoup;

/// Iterator over faces of polygon soup
pub struct FacesIter<'a, TScalar: RealNumber> {
    polygon_soup: &'a PolygonSoup<TScalar>,
    current_face: usize
}

impl<'a, TScalar: RealNumber> FacesIter<'a, TScalar> {
    pub fn new(polygon_soup: &'a PolygonSoup<TScalar>) -> Self { 
        Self { 
            polygon_soup,
            current_face: 0,
        }
    }
}

impl<'a, TScalar: RealNumber> Iterator for FacesIter<'a, TScalar> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        let index = self.current_face;
        self.current_face += 3;

        if index < self.polygon_soup.vertices.len() {
            return Some(index);
        }

        None
    }
}

/// Iterator over edges of polygon soup
pub struct EdgesIter<'a, TScalar: RealNumber> {
    polygon_soup: &'a PolygonSoup<TScalar>,
    current_edge: usize
}

impl<'a, TScalar: RealNumber> EdgesIter<'a, TScalar> {
    pub fn new(polygon_soup: &'a PolygonSoup<TScalar>) -> Self { 
        Self { 
            polygon_soup,
            current_edge: 0,
        }
    }
}

impl<'a, TScalar: RealNumber> Iterator for EdgesIter<'a, TScalar> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        let index = self.current_edge;
        self.current_edge += 1;

        if index < self.polygon_soup.vertices.len() {
            return Some(index);
        }

        None
    }
}