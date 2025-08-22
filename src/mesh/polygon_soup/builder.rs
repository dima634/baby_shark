use crate::{
    geometry::traits::RealNumber, helpers::aliases::Vec3, io,
    mesh::polygon_soup::data_structure::PolygonSoup,
};

#[derive(Debug)]
struct SoupBuilder<R: RealNumber> {
    vertices: Vec<Vec3<R>>,
}

impl<R: RealNumber> Default for SoupBuilder<R> {
    #[inline]
    fn default() -> Self {
        Self {
            vertices: Vec::default(),
        }
    }
}

impl<R: RealNumber> io::SoupBuilder<R, PolygonSoup<R>> for SoupBuilder<R> {
    fn add_face<T: Into<[R; 3]>>(&mut self, v1: T, v2: T, v3: T) -> Result<(), io::BuildError> {
        self.vertices.push(v1.into().into()); // into array and then into nalgebra
        self.vertices.push(v2.into().into());
        self.vertices.push(v3.into().into());
        Ok(())
    }

    fn set_num_faces(&mut self, count: usize) {
        let reserve_count = (count * 3).saturating_sub(self.vertices.len());
        self.vertices.reserve(reserve_count);
    }

    #[inline]
    fn finish(self) -> Result<PolygonSoup<R>, io::BuildError> {
        Ok(PolygonSoup {
            vertices: self.vertices,
        })
    }
}

#[derive(Debug)]
struct IndexedBuilder<R: RealNumber> {
    vertices: Vec<[R; 3]>,
    triangles: Vec<Vec3<R>>,
}

impl<R: RealNumber> Default for IndexedBuilder<R> {
    #[inline]
    fn default() -> Self {
        Self {
            vertices: Vec::default(),
            triangles: Vec::default(),
        }
    }
}

impl<R: RealNumber> io::IndexedBuilder<R, PolygonSoup<R>> for IndexedBuilder<R> {
    #[inline]
    fn add_vertex<T: Into<[R; 3]>>(&mut self, vertex: T) -> Result<usize, io::BuildError> {
        self.vertices.push(vertex.into());
        Ok(self.vertices.len() - 1)
    }

    fn add_face(&mut self, v1: usize, v2: usize, v3: usize) -> Result<(), io::BuildError> {
        if v1 >= self.vertices.len() || v2 >= self.vertices.len() || v3 >= self.vertices.len() {
            return Err(io::BuildError::InvalidVertex);
        }

        self.triangles.push(self.vertices[v1].into());
        self.triangles.push(self.vertices[v2].into());
        self.triangles.push(self.vertices[v3].into());
        Ok(())
    }

    #[inline]
    fn finish(self) -> Result<PolygonSoup<R>, io::BuildError> {
        Ok(PolygonSoup {
            vertices: self.triangles,
        })
    }

    fn set_num_vertices(&mut self, count: usize) {
        let reserve_count = count.saturating_sub(self.vertices.len());
        self.vertices.reserve(reserve_count);
    }

    fn set_num_faces(&mut self, count: usize) {
        let reserve_count = count.saturating_sub(self.triangles.len());
        self.triangles.reserve(reserve_count);
    }
}

impl<R: RealNumber> io::Builder for PolygonSoup<R> {
    type Scalar = R;
    type Mesh = PolygonSoup<R>;

    #[inline]
    fn builder_indexed() -> impl io::IndexedBuilder<Self::Scalar, Self::Mesh> {
        IndexedBuilder::default()
    }

    #[inline]
    fn builder_soup() -> impl io::SoupBuilder<Self::Scalar, Self::Mesh> {
        SoupBuilder::default()
    }
}
