use crate::{
    geometry::traits::RealNumber,
    helpers::aliases::Vec3,
    io::{BuildError, BuildMode, CreateBuilder, MeshBuilder},
    mesh::polygon_soup::data_structure::PolygonSoup,
};

#[derive(Debug)]
struct Builder<R: RealNumber> {
    mode: BuildMode,
    vertices: Vec<Vec3<R>>,
    indices: Vec<usize>,
}

impl<R: RealNumber> Builder<R> {
    #[inline]
    fn new(mode: BuildMode) -> Self {
        Self {
            mode,
            vertices: Vec::new(),
            indices: Vec::new(),
        }
    }
}

impl<R: RealNumber> MeshBuilder<R, PolygonSoup<R>> for Builder<R> {
    fn add_face<T: Into<[R; 3]>>(&mut self, v1: T, v2: T, v3: T) -> Result<(), BuildError> {
        if self.mode != BuildMode::Soup {
            return Err(BuildError::WrongMode);
        }

        self.vertices.push(v1.into().into()); // into array and then into nalgebra
        self.vertices.push(v2.into().into());
        self.vertices.push(v3.into().into());
        Ok(())
    }

    fn add_vertex<T: Into<[R; 3]>>(&mut self, vertex: T) -> Result<usize, BuildError> {
        if self.mode != BuildMode::Indexed {
            return Err(BuildError::WrongMode);
        }

        let idx = self.vertices.len();
        self.vertices.push(vertex.into().into()); // into array and then into nalgebra
        Ok(idx)
    }

    fn add_face_indexed(&mut self, v0: usize, v1: usize, v2: usize) -> Result<(), BuildError> {
        if self.mode != BuildMode::Indexed {
            return Err(BuildError::WrongMode);
        }

        if v0 >= self.vertices.len() || v1 >= self.vertices.len() || v2 >= self.vertices.len() {
            return Err(BuildError::InvalidVertex);
        }

        self.indices.push(v0);
        self.indices.push(v1);
        self.indices.push(v2);
        Ok(())
    }

    fn set_num_vertices(&mut self, count: usize) {
        if self.mode != BuildMode::Indexed {
            let reserve_count = count.saturating_sub(self.vertices.len());
            self.vertices.reserve(reserve_count);
        }
    }

    fn set_num_faces(&mut self, count: usize) {
        match self.mode {
            BuildMode::Indexed => {
                let reserve_count = (count * 3).saturating_sub(self.indices.len());
                self.indices.reserve(reserve_count);
            }
            BuildMode::Soup => {
                let reserve_count = (count * 3).saturating_sub(self.vertices.len());
                self.vertices.reserve(reserve_count);
            }
        }
    }

    fn finish(self) -> Result<PolygonSoup<R>, BuildError> {
        match self.mode {
            BuildMode::Indexed => {
                let mut vertices = Vec::with_capacity(self.indices.len());

                for vertex_idx in self.indices {
                    vertices.push(self.vertices[vertex_idx].clone());
                }

                Ok(PolygonSoup { vertices })
            }
            BuildMode::Soup => Ok(PolygonSoup {
                vertices: self.vertices,
            }),
        }
    }
    
    #[inline]
    fn mode(&self) -> BuildMode {
        self.mode
    }
}

impl<R: RealNumber> CreateBuilder for PolygonSoup<R> {
    type Scalar = R;
    type Mesh = PolygonSoup<R>;

    fn builder(mode: BuildMode) -> impl MeshBuilder<Self::Scalar, Self::Mesh> {
        Builder::new(mode)
    }
}
