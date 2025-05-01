use crate::{
    mesh::traits::{FromSoup, Triangles},
    voxel::{
        mesh_to_volume::MeshToVolume,
        meshing::{DualContouringMesher, MarchingCubesMesher},
    },
};

pub enum MeshingMethod {
    /// Feature preserving meshing, which tries to preserve sharp features but may produce non-manifold/self-intersecting meshes.
    FeaturePreserving,
    /// Meshing which provides strong guarantees about topology (no self-intersections, no non-manifold edges/vertices) of the output mesh, but may smooth sharp features.
    Manifold,
}

///
/// Voxel remeshing.
/// This algorithm convert mesh into SDF which is then used to create a new mesh using marching cubes.
/// So average edge length of the output mesh is equal to the voxel size.
///
/// Self-intersecting and open meshes are supported. If the input mesh is open,
/// the output mesh may contain holes/open edges. However, small holes are usually closed.
/// Also, the input mesh should have more or less consistent orientation.
/// Output mesh is guaranteed to be manifold.
///
/// For now only f32 is supported as a underlying scalar type.
///
/// ## Example
/// ```ignore
/// use baby_shark::{
///     mesh::{builder, polygon_soup::data_structure::PolygonSoup},
///     remeshing::voxel::VoxelRemesher,
/// };
/// use nalgebra::Vector3;
///
/// fn main() {
///     let mesh: PolygonSoup<f32> = builder::cube(Vector3::zeros(), 1.0, 1.0, 1.0);
///     let mut remesher = VoxelRemesher::default().with_voxel_size(0.02);
///     let remeshed = remesher.remesh(&mesh).unwrap();
/// }
/// ```
///
pub struct VoxelRemesher {
    mesh_to_sdf: MeshToVolume,
    meshing_method: MeshingMethod,
    voxel_size: f32,
}

impl VoxelRemesher {
    #[inline]
    pub fn with_voxel_size(mut self, size: f32) -> Self {
        self.mesh_to_sdf.set_voxel_size(size);
        self.voxel_size = size;
        self
    }

    #[inline]
    pub fn with_meshing_method(mut self, method: MeshingMethod) -> Self {
        self.meshing_method = method;
        self
    }

    pub fn remesh<T>(&mut self, mesh: &T) -> Option<T>
    where
        T: Triangles<Scalar = f32> + FromSoup<Scalar = f32>,
    {
        let distance_field = self.mesh_to_sdf.convert(mesh)?;

        let faces = match self.meshing_method {
            MeshingMethod::FeaturePreserving => {
                let mut dc = DualContouringMesher::default().with_voxel_size(self.voxel_size);
                dc.mesh(&distance_field)?
            }
            MeshingMethod::Manifold => {
                let mut mc = MarchingCubesMesher::default().with_voxel_size(self.voxel_size);
                mc.mesh(&distance_field)
            }
        };

        let mesh = T::from_triangles_soup(faces.into_iter());

        Some(mesh)
    }
}

impl Default for VoxelRemesher {
    fn default() -> Self {
        Self {
            mesh_to_sdf: MeshToVolume::default().with_narrow_band_width(0),
            voxel_size: 1.0,
            meshing_method: MeshingMethod::Manifold,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::VoxelRemesher;
    use crate::{
        helpers::aliases::Vec3,
        mesh::{builder, polygon_soup::data_structure::PolygonSoup},
    };

    #[test]
    fn test_voxel_remeshing() {
        let mesh: PolygonSoup<f32> = builder::cube(Vec3::zeros(), 1.0, 1.0, 1.0);
        let mut remesher = VoxelRemesher::default().with_voxel_size(0.1);
        let remeshed = remesher.remesh(&mesh).unwrap();

        assert!(remeshed.faces().count() > 0);
    }
}
