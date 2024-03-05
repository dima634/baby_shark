use crate::{
    algo::merge_points::merge_points,
    mesh::traits::Mesh,
    voxel::{mesh_to_volume::MeshToVolume, meshing::MarchingCubesMesher},
};

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
    marching_cubes: MarchingCubesMesher,
}

impl VoxelRemesher {
    #[inline]
    pub fn with_voxel_size(mut self, size: f32) -> Self {
        self.mesh_to_sdf.set_voxel_size(size);
        self.marching_cubes.set_voxel_size(size);
        self
    }

    pub fn remesh<T: Mesh<ScalarType = f32>>(&mut self, mesh: &T) -> Option<T> {
        let distance_field = self.mesh_to_sdf.convert(mesh)?;
        let faces = self.marching_cubes.mesh(&distance_field);
        let indexed_faces = merge_points(&faces);
        let mesh = T::from_vertices_and_indices(&indexed_faces.points, &indexed_faces.indices);

        Some(mesh)
    }
}

impl Default for VoxelRemesher {
    fn default() -> Self {
        Self {
            mesh_to_sdf: MeshToVolume::default().with_narrow_band_width(0),
            marching_cubes: MarchingCubesMesher::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::VoxelRemesher;
    use crate::{
        helpers::aliases::Vec3,
        mesh::{builder, polygon_soup::data_structure::PolygonSoup, traits::Mesh},
    };

    #[test]
    fn test_voxel_remeshing() {
        let mesh: PolygonSoup<f32> = builder::cube(Vec3::zeros(), 1.0, 1.0, 1.0);
        let mut remesher = VoxelRemesher::default().with_voxel_size(0.1);
        let remeshed = remesher.remesh(&mesh).unwrap();

        assert!(remeshed.faces().count() > 0);
    }
}
