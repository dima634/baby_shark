use crate::{
    algo::merge_points::merge_points,
    mesh::traits::Mesh,
    voxel::{mesh_to_sdf::MeshToSdf, meshing::MarchingCubesMesher},
};

pub struct VoxelRemesher {
    mesh_to_sdf: MeshToSdf,
    marching_cubes: MarchingCubesMesher,
}

impl VoxelRemesher {
    #[inline]
    pub fn with_voxel_size(mut self, size: f32) -> Self {
        self.mesh_to_sdf.set_voxel_size(size);
        self.marching_cubes.set_voxel_size(size);
        self
    }

    pub fn remesh<T: Mesh<ScalarType = f32>>(&mut self, mesh: &T) -> T {
        let distance_field = self.mesh_to_sdf.convert(mesh);
        let faces = self.marching_cubes.mesh(distance_field);
        let indexed_faces = merge_points(&faces);
        T::from_vertices_and_indices(&indexed_faces.points, &indexed_faces.indices)
    }
}

impl Default for VoxelRemesher {
    fn default() -> Self {
        Self {
            mesh_to_sdf: MeshToSdf::default(),
            marching_cubes: MarchingCubesMesher::default(),
        }
    }
}
