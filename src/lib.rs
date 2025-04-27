pub mod algo;
pub mod data_structures;
pub mod decimation;
pub mod geometry;
pub mod io;
pub mod mesh;
pub mod remeshing;
pub mod spatial_partitioning;
pub mod voxel;

pub mod exports {
    pub use nalgebra;
}

mod deform;
mod helpers;

#[cfg(target_arch = "wasm32")]
mod wasm;

// Public APIs
pub use deform::{prepare_deform, DeformError, PrepareDeformError, PreparedDeform};
pub use mesh::region::{extend_region, region_boundary};
