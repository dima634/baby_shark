pub mod mesh;
pub mod algo;
pub mod data_structures;
pub mod io;
pub mod remeshing;
pub mod spatial_partitioning;
pub mod geometry;
pub mod decimation;
pub mod voxel;

pub mod exports {
    pub use nalgebra as nalgebra;
}

mod deform;
mod helpers;

pub use deform::{prepare_deform, PreparedDeform};
