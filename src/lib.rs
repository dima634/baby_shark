pub mod mesh;
pub mod algo;
pub mod data_structures;
pub mod io;
pub mod remeshing;
pub mod spatial_partitioning;
pub mod geometry;
pub mod decimation;
pub mod triangulation;

#[cfg(feature="rerun")]
pub mod rerun;

pub mod exports {
    pub use nalgebra as nalgebra;
}

#[allow(clippy::all)]
pub mod reeb_graph;

mod helpers;
