pub mod edge_decimation;

pub use edge_decimation::{
    IncrementalDecimator,
    QuadricError,
    AlwaysDecimate,
    NeverDecimate,
    ConstantErrorDecimationCriteria,
    BoundingSphereDecimationCriteria,
    CollapseStrategy,
    EdgeDecimationCriteria,
};

/// Mesh decimation through edge collapsing. For details see [IncrementalDecimator].
pub type EdgeDecimator<S, TEdgeDecimationCriteria> = IncrementalDecimator<S, QuadricError, TEdgeDecimationCriteria>;
