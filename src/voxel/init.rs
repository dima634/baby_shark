#[macro_export]
macro_rules! static_vdb {
    ($value_type:ty, $branching:expr) => {
        $crate::voxel::LeafNode<
            $value_type,
            $branching,
            $branching,
            { $crate::voxel::leaf_node_size($branching) },
            { $crate::voxel::leaf_node_bit_size($branching) }
        >
    };

    ($value_type:ty, $branching:expr, $($rest:tt)+) => {
        $crate::voxel::InternalNode::<
            $value_type,
            $crate::static_vdb!($value_type, $($rest)*),
            $branching,
            { $crate::voxel::internal_node_branching::<$crate::static_vdb!($value_type, $($rest)*)>($branching) },
            { $crate::voxel::internal_node_size($branching) },
            { $crate::voxel::internal_node_bit_size($branching) },
            false,
        >
    };

    ($value_type:ty, par $branching:expr, $($rest:tt)+) => {
        $crate::voxel::InternalNode::<
            $value_type,
            $crate::static_vdb!($value_type, $($rest)*),
            $branching,
            { $crate::voxel::internal_node_branching::<$crate::static_vdb!($value_type, $($rest)*)>($branching) },
            { $crate::voxel::internal_node_size($branching) },
            { $crate::voxel::internal_node_bit_size($branching) },
            true,
        >
    };
}

#[macro_export]
macro_rules! dynamic_vdb {
    ($value_type:ty, $($rest:tt)+) => {
        $crate::voxel::RootNode::<$crate::static_vdb!($value_type, $($rest)*)>
    };
}

#[cfg(test)]
mod tests {
    use crate::voxel::{value::empty::Empty, TreeNode};

    #[test]
    fn test_static_vdb_macro_5_4_3() {
        type Grid = static_vdb!(Empty, 5, 4, 3);
        type Lvl1 = <Grid as TreeNode>::Child;
        type Lvl2 = <Lvl1 as TreeNode>::Child;

        assert_eq!(Grid::BRANCHING_TOTAL, 12);
        assert_eq!(Grid::BRANCHING, 5);
        assert_eq!(Grid::SIZE, 32768); // 1<<5*3 = 32768

        assert_eq!(Lvl1::BRANCHING_TOTAL, 7);
        assert_eq!(Lvl1::BRANCHING, 4);
        assert_eq!(Lvl1::SIZE, 4096); // 1<<4*3 = 4096

        assert_eq!(Lvl2::BRANCHING_TOTAL, 3);
        assert_eq!(Lvl2::BRANCHING, 3);
        assert_eq!(Lvl2::SIZE, 512 / usize::BITS as usize); // 1<<3*3 = 512 voxels (8 u64)
    }

    #[test]
    fn test_dynamic_vdb_macro() {
        type Grid = dynamic_vdb!(Empty, 5, 4, 3);
        type Lvl1 = <Grid as TreeNode>::Child;
        type Lvl2 = <Lvl1 as TreeNode>::Child;
        type Lvl3 = <Lvl2 as TreeNode>::Child;

        assert_eq!(Lvl1::BRANCHING_TOTAL, 12);
        assert_eq!(Lvl1::BRANCHING, 5);
        assert_eq!(Lvl1::SIZE, 32768); // 1<<5*3 = 32768

        assert_eq!(Lvl2::BRANCHING_TOTAL, 7);
        assert_eq!(Lvl2::BRANCHING, 4);
        assert_eq!(Lvl2::SIZE, 4096); // 1<<4*3 = 4096

        assert_eq!(Lvl3::BRANCHING_TOTAL, 3);
        assert_eq!(Lvl3::BRANCHING, 3);
        assert_eq!(Lvl3::SIZE, 512 / usize::BITS as usize); // 1<<3*3 = 512 voxels (8 u64)
    }

    #[test]
    fn test_parallel_internal_node_macro() {
        type _Grid1 = dynamic_vdb!(Empty, par 5, 4, 3);
        type _Grid2 = dynamic_vdb!(Empty, par 5, par 4, 3);
        type _Grid3 = dynamic_vdb!(Empty, 5, par 4, 3);
    }
}
