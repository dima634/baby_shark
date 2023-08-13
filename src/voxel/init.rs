#[macro_export]
macro_rules! vdb {
    ($($rest:expr),+) => { 
        $crate::voxel::tree::RootNode::<vdb!(@internal $($rest,)*)>
    };

    (@internal $branching:expr,) => { 
        $crate::voxel::tree::LeafNode<$branching, $branching, { $crate::voxel::tree::leaf_node_size($branching) }>
    };

    ($(@internal)? $branching:expr, $($rest:expr),+ $(,)?) => { 
        $crate::voxel::tree::InternalNode::<
            vdb!(@internal $($rest,)*),
            $branching,
            { $crate::voxel::tree::internal_node_branching::<vdb!(@internal $($rest,)*)>($branching) }, 
            { $crate::voxel::tree::internal_node_size::<vdb!(@internal $($rest,)*)>($branching) },
            { $crate::voxel::tree::internal_node_bit_size::<vdb!(@internal $($rest,)*)>($branching) }
        >
    };
}

#[cfg(test)]
mod tests {
    use crate::voxel::tree::{HasChild, TreeNode};

    #[test]
    fn test_vdb_macro_2_3_4() {
        type Grid = vdb!(5, 4, 3);
        type Lvl1 = <Grid as HasChild>::Child;
        type Lvl2 = <Lvl1 as HasChild>::Child;
        type Lvl3 = <Lvl2 as HasChild>::Child;

        assert_eq!(Lvl1::BRANCHING_TOTAL, 12);
        assert_eq!(Lvl1::BRANCHING, 5);
        assert_eq!(Lvl1::SIZE, 32768); // 1<<5*3 = 32768

        assert_eq!(Lvl2::BRANCHING_TOTAL, 7);
        assert_eq!(Lvl2::BRANCHING, 4);
        assert_eq!(Lvl2::SIZE, 4096); // 1<<4*3 = 4096

        assert_eq!(Lvl3::BRANCHING_TOTAL, 3);
        assert_eq!(Lvl3::BRANCHING, 3);
        assert_eq!(Lvl3::SIZE, 512); // 1<<3*3 = 512 voxels (64 u8)
    }
}
