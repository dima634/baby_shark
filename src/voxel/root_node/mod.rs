mod csg;
mod flood_fill;
mod tree_node;

use super::*;
use crate::helpers::aliases::Vec3i;
use std::{
    collections::BTreeMap,
    hash::{Hash, Hasher},
};

#[derive(Debug)]
pub(super) struct RootNode<TChild: TreeNode> {
    root: BTreeMap<RootKey, Box<TChild>>,
}

impl<TChild: TreeNode> RootNode<TChild> {
    #[inline]
    pub fn new() -> Self {
        Self {
            root: Default::default(),
        }
    }

    #[inline]
    fn root_key(index: &Vec3i) -> RootKey {
        RootKey(Vec3i::new(
            index.x & !((1 << TChild::BRANCHING_TOTAL) - 1),
            index.y & !((1 << TChild::BRANCHING_TOTAL) - 1),
            index.z & !((1 << TChild::BRANCHING_TOTAL) - 1),
        ))
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
struct RootKey(Vec3i);

impl RootKey {
    #[inline]
    fn x(&self) -> isize {
        self.0.x
    }

    #[inline]
    fn y(&self) -> isize {
        self.0.y
    }

    #[inline]
    fn z(&self) -> isize {
        self.0.z
    }
}

impl PartialOrd for RootKey {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for RootKey {
    #[inline]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.data.0.cmp(&other.0.data.0)
    }
}

impl Hash for RootKey {
    #[inline]
    fn hash<H: Hasher>(&self, state: &mut H) {
        let hash =
            ((1 << 8) - 1) & ((self.0.x * 73856093) ^ (self.0.y * 19349663) ^ (self.0.z * 83492791));
        state.write_isize(hash);
    }
}
