use crate::dynamic_vdb;
use crate::static_vdb;
use crate::voxel::utils::box_indices;
use crate::voxel::*;

type StaticTree = static_vdb!((), 4, 3, 2);
type DynamicTree = dynamic_vdb!((), 4, 3, 2);

#[test]
fn test_static_tree_insert_remove() {
    let mut tree = StaticTree::empty(Vector3::zeros());

    let size = 32;

    for idx in box_indices(0, size) {
        tree.insert(&idx, ());
        assert!(tree.at(&idx).is_some());
    }

    assert!(!tree.is_empty());

    for idx in box_indices(0, size) {
        tree.remove(&idx);
        assert!(tree.at(&idx).is_none());
    }

    assert!(tree.is_empty());
}

#[test]
fn test_static_tree_remove() {
    let mut tree = StaticTree::empty(Vector3::zeros());
    
    let size = 32;
    let indices = box_indices(0, size).collect::<Vec<_>>();

    for idx in &indices {
        tree.insert(idx, ());
        assert!(tree.at(&idx).is_some());
    }

    let half = (size * size * size / 2) as usize;
    for idx in indices.iter().take(half) {
        tree.remove(idx);
    }

    for (i, idx) in indices.iter().enumerate() {
        let at = tree.at(idx);
        
        if i < half {
            assert!(at.is_none());
        } else {
            assert!(at.is_some());
        }
    }
}

#[test]
fn test_static_tree_fill() {
    type Tree = static_vdb!((), 3, 2);

    let mut tree = Tree::empty(Vector3::zeros());
    let voxels_per_dim = Tree::resolution();

    assert!(tree.is_empty());

    tree.fill(());
    
    for idx in box_indices(0, voxels_per_dim as isize) {
        assert!(tree.at(&idx).is_some());
    }
}

#[test]
fn test_dynamic_tree_insert_remove() {
    let mut tree = DynamicTree::new();

    assert!(tree.is_empty());

    let size = 32;

    for idx in box_indices(0, size) {
        tree.insert(&idx, ());
        assert!(tree.at(&idx).is_some());
    }

    assert!(!tree.is_empty());

    for idx in box_indices(0, size) {
        tree.remove(&idx);
    }

    assert!(tree.is_empty());
}
