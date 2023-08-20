use crate::dynamic_vdb;
use crate::static_vdb;
use crate::voxel::*;

type StaticTree = static_vdb!(4, 3, 2);
type DynamicTree = dynamic_vdb!(4, 3, 2);

#[test]
fn test_static_tree_insert() {
    let mut tree = StaticTree::new_inactive(Vector3::zeros());

    assert!(tree.is_empty());

    let size = 100;

    for idx in box_indices(0, size) {
        tree.insert(&idx);
        assert!(tree.at(&idx));
    }
    
    assert!(!tree.is_empty());
}

#[test]
fn test_static_tree_remove() {
    let mut tree = StaticTree::new_active(Vector3::zeros());

    assert!(!tree.is_empty());

    let size = 100;

    for idx in box_indices(0, size) {
        tree.remove(&idx);
        assert!(!tree.at(&idx));
    }
    
    assert!(!tree.is_empty());
}

#[test]
fn test_static_tree_is_empty() {
    let tree = StaticTree::new_active(Vector3::zeros());
    assert!(!tree.is_empty());
    
    let mut tree = StaticTree::new_inactive(Vector3::zeros());
    assert!(tree.is_empty());
    tree.insert(&Vector3::zeros());
    assert!(!tree.is_empty());
}

#[test]
fn test_dynamic_tree_insert() {
    let mut tree = DynamicTree::new();

    assert!(tree.is_empty());

    let size = 100;

    for idx in box_indices(0, size) {
        tree.insert(&idx);
        assert!(tree.at(&idx));
    }
    
    assert!(!tree.is_empty());
}

#[test]
fn test_dynamic_tree_remove() {
    let mut tree = DynamicTree::new();

    assert!(tree.is_empty());

    let size = 100;

    for idx in box_indices(0, size) {
        tree.insert(&idx);
    }

    for idx in box_indices(0, size) {
        tree.remove(&idx);
    }
    
    assert!(tree.is_empty());
}

#[test]
fn test_dynamic_tree_is_empty() {
    let mut tree = DynamicTree::new();
    assert!(tree.is_empty());
    tree.insert(&Vector3::zeros());
    assert!(!tree.is_empty());
}

fn box_indices(start: usize, end: usize) -> impl Iterator<Item = Vector3<usize>> {
    return (start..end).flat_map(move |x| {
        (start..end).flat_map(move |y| {
            (start..end).map(move |z| Vector3::new(x, y, z))
        })
    });
}

