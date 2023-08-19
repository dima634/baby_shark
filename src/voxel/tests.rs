use crate::static_vdb;
use crate::voxel::*;

type Tree = static_vdb!(4, 3, 2);

#[test]
fn test_insert() {
    let mut tree = Tree::new_inactive(Vector3::zeros());

    assert!(tree.is_empty());

    let size = 100;

    for x in 0..size {
        for y in 0..size {
            for z in 0..size {
                let idx = Vector3::new(x, y, z);
                tree.insert(&idx);
                assert!(tree.at(&idx));
            }
        }
    }
    
    assert!(!tree.is_empty());
}

#[test]
fn test_remove() {
    let mut tree = Tree::new_active(Vector3::zeros());

    assert!(!tree.is_empty());

    let size = 100;

    for x in 0..size {
        for y in 0..size {
            for z in 0..size {
                let idx = Vector3::new(x, y, z);
                tree.remove(&idx);
                assert!(!tree.at(&idx));
            }
        }
    }
    
    assert!(!tree.is_empty());
}

#[test]
fn test_is_empty() {
    let tree = Tree::new_active(Vector3::zeros());
    assert!(!tree.is_empty());
    
    let mut tree = Tree::new_inactive(Vector3::zeros());
    assert!(tree.is_empty());
    tree.insert(&Vector3::zeros());
    assert!(!tree.is_empty());
}
