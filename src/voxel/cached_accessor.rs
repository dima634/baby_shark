// use std::{cell::RefCell, marker::PhantomData, ptr::NonNull};

// use nalgebra::Vector3;

// use super::{Accessor, Node, TreeNode};

// pub struct CachedAccessor<'a, T: TreeNode + Node<T::LeafNode>> {
//     cache: RefCell<Vec<CacheEntry<'a, T::LeafNode>>>,
// }

// impl<'a, T: TreeNode + Node<T::LeafNode>> Accessor for CachedAccessor<'a, T> {
//     fn at(&self, index: &Vector3<isize>) -> bool {
//         let mut cache = self.cache.borrow_mut();
        
//         debug_assert!(cache.len() > 0);

//         let key = cache_key(index, T::BRANCHING_TOTAL);
        
//         // while let Some(mut entry) = cache.pop() {
//         //     if entry.key != key {
//         //         continue;
//         //     }
            
//         //     cache.push(entry.clone());

//         //     // if entry.node.is_leaf() { // 171 + 76
//         //     //     return entry.node.at(index);
//         //     // }

//         //     if let Some(value) = entry.node.at_if_leaf(index) { // 117
//         //         return value;
//         //     }

//         //     while let Some(next) = entry.node.next(index) {
//         //         entry = CacheEntry { 
//         //             key: cache_key(index, next.total_branching()),
//         //             node: next,
//         //         };
//         //         cache.push(entry.clone());

//         //         // if next.is_leaf() {
//         //         //     return next.at(index);
//         //         // }

//         //         if let Some(value) = next.at_if_leaf(index) {
//         //             return value;
//         //         }
//         //     }
//         // }



//         loop {
            
//             // println!("get {:?}", index);
//             let mut entry = cache.last().unwrap().clone();

//             if entry.key != key {
//                 println!("pop");
//                 cache.pop();
//                 continue;
//             }

//             // if entry.node.is_leaf() { // 171 + 76
//             //     return entry.node.at(index);
//             // }

//             if let Some(value) = entry.node.at_if_leaf(index) { // 117
//                 return value;
//             }

//             while let Some(next) = entry.node.next(index) {
//                 println!("next");
//                 entry = CacheEntry {
//                     key: cache_key(index, next.total_branching()),
//                     node: next,
//                 };
//                 cache.push(entry.clone());

//                 // if next.is_leaf() {
//                 //     return next.at(index);
//                 // }

//                 if let Some(value) = next.at_if_leaf(index) {
//                     return value;
//                 }
//             }
//         }
//     }

//     fn insert(&mut self, index: &Vector3<isize>) {
//         todo!()
//     }

//     fn remove(&mut self, index: &Vector3<isize>) {
//         todo!()
//     }
// }

// impl<'a, T: TreeNode + Node<T::LeafNode>> CachedAccessor<'a, T> {
//     pub fn new(root: &'a T) -> Self {
//         let mut cache = Vec::with_capacity(5);
//         cache.push(CacheEntry {
//             key: cache_key(&root.origin(), T::BRANCHING_TOTAL),
//             node: root,
//         });

//         Self {
//             cache: RefCell::new(cache)
//         }
//     }
// }

// pub struct CacheEntry<'a, T: Accessor> {
//     key: Vector3<isize>,
//     node: &'a dyn Node<T>,
// }

// impl<'a, T: Accessor> Clone for CacheEntry<'a, T> {
//     #[inline]
//     fn clone(&self) -> Self {
//         Self { 
//             key: self.key.clone(), 
//             node: self.node.clone(),
//         }
//     }
// }

// #[inline]
// fn cache_key(index: &Vector3<isize>, total_branching: usize) -> Vector3<isize> {
//     Vector3::new(
//         index.x & !((1 << total_branching) - 1),
//         index.y & !((1 << total_branching) - 1),
//         index.z & !((1 << total_branching) - 1),
//     )
// }

// #[cfg(test)] 
// mod tests {
//     use crate::{static_vdb, voxel::{utils::box_indices, Accessor}};

//     #[test]
//     fn test_cached_accessor() {
//         type Tree = static_vdb!(4, 3, 2);

//         let size = 10;
//         let mut tree = Tree::new();

//         for idx in box_indices(0, size) {
//             tree.insert(&idx);
//         } 

//         let cached_accessor = tree.cached_accessor();

//         for idx in box_indices(0, size) {
//             let direct = tree.at(&idx);
//             let cached = cached_accessor.at(&idx);

//             assert_eq!(direct, cached);
//         } 
//     }
// }
