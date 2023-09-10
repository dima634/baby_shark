// use std::{cell::RefCell, marker::PhantomData, ptr::NonNull};

// use nalgebra::Vector3;

// use super::{Accessor};

// pub struct CachedAccessor<'accessor> {
//     cache: RefCell<Vec<CacheEntry>>,
//     lifetime: PhantomData<&'accessor ()>,
// }

// // pub trait TreeTraverse: Accessor {
// //     fn visit_and_cache(&mut self, index: &Vector3<usize>, cache: &mut Vec<CacheEntry>);
// // }

// impl<'accessor> CachedAccessor<'accessor> {
//     pub fn new(root: NonNull<dyn TreeTraverse>) -> Self {
//         let key = unsafe { root.as_ref().index_key(&Vector3::zeros()) };
//         Self {
//             cache: RefCell::new(vec![ 
//                 CacheEntry {
//                     key,
//                     accessor: root,
//                 }
//             ]),
//             lifetime: PhantomData,
//         }
//     }
// }

// impl<'tree> Accessor for CachedAccessor<'tree> {
//     fn at(&self, index: &Vector3<usize>) -> bool {
//         let mut cache = self.cache.borrow_mut();

//         while let Some(mut cached) = cache.pop() {
//             let key = cached.accessor().index_key(index);

//             if cached.key == key || cache.is_empty() {
//                 cached.accessor_mut().visit_and_cache(index, &mut cache);
//                 let value = cache.last().unwrap().accessor().at(index);
//                 cache.push(cached);

//                 return value;
//             }
//         }

//         unreachable!()
//     }

//     fn insert(&mut self, index: &Vector3<usize>) {
//         todo!()
//     }

//     fn remove(&mut self, index: &Vector3<usize>) {
//         todo!()
//     }

//     fn index_key(&self, index: &Vector3<usize>) -> Vector3<usize> {
//         unimplemented!()
//     }
// }

// pub struct CacheEntry {
//     key: Vector3<usize>,
//     accessor: NonNull<dyn TreeTraverse>,
// }

// impl CacheEntry {
//     pub fn accessor(&self) -> &dyn TreeTraverse {
//         unsafe { self.accessor.as_ref() }
//     }
//     pub fn accessor_mut(&mut self) -> &mut dyn TreeTraverse {
//         unsafe { self.accessor.as_mut() }
//     }
// }

// #[cfg(test)] 
// mod tests {
//     // #[test]
//     // fn test_cached_accessor() {
//     //     let 
//     // }
// }
