use std::cell::RefCell;

use nalgebra::Vector3;

use super::Accessor;

pub struct CachedAccessor<'tree> {
    cache: RefCell<Vec<CacheEntry<'tree>>>,
}

impl<'tree> Accessor for CachedAccessor<'tree> {
    fn at(&self, index: &nalgebra::Vector3<usize>) -> bool {
        while let Some(cache) = self.cache.borrow_mut().pop() {
            let key = cache.accessor.index_key(index);

            if cache.key == key {
                let value = cache.accessor.at(index);
                self.cache.borrow_mut().push(cache);
                return value;
            }
        }

        unreachable!()
    }

    fn insert(&mut self, index: &nalgebra::Vector3<usize>) {
        todo!()
    }

    fn remove(&mut self, index: &nalgebra::Vector3<usize>) {
        todo!()
    }

    fn index_key(&self, index: &Vector3<usize>) -> Vector3<usize> {
        unimplemented!()
    }
}

impl<'tree> CachedAccessor<'tree> {
    pub fn new() -> Self {
        Self { cache: RefCell::new(Vec::new()) }
    }
}

struct CacheEntry<'node> {
    key: Vector3<usize>,
    accessor: &'node mut dyn Accessor,
}
