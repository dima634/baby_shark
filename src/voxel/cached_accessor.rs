use super::Accessor;

pub struct CachedAccessor<'tree> {
    cache: Vec<&'tree dyn Accessor>,
}
