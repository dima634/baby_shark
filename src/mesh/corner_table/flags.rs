use std::fmt::Display;
use bitflags::bitflags;

use super::traits;

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct Flags: u8 {
        const IS_DELETED   = 0b00000001;
        const IS_VISITED   = 0b00000010;
        const IS_MARKED_1  = 0b10000000;
        const IS_MARKED_2  = 0b01000000;
        const IS_MARKED_3  = 0b00100000;
    }
}

impl Default for Flags {
    #[inline]
    fn default() -> Self {
        Self(Default::default())
    }
}

///
/// Sets visited flag to `false`
/// 
#[inline]
pub fn clear_visited<'a, TEntity, TEntitiesIter>(iter: TEntitiesIter) 
    where TEntity: 'a + traits::Flags, 
    TEntitiesIter: Iterator<Item = &'a TEntity> 
{
    for entity in iter {
        entity.set_visited(false);
    }
}

///
/// Sets all 'marked' flags to `false`
/// 
#[inline]
pub fn clear_marked<'a, TEntity, TEntitiesIter>(iter: TEntitiesIter) 
    where TEntity: 'a + traits::Flags, 
    TEntitiesIter: Iterator<Item = &'a TEntity> 
{
    for entity in iter {
        entity.set_marked_1(false);
        entity.set_marked_2(false);
        entity.set_marked_3(false);
    }
}

impl Display for Flags {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:#010b}", self.bits())
    }
}
