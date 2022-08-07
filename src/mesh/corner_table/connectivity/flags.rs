use bitflags::bitflags;

bitflags! {
    pub struct TopologyFlags: u8 {
        const IS_DELETED = 0b00000001;
        const IS_VISITED = 0b00000010;
    }
}

impl Default for TopologyFlags {
    fn default() -> Self {
        return Self { 
            bits: 0b00000000
        };
    }
}

// pub fn clear_visited<'a, TEntity, TEntitiesIter>(iter: TEntitiesIter) 
//     where TEntity: TopologyPrimitive + 'a, TEntitiesIter: Iterator<Item = &'a mut TEntity> 
// {
//     for entity in iter {
//         entity.set_visited(false);
//     }
// }
