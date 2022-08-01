use super::{flags, traits::{TopologyFlags, Face, TopologyPrimitive}};

///
/// Default implementation of Face trait
/// 
struct DefaultFace {
    corner_index: usize,

    flags: flags::TopologyFlags,
    index: usize
}

impl Default for DefaultFace {
    fn default() -> Self {
        return Self {
            corner_index: usize::max_value(),
            index: usize::max_value(),
            flags: Default::default() 
        };
    }
}

impl TopologyFlags for DefaultFace {
    #[inline]
    fn get_flags_mut(&mut self) -> &mut flags::TopologyFlags {
        return &mut self.flags;
    }

    #[inline]
    fn get_flags(&self) -> &flags::TopologyFlags {
        return &self.flags;
    }
}

impl TopologyPrimitive for DefaultFace {
    #[inline]
    fn get_index(&self) ->  usize {
        return self.index;
    }

    #[inline]
    fn set_index(&mut self, index:  usize) -> &mut Self {
        self.index = index;
        return self;
    }
}

impl Face for DefaultFace {
    #[inline]
    fn get_corner_index(&self) ->  usize {
        return self.corner_index;
    }

    fn set_corner_index(&mut self, index:  usize) -> &mut Self {
        self.corner_index = index;
        return self;
    }
}
