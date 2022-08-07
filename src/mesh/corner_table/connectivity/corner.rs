use super::{traits::{TopologyFlags, TopologyPrimitive, Corner}, flags};

///
/// Default implementation for Corner trait
/// 
#[derive(PartialEq, Eq, Debug)]
pub struct DefaultCorner {
    next_corner_index: usize,
    opposite_corner_index: Option<usize>,
    vertex_index: usize,

    index: usize,
    flags: flags::TopologyFlags
}

impl DefaultCorner {
    pub fn new(
        next_corner_index: usize, 
        opposite_corner_index: Option<usize>,
        vertex_index: usize, 
        index: usize, 
        flags: flags::TopologyFlags
    ) -> Self { 
        return Self { 
            next_corner_index, 
            opposite_corner_index, 
            vertex_index, 
            index, 
            flags 
        };
    }
}

impl Default for DefaultCorner {
    fn default() -> Self {
        return Self { 
            next_corner_index:      usize::max_value(), 
            opposite_corner_index:  None,
            vertex_index:           usize::max_value(), 
            index:                  usize::max_value(), 
            flags:                  Default::default() 
        };
    }
}

impl TopologyFlags for DefaultCorner {
    #[inline]
    fn get_flags_mut(&mut self) -> &mut super::flags::TopologyFlags {
        return &mut self.flags;
    }

    #[inline]
    fn get_flags(&self) -> &super::flags::TopologyFlags {
        return &self.flags;
    }
}

impl TopologyPrimitive for DefaultCorner {
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

impl Corner for DefaultCorner {
    #[inline]
    fn get_next_corner_index(&self) -> usize {
        return self.next_corner_index;
    }

    #[inline]
    fn set_next_corner_index(&mut self, index: usize) -> &Self {
        self.next_corner_index = index;
        return self;
    }

    #[inline]
    fn get_opposite_corner_index(&self) -> Option<usize> {
        return self.opposite_corner_index;
    }

    #[inline]
    fn set_opposite_corner_index(&mut self, index: usize) -> &mut Self {
        self.opposite_corner_index = Some(index);
        return self;
    }

    #[inline]
    fn get_face_index(&self) -> usize {
        return self.get_index() / 3;
    }

    #[inline]
    fn get_vertex_index(&self) -> usize {
        return self.vertex_index;
    }

    #[inline]
    fn set_vertex_index(&mut self, index: usize) -> &mut Self {
        self.vertex_index = index;
        return self;
    }
}
