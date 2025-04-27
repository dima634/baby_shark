use std::fmt::Display;
use bitflags::bitflags;

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct Flags: u8 {
        const IS_DELETED   = 1;
        const IS_VISITED   = 1 << 1;
        const IS_MARKED_1  = 1 << 2;
        const IS_MARKED_2  = 1 << 3;
        const IS_MARKED_3  = 1 << 4;
    }
}

impl Default for Flags {
    #[inline]
    fn default() -> Self {
        Self(Default::default())
    }
}

impl Display for Flags {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:#010b}", self.bits())
    }
}
