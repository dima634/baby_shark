use std::ops::Sub;

use crate::helpers::aliases::Vec3f;

use super::GridValue;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Empty;

impl GridValue for Empty {}

impl Sub for Empty {
    type Output = Self;

    #[inline]
    fn sub(self, _: Self) -> Self {
        self
    }
}

impl From<()> for Empty {
    #[inline]
    fn from(_: ()) -> Self {
        Self
    }
}

impl GridValue for Vec3f {}
impl GridValue for f32 {}

#[cfg(test)]
mod tests {
    use super::Empty;

    #[test]
    fn test_none() {
        assert_eq!(Empty, Empty);
        assert!(!(Empty > Empty));
        assert!(!(Empty < Empty));
    }
}
