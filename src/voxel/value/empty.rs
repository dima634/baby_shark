use crate::voxel::Value;
use std::ops::Sub;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Empty;

impl Value for Empty {}

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

#[cfg(test)]
mod tests {
    use super::Empty;

    #[test]
    fn test_none() {
        assert_eq!(Empty, Empty);
        assert!(Empty <= Empty);
        assert!(Empty >= Empty);
    }
}
