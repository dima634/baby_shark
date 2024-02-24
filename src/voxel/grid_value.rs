use std::ops::Sub;

use crate::helpers::aliases::Vec3f;

use super::{Value, Signed, Sign};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
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

impl Value for Vec3f {}
impl Value for f32 {}
impl Signed for f32 {
    #[inline]
    fn set_sign(&mut self, sign: Sign) {
        let num = match sign {
            Sign::Positive => self.copysign(1.0),
            Sign::Negative => self.copysign(-1.0),
        };
        *self = self.copysign(num);
    }

    #[inline]
    fn sign(&self) -> Sign {
        if self.is_sign_negative() {
            Sign::Negative
        } else {
            Sign::Positive
        }
    }

    #[inline]
    fn far() -> Self {
        f32::MAX
    }
}

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
