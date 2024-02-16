use std::ops::Sub;

use crate::helpers::aliases::Vec3f;

use super::{GridValue, Signed, ValueSign};

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
impl Signed for f32 {
    #[inline]
    fn set_sign(&mut self, sign: ValueSign) {
        let num = match sign {
            ValueSign::Positive => self.copysign(1.0),
            ValueSign::Negative => self.copysign(-1.0),
        };
        *self = self.copysign(num);
    }

    #[inline]
    fn sign(&self) -> ValueSign {
        if self.is_sign_negative() {
            ValueSign::Negative
        } else {
            ValueSign::Positive
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
