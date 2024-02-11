use std::ops::Sub;

use crate::helpers::aliases::Vec3f;

use super::{GridValue, IsWithinTolerance};

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

impl IsWithinTolerance for Empty {
    #[inline]
    fn is_within_tolerance(&self, _: Self, _: Self) -> bool {
        true
    }
}

impl From<()> for Empty {
    #[inline]
    fn from(_: ()) -> Self {
        Self
    }
}

#[derive(Debug, Clone, Copy, PartialOrd)]
pub struct Scalar {
    pub value: f32,
}

pub const SMALL_SCALAR: Scalar = Scalar {
    value: SMALL_NUMBER,
};

impl GridValue for Scalar {}

impl Sub for Scalar {
    type Output = Scalar;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            value: self.value - rhs.value,
        }
    }
}

impl IsWithinTolerance for Scalar {
    #[inline]
    fn is_within_tolerance(&self, value: Self, tolerance: Self) -> bool {
        (self.value - value.value).abs() < tolerance.value
    }
}

impl From<f32> for Scalar {
    #[inline]
    fn from(value: f32) -> Self {
        Self { value }
    }
}

impl Into<f32> for Scalar {
    #[inline]
    fn into(self) -> f32 {
        self.value
    }
}

impl Eq for Scalar {}

impl PartialEq for Scalar {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        (self.value - other.value).abs() < SMALL_NUMBER
    }
}

const SMALL_NUMBER: f32 = 1e-6;

impl GridValue for Vec3f {}
impl GridValue for f32 {}

#[cfg(test)]
mod tests {
    use super::{Empty, Scalar};

    #[test]
    fn test_none() {
        assert_eq!(Empty, Empty);
        assert!(!(Empty > Empty));
        assert!(!(Empty < Empty));
    }

    #[test]
    fn test_scalar_eq() {
        assert_eq!(Scalar::from(0.0), Scalar::from(0.0));
        assert_eq!(Scalar::from(-0.0), Scalar::from(0.0));
        assert_eq!(Scalar::from(1.0), Scalar::from(1.000001));
        assert_ne!(Scalar::from(-1.0), Scalar::from(1.000001));
        assert_ne!(Scalar::from(f32::NAN), Scalar::from(1.0));
        assert_ne!(Scalar::from(f32::INFINITY), Scalar::from(f32::MAX));
    }
}
