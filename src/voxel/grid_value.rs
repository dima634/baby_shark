use super::GridValue;


#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd)]
pub struct Empty;

impl GridValue for Empty {}

impl From<()> for Empty {
    #[inline]
    fn from(_: ()) -> Self {
        Self
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Scalar {
    value: f32,
}

impl GridValue for Scalar {}

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

impl PartialEq for Scalar {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        (self.value - other.value).abs() < SMALL_NUMBER
    }
}

impl GridValue for () {}

const SMALL_NUMBER: f32 = 1e-6;

mod tests {
    use super::{Scalar, Empty};

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
