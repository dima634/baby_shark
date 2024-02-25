use super::*;
use crate::{
    helpers::aliases::Vec3f,
    voxel::{Sign, Signed, Value},
};

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

impl Value for Vec3f {}
