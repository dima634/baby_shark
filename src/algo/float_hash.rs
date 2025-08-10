use crate::{geometry::traits::*, helpers::aliases::Vec3};

pub fn hash_float<R: RealNumber>(float: R) -> i32 {
    if float == R::zero() {
        return 0;
    }

    (R::floor(float * R::i32(73856093)) % R::i32(i32::MAX)).as_()
}

pub fn hash_vec3<T: RealNumber>(v: &Vec3<T>) -> i32 {
    let x = hash_float(v.x);
    let y = hash_float(v.y);
    let z = hash_float(v.z);

    combine_hash(combine_hash(x, y), z)
}

#[inline]
pub fn combine_hash(h1: i32, h2: i32) -> i32 {
    i32::wrapping_add(h1 << 5, h1) ^ h2
}
