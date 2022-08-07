use num_traits::Float;

#[inline]
pub fn hash_float<TFloat: Float>(float: TFloat) -> i32 {
    if float == TFloat::zero() {
        return 0;
    }

    return (float * TFloat::from(73856093).unwrap()).floor().to_i32().unwrap();
}

#[inline]
pub fn combine_hash(h1: i32, h2: i32) -> i32 {
    return i32::wrapping_add(h1 << 5, h1) ^ h2;
}
