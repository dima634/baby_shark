use std::mem::swap;

pub fn sort3<T: Ord>(a: &mut T, b: &mut T, c: &mut T) {
    if a > c {
        swap(a, c);
    }

    if a > b {
        swap(a, b);
    }

    if b > c {
        swap(b, c);
    }
}