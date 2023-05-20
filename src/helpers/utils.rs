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

/// Sort three values by given value
pub fn sort3_by<TValue, TBy: PartialOrd, TGetBy: Fn(&TValue) -> TBy>(a: &mut TValue, b: &mut TValue, c: &mut TValue, by: TGetBy) {
    let mut a_by = by(a);
    let mut b_by = by(b);
    let mut c_by = by(c);

    if a_by > c_by {
        swap(a, c);
        swap(&mut a_by, &mut c_by);
    }

    if a_by > b_by {
        swap(a, b);
        swap(&mut a_by, &mut b_by);
    }

    if b_by > c_by {
        swap(b, c);
        swap(&mut b_by, &mut c_by);
    }
}
