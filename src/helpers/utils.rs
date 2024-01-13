use std::mem::{swap, MaybeUninit};

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

#[macro_export]
macro_rules! const_map_fn {
    ($name:ident, $src:ty, $dest:ty, $map:path) => {
        pub const fn $name<const SIZE: usize>(array: &[$src; SIZE]) -> [$dest; SIZE] {
            let mut mapped: [$dest; SIZE] = unsafe { std::mem::MaybeUninit::uninit().assume_init() };
        
            let mut i = 0;
        
            while i < SIZE {
                mapped[i] = $map(array[i]);
                i += 1;
            }
        
            mapped
        }
    };
}
