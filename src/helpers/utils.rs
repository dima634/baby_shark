use std::mem::swap;

/// Sorts three values in ascending order
pub fn sort3<TValue: PartialOrd>(a: &mut TValue, b: &mut TValue, c: &mut TValue) {
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

#[macro_export]
macro_rules! const_map_fn {
    ($name:ident, $src:ty, $dest:ty, $map:path) => {
        pub const fn $name<const SIZE: usize>(array: &[$src; SIZE]) -> [$dest; SIZE] {
            let mut mapped: [$dest; SIZE] =
                unsafe { std::mem::MaybeUninit::uninit().assume_init() };

            let mut i = 0;

            while i < SIZE {
                mapped[i] = $map(array[i]);
                i += 1;
            }

            mapped
        }
    };
}
