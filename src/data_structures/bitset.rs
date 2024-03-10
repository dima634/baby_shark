use std::ops::{BitAnd, BitAndAssign, BitOr, BitOrAssign, BitXor, BitXorAssign, Not};

#[derive(Debug, Clone, Copy)]
pub struct BitArray<const BITS: usize, const STORAGE_SIZE: usize> {
    storage: [usize; STORAGE_SIZE],
}

impl<const BITS: usize, const STORAGE_SIZE: usize> BitArray<BITS, STORAGE_SIZE> {
    #[inline]
    pub fn zeroes() -> Self {
        Self::with(0)
    }

    #[inline]
    pub fn ones() -> Self {
        Self::with(usize::MAX)
    }

    #[inline]
    pub fn with(value: usize) -> Self {
        debug_assert!(BITS <= usize::BITS as usize * STORAGE_SIZE);
        debug_assert!(Self::unused_bits() < USIZE_BITS);

        Self {
            storage: [value; STORAGE_SIZE],
        }
    }

    #[inline]
    pub const fn unused_bits() -> usize {
        STORAGE_SIZE * USIZE_BITS - BITS
    }

    #[inline]
    pub const fn unused_mask() -> usize {
        usize::MAX << Self::unused_bits()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        let mut value = 0;

        for i in 0..STORAGE_SIZE - 1 {
            value |= self.storage[i];
        }

        value |= self.storage[STORAGE_SIZE - 1] & Self::unused_mask();

        value == 0
    }

    #[inline]
    pub fn is_full(&self) -> bool {
        let mut value = usize::MAX;

        for i in 0..STORAGE_SIZE - 1 {
            value &= self.storage[i];
        }

        value &= self.storage[STORAGE_SIZE - 1] | !Self::unused_mask();

        value == usize::MAX
    }

    #[inline]
    pub fn at(&self, index: usize) -> bool {
        debug_assert!(index < BITS);

        let storage_index = index / USIZE_BITS;
        let value_mask = Self::value_mask(index);

        self.storage[storage_index] & value_mask != 0
    }

    #[inline]
    pub fn set(&mut self, index: usize, value: bool) {
        debug_assert!(index < BITS);

        let storage_index = index / USIZE_BITS;
        let value_mask = Self::value_mask(index);

        if value {
            self.storage[storage_index] |= value_mask;
        } else {
            self.storage[storage_index] &= !value_mask;
        }
    }

    #[inline]
    pub fn on(&mut self, index: usize) {
        self.set(index, true);
    }

    pub fn on_all(&mut self) {
        for i in 0..STORAGE_SIZE {
            self.storage[i] = usize::MAX;
        }
    }

    pub fn find_first_on(&self) -> Option<usize> {
        let mut offset = 0;
        for i in 0..STORAGE_SIZE {
            if self.storage[i] == 0 {
                offset += USIZE_BITS;
                continue;
            }

            let zeroes = self.storage[i].leading_zeros();
            return Some(offset + zeroes as usize);
        }

        None
    }

    #[inline]
    pub fn off(&mut self, index: usize) {
        self.set(index, false);
    }

    pub fn off_all(&mut self) {
        for i in 0..STORAGE_SIZE {
            self.storage[i] = 0;
        }
    }

    #[inline]
    pub fn is_on(&self, index: usize) -> bool {
        self.at(index)
    }

    #[inline]
    pub fn is_off(&self, index: usize) -> bool {
        !self.at(index)
    }

    #[inline]
    pub fn iter(&self) -> impl Iterator<Item = bool> + '_ {
        BitsIter {
            bit_set: self,
            bit: 0,
        }
    }

    #[cfg(target_endian = "little")]
    #[inline]
    fn value_mask(index: usize) -> usize {
        (1 << USIZE_BITS - 1) >> (index % USIZE_BITS)
    }

    #[cfg(target_endian = "big")]
    #[inline]
    fn value_mask(index: usize) -> usize {
        1 >> (index % USIZE_BITS)
    }
}

impl<const BITS: usize, const STORAGE_SIZE: usize> BitAndAssign for BitArray<BITS, STORAGE_SIZE> {
    #[inline]
    fn bitand_assign(&mut self, rhs: Self) {
        for i in 0..STORAGE_SIZE {
            self.storage[i] &= rhs.storage[i];
        }
    }
}

impl<const BITS: usize, const STORAGE_SIZE: usize> BitAnd for BitArray<BITS, STORAGE_SIZE> {
    type Output = Self;

    #[inline]
    fn bitand(self, rhs: Self) -> Self::Output {
        let mut storage: [usize; STORAGE_SIZE] =
            unsafe { std::mem::MaybeUninit::uninit().assume_init() };

        for i in 0..STORAGE_SIZE {
            storage[i] = self.storage[i] & rhs.storage[i];
        }

        Self { storage }
    }
}

impl<const BITS: usize, const STORAGE_SIZE: usize> BitOrAssign for BitArray<BITS, STORAGE_SIZE> {
    #[inline]
    fn bitor_assign(&mut self, rhs: Self) {
        for i in 0..STORAGE_SIZE {
            self.storage[i] |= rhs.storage[i];
        }
    }
}

impl<const BITS: usize, const STORAGE_SIZE: usize> BitOr for BitArray<BITS, STORAGE_SIZE> {
    type Output = Self;

    #[inline]
    fn bitor(self, rhs: Self) -> Self::Output {
        let mut storage: [usize; STORAGE_SIZE] =
            unsafe { std::mem::MaybeUninit::uninit().assume_init() };

        for i in 0..STORAGE_SIZE {
            storage[i] = self.storage[i] | rhs.storage[i];
        }

        Self { storage }
    }
}

impl<const BITS: usize, const STORAGE_SIZE: usize> BitXorAssign for BitArray<BITS, STORAGE_SIZE> {
    #[inline]
    fn bitxor_assign(&mut self, rhs: Self) {
        for i in 0..STORAGE_SIZE {
            self.storage[i] ^= rhs.storage[i];
        }
    }
}

impl<const BITS: usize, const STORAGE_SIZE: usize> BitXor for BitArray<BITS, STORAGE_SIZE> {
    type Output = Self;

    #[inline]
    fn bitxor(self, rhs: Self) -> Self::Output {
        let mut storage: [usize; STORAGE_SIZE] =
            unsafe { std::mem::MaybeUninit::uninit().assume_init() };

        for i in 0..STORAGE_SIZE {
            storage[i] = self.storage[i] ^ rhs.storage[i];
        }

        Self { storage }
    }
}

impl<const BITS: usize, const STORAGE_SIZE: usize> Not for BitArray<BITS, STORAGE_SIZE> {
    type Output = Self;

    fn not(self) -> Self::Output {
        let mut storage: [usize; STORAGE_SIZE] =
            unsafe { std::mem::MaybeUninit::uninit().assume_init() };

        for i in 0..STORAGE_SIZE {
            storage[i] = !self.storage[i];
        }

        Self { storage }
    }
}

impl<const BITS: usize, const STORAGE_SIZE: usize> PartialEq for BitArray<BITS, STORAGE_SIZE> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        let mut xor = 0;

        for i in 0..STORAGE_SIZE - 1 {
            xor |= self.storage[i] ^ other.storage[i];
        }

        xor |= (self.storage[STORAGE_SIZE - 1] & Self::unused_mask())
            ^ (other.storage[STORAGE_SIZE - 1] & Self::unused_mask());

        xor == 0
    }
}

impl<const BITS: usize, const STORAGE_SIZE: usize> Eq for BitArray<BITS, STORAGE_SIZE> {}

pub struct BitsIter<'a, const BITS: usize, const STORAGE_SIZE: usize> {
    bit_set: &'a BitArray<BITS, STORAGE_SIZE>,
    bit: usize,
}

impl<'a, const BITS: usize, const STORAGE_SIZE: usize> Iterator
    for BitsIter<'a, BITS, STORAGE_SIZE>
{
    type Item = bool;

    fn next(&mut self) -> Option<Self::Item> {
        if self.bit == BITS {
            return None;
        }

        let value = self.bit_set.at(self.bit);
        self.bit += 1;

        Some(value)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (BITS, Some(BITS))
    }

    fn count(self) -> usize
    where
        Self: Sized,
    {
        BITS
    }
}

const USIZE_BITS: usize = usize::BITS as usize;

#[cfg(test)]
mod tests {
    use super::{BitArray, USIZE_BITS};

    #[test]
    #[should_panic]
    fn test_new_with_redundant_space_allocated1() {
        BitArray::<0, 1>::ones();
    }

    #[test]
    #[should_panic]
    fn test_new_with_redundant_space_allocated2() {
        BitArray::<{ USIZE_BITS - 1 }, 2>::ones();
    }

    #[test]
    #[should_panic]
    fn test_new_with_redundant_space_allocated3() {
        BitArray::<USIZE_BITS, 2>::ones();
    }

    #[test]
    fn test_unused_bits() {
        assert_eq!(BitArray::<1, 1>::unused_bits(), USIZE_BITS - 1);
        assert_eq!(BitArray::<2, 1>::unused_bits(), USIZE_BITS - 2);
        assert_eq!(BitArray::<USIZE_BITS, 1>::unused_bits(), 0);
        assert_eq!(BitArray::<{ USIZE_BITS - 1 }, 1>::unused_bits(), 1);
        assert_eq!(BitArray::<{ USIZE_BITS * 2 }, 2>::unused_bits(), 0);
    }

    #[test]
    fn test_unused_mask() {
        assert_eq!(BitArray::<1, 1>::unused_mask(), 0b1 << (USIZE_BITS - 1));
        assert_eq!(BitArray::<2, 1>::unused_mask(), 0b11 << (USIZE_BITS - 2));
        assert_eq!(BitArray::<3, 1>::unused_mask(), 0b111 << (USIZE_BITS - 3));
        assert_eq!(BitArray::<USIZE_BITS, 1>::unused_mask(), usize::MAX);
        assert_eq!(
            BitArray::<{ USIZE_BITS - 1 }, 1>::unused_mask(),
            usize::MAX << 1
        );
        assert_eq!(
            BitArray::<{ USIZE_BITS + 1 }, 2>::unused_mask(),
            0b1 << (USIZE_BITS - 1)
        );
        assert_eq!(
            BitArray::<{ USIZE_BITS + 2 }, 2>::unused_mask(),
            0b11 << (USIZE_BITS - 2)
        );
    }

    #[test]
    fn test_is_full() {
        assert!(BitArray::<1, 1>::ones().is_full());
        assert!(BitArray::<USIZE_BITS, 1>::ones().is_full());
    }

    #[test]
    fn test_is_empty() {
        assert!(BitArray::<1, 1>::zeroes().is_empty());
        assert!(BitArray::<USIZE_BITS, 1>::zeroes().is_empty());
    }

    #[test]
    fn test_find_first_on() {
        let set = BitArray::<10, 1>::zeroes();
        assert_eq!(set.find_first_on(), None);

        let set = BitArray::<10, 1>::ones();
        assert_eq!(set.find_first_on(), Some(0));

        let mut set = BitArray::<10, 1>::zeroes();
        set.on(3);
        assert_eq!(set.find_first_on(), Some(3));

        set.on(4);
        assert_eq!(set.find_first_on(), Some(3));
        
        set.on(2);
        assert_eq!(set.find_first_on(), Some(2));
    }

    #[test]
    fn test_eq() {
        let set1 = BitArray::<10, 1>::ones();
        let set2 = BitArray::<10, 1>::zeroes();
        assert_ne!(set1, set2);

        let set1 = BitArray::<10, 1>::ones();
        let mut set2 = BitArray::<10, 1>::ones();
        set2.off(5);
        assert_ne!(set1, set2);

        let mut set1 = BitArray::<10, 1>::ones();
        set1.off(1);
        let mut set2 = BitArray::<10, 1>::ones();
        set2.off(5);
        assert_ne!(set1, set2);

        let set1 = BitArray::<10, 1>::ones();
        let set2 = BitArray::<10, 1>::ones();
        assert_eq!(set1, set2);

        let mut set1 = BitArray::<10, 1>::zeroes();
        set1.on(3);
        let mut set2 = BitArray::<10, 1>::zeroes();
        set2.on(3);
        assert_eq!(set1, set2);
    }
}
