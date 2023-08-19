/// Check if all bits in mask are zeroes
pub fn is_mask_empty<const SIZE: usize>(mask: &[usize]) -> bool {
    let mut value = 0;

    for i in 0..SIZE {
        value |= mask[i];
    }

    return value == 0;
}
