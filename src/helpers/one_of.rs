#[derive(Debug)]
pub enum OneOf<T1, T2> {
    T1(T1),
    T2(T2),
}
