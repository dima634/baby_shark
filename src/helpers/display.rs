use std::{fmt::Display, cell::UnsafeCell};

pub fn display_option<T: Display>(o: &Option<T>) -> String {
    match o {
        Some(s) => format!("{}", s),
        None => format!("None"),
    }
}

pub fn display_unsafecell<T: Display>(cell: &UnsafeCell<T>) -> String {
    unsafe {
        format!("{}", *cell.get())
    }
}
