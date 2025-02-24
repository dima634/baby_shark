use std::{cell::RefCell, fmt::Display};

pub fn display_option<T: Display>(o: &Option<T>) -> String {
    match o {
        Some(s) => format!("{}", s),
        None => "None".to_string(),
    }
}

pub fn display_refcell<T: Display>(cell: &RefCell<T>) -> String {
    format!("{}", *cell.borrow())
}
