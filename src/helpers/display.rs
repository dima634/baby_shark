use std::fmt::Display;

pub fn display_option<T: Display>(o: &Option<T>) -> String {
    match o {
        Some(s) => format!("{}", s),
        None => format!("None"),
    }
}
