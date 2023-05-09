use std::{rc::Rc, ptr::NonNull};

pub struct Link<T> {
    node_ptr: NonNull<Node<T>>
}

impl<T> Clone for Link<T> {
    #[inline]
    fn clone(&self) -> Self {
        return Self { node_ptr: self.node_ptr.clone() };
    }
}

impl<T> Copy for Link<T> {}

impl<T> PartialEq for Link<T> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        return self.node_ptr == other.node_ptr;
    }
}

impl<T> Eq for Link<T> {}

impl<T> Link<T> {
    fn new_node(next: MaybeLink<T>, prev: MaybeLink<T>, value: T) -> Self {
        let node = Node {
            next,
            prev,
            value
        };

        let boxed = Box::new(node);
        return Self {
            node_ptr: Box::leak(boxed).into()
        };
    }
    
    #[inline]
    fn node_ptr(&self) -> *mut Node<T> {
        return self.node_ptr.as_ptr();
    }
}

type MaybeLink<T> = Option<Link<T>>;

pub struct Node<T> {
    value: T,
    next: MaybeLink<T>,
    prev: MaybeLink<T>
}

impl<T> Node<T> {
    #[inline]
    pub fn value(&self) -> &T {
        return &self.value;
    }
}

///
/// Unsafe implementation of linked list based on raw pointers
/// 
pub struct LinkedList<T> {
    head: MaybeLink<T>,
    tail: MaybeLink<T>
}

impl<T> LinkedList<T> {
    pub fn new() -> Self {
        return Self {
            head: None,
            tail: None
        };
    }

    #[inline]
    pub fn empty(&self) -> bool {
        return self.head.is_none();
    }

    pub fn push_front(&mut self, value: T) -> Link<T> {
        unsafe {
            let link = Some(Link::new_node(self.head, None, value));
    
            if let Some(head) = self.head {
                (*head.node_ptr()).prev = link;
            } else {
                self.tail = link;
            }

            self.head = link;
            
            return link.unwrap_unchecked();
        }
    }

    pub fn pop_front(&mut self) -> Option<T> {
        unsafe {
            if let Some(head) = self.head {
                let mut node = Box::from_raw(head.node_ptr());
                self.head = node.next;
                node.next = None;

                if let Some(new_head) = self.head {
                    (*new_head.node_ptr()).prev = None;
                } else {
                    self.tail = None;
                }

                return Some(node.value);
            }

            return None;
        }
    }    
    
    pub fn push_back(&mut self, value: T) -> Link<T> {
        unsafe {
            let link = Some(Link::new_node(None, self.tail, value));

            if let Some(tail) = self.tail {
                (*tail.node_ptr()).next = link;
            } else {
                self.head = link;
            }
            
            self.tail = link;

            return link.unwrap_unchecked();
        }
    }

    pub fn pop_back(&mut self) -> Option<T> {
        unsafe {
            if let Some(tail) = self.tail {
                let mut node = Box::from_raw(tail.node_ptr());
                self.tail = node.prev;
                node.prev = None;

                if let Some(new_tail) = self.tail {
                    (*new_tail.node_ptr()).next = None;
                } else {
                    self.head = None;
                }

                return Some(node.value);
            }

            return None;
        }
    }

    pub fn insert_after(&mut self, after: Link<T>, value: T) -> Link<T> {
        unsafe {
            let after_node = &mut *after.node_ptr();
            debug_assert!(!(after_node.next.is_none() && after_node.next.is_none() && (self.head.is_none() || self.head.unwrap() != after)));

            let after_next = after_node.next;
            let link = Link::new_node(after_next, Some(after), value);
            after_node.next = Some(link);

            if let Some(after_next) = after_next {
                (*after_next.node_ptr()).prev = Some(link);
            } else {
                debug_assert!(after == self.tail.unwrap());
                self.tail = Some(link);
            }

            return link;
        }
    }

    #[inline]
    pub fn iter(&self) -> LinkedListIter<T> {
        return LinkedListIter::new(self);
    }
}

pub struct LinkedListIter<'a, T> {
    current: &'a MaybeLink<T>
}

impl<'a, T> LinkedListIter<'a, T> {
    pub fn new(linked_list: &'a LinkedList<T>) -> Self {
        return Self {
            current: &linked_list.head
        };
    }
}

impl<'a, T> Iterator for LinkedListIter<'a, T> {
    type Item = &'a T;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        match self.current {
            Some(current) => {
                unsafe { 
                    let node = &*current.node_ptr();
                    let value = &node.value;
                    self.current = &node.next;
                    return Some(value);
                }
            },
            None => return None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::LinkedList;

    #[test]
    fn test_linked_list_push_front() {
        let mut ll = LinkedList::new();
        ll.push_front(1);
        ll.push_front(2);
        ll.push_front(3);

        assert!(ll.iter().eq([3, 2, 1].iter()));
    }

    #[test]
    fn test_linked_list_pop_front() {
        let mut ll = LinkedList::new();
        ll.push_front(1);
        ll.push_front(2);
        ll.push_front(3);

        let v = ll.pop_front();
        assert_eq!(v, Some(3));

        let v = ll.pop_front();
        assert_eq!(v, Some(2));

        let v = ll.pop_front();
        assert_eq!(v, Some(1));

        let v = ll.pop_front();
        assert_eq!(v, None);
    }    
    
    #[test]
    fn test_linked_list_push_back() {
        let mut ll = LinkedList::new();
        ll.push_back(1);
        ll.push_back(2);
        ll.push_back(3);

        assert!(ll.iter().eq([1, 2, 3].iter()));
    }

    #[test]
    fn test_linked_list_pop_back() {
        let mut ll = LinkedList::new();
        ll.push_back(1);
        ll.push_back(2);
        ll.push_back(3);

        let v = ll.pop_back();
        assert_eq!(v, Some(3));

        let v = ll.pop_back();
        assert_eq!(v, Some(2));

        let v = ll.pop_back();
        assert_eq!(v, Some(1));

        let v = ll.pop_back();
        assert_eq!(v, None);
    }

    #[test]
    fn test_linked_list_insert() {
        let mut ll = LinkedList::new();
        let first = ll.push_back(1);

        ll.insert_after(first, 2);
        ll.insert_after(first, 3); // 1, 3, 2

        let v = ll.pop_back();
        assert_eq!(v, Some(2));

        let v = ll.pop_back();
        assert_eq!(v, Some(3));

        let v = ll.pop_back();
        assert_eq!(v, Some(1));
    }
}
