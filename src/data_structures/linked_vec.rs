use std::ops::Index;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Link(usize);

#[derive(Debug)]
struct Node<T> {
    next: Option<Link>,
    prev: Option<Link>,
    value: T
}

///
/// Doubly-linked list based on `Vec`
/// 
#[derive(Debug)]
pub struct LinkedVec<T> {
    head: Option<Link>, // First node
    tail: Option<Link>, // Last node
    free: Option<Link>, // Singly-linked list of free nodes
    vec: Vec<Node<T>>,  // Nodes storage
    len: usize
}

impl<T> LinkedVec<T> {
    pub fn new() -> Self {
        return Self { 
            head: None, 
            tail: None,
            free: None,
            vec: Vec::new(),
            len: 0
        };
    }

    #[inline]
    pub fn len(&self) -> usize {
        return self.len;
    }

    #[inline]
    pub fn reserve(&mut self, additional: usize) {
        self.vec.reserve(additional);
    }

    #[inline]
    pub fn clear(&mut self) {
        self.head = None;
        self.tail = None;
        self.vec.clear();
    }

    #[inline]
    pub fn head(&self) -> Option<Link> {
        return self.head;
    }

    #[inline]
    pub fn tail(&self) -> Option<Link> {
        return self.tail;
    }

    #[inline]
    pub fn next(&self, element: Link) -> Option<Link> {
        debug_assert!(!self.is_free(element));
        return self.node(element).next;
    }
    
    #[inline]
    pub fn next_circular(&self, element: Link) -> Option<Link> {
        debug_assert!(!self.is_free(element));
        return self.node(element).next.or(self.head);
    }

    #[inline]
    pub fn prev(&self, element: Link) -> Option<Link> {
        debug_assert!(!self.is_free(element));
        return self.node(element).prev;
    }

    #[inline]
    pub fn prev_circular(&self, element: Link) -> Option<Link> {
        debug_assert!(!self.is_free(element));
        return self.node(element).prev.or(self.tail);
    }
    
    #[inline]
    pub fn value(&self, element: Link) -> &T {
        debug_assert!(!self.is_free(element));
        return &self.node(element).value;
    }

    pub fn push_front(&mut self, value: T) -> Link {
        let node = self.new_node(self.head, None, value);

        if let Some(head) = self.head {
            self.node_mut(head).prev = Some(node);
        } else {
            self.tail = Some(node);
        }

        self.head = Some(node);
        
        return node;
    }

    pub fn pop_front(&mut self) -> Option<&T> {
        return self.head.map(|head| {
            self.head = self.node(head).next;

            if let Some(new_head) = self.head {
                self.node_mut(new_head).prev = None;
            } else {
                self.tail = None;
            }

            self.release(head);
            
            return &self.node(head).value;
        });
    }    
    
    pub fn push_back(&mut self, value: T) -> Link {
        let node = self.new_node(None, self.tail, value);

        if let Some(tail) = self.tail {
            self.node_mut(tail).next = Some(node);
        } else {
            self.head = Some(node);
        }

        self.tail = Some(node);
        
        return node;
    }

    pub fn pop_back(&mut self) -> Option<&T> {
        return self.tail.map(|tail| {
            self.tail = self.node(tail).prev;

            if let Some(new_tail) = self.tail {
                self.node_mut(new_tail).next = None;
            } else {
                self.head = None;
            }

            self.release(tail);
            
            return &self.node(tail).value;
        });
    }

    pub fn insert_after(&mut self, after: Link, value: T) -> Link {
        debug_assert!(!self.is_free(after));
        
        if let Some(after_next) = self.node(after).next {
            let node = self.new_node(Some(after_next), Some(after), value);
            self.node_mut(after).next = Some(node);
            self.node_mut(after_next).prev = Some(node);
            return node;
        } else {
            return self.push_back(value);
        }
    }

    pub fn insert_before(&mut self, before: Link, value: T) -> Link {
        debug_assert!(!self.is_free(before));
        
        if let Some(before_prev) = self.node(before).prev {
            let node = self.new_node(Some(before), Some(before_prev), value);
            self.node_mut(before).prev = Some(node);
            self.node_mut(before_prev).next = Some(node);
            return node;
        } else {
            return self.push_front(value);
        }
    }

    pub fn remove(&mut self, element: Link) -> &T {
        debug_assert!(!self.is_free(element));
 
        if Some(element) == self.head {
            return self.pop_front().unwrap();
        }

        if Some(element) == self.tail {
            return self.pop_back().unwrap();
        }

        let node = self.node(element);
        let next = node.next.unwrap();
        let prev = node.prev.unwrap();
        let next_node = self.node_mut(next);
        next_node.prev = Some(prev);
        let prev_node = self.node_mut(prev);
        prev_node.next = Some(next);

        self.release(element);

        return &self.node(element).value;
    }

    #[inline]
    pub fn iter(&self) -> LinkedVecIter<T, FORWARD> {
        return LinkedVecIter::from_head(self);
    }

    #[inline]
    pub fn values(&self) -> impl Iterator<Item = &T> {
        return self.iter().map(|l| &self[l]);
    }

    #[inline]
    pub fn before(&self, element: Link) -> LinkedVecIter<T, BACKWARD> {
        return LinkedVecIter::from_node(self, Some(element));
    }

    #[inline]
    pub fn after(&self, element: Link) -> LinkedVecIter<T, FORWARD> {
        return LinkedVecIter::from_node(self, Some(element));
    }

    #[inline]
    fn node(&self, link: Link) -> &Node<T> {
        return &self.vec[link.0];
    }

    #[inline]
    fn node_mut(&mut self, link: Link) -> &mut Node<T> {
        return &mut self.vec[link.0];
    }

    fn new_node(&mut self, next: Option<Link>, prev: Option<Link>,  value: T) -> Link {
        let new_node = Node {
            next,
            prev,
            value
        };
        self.len += 1;

        if let Some(free) = self.free {
            // Reuse free node
            self.free = self.node(free).prev;
            self.vec[free.0] = new_node;
            return free;
        } else {
            // Push new node
            let link = Link(self.vec.len());
            self.vec.push(new_node);
            return link;
        }
    }

    fn release(&mut self, link: Link) {
        let free = self.free;
        let node = self.node_mut(link);
        node.next = None;
        node.prev = free;
        self.free = Some(link);
        self.len -= 1;
    }

    fn is_free(&self, link: Link) -> bool {
        let node = self.node(link);
        let link = Some(link);
        return 
            link != self.tail && 
            link != self.head && 
            (
                link == self.free ||
                (
                    node.prev.is_some() &&
                    node.next.is_none()
                )
            );
    }
}

impl<T> Index<Link> for LinkedVec<T> {
    type Output = T;

    #[inline]
    fn index(&self, index: Link) -> &Self::Output {
        return self.value(index);
    }
}

const FORWARD: bool = true;
const BACKWARD: bool = false;

pub struct LinkedVecIter<'a, T, const FORWARD: bool> {
    linked_vec: &'a LinkedVec<T>,
    current: Option<Link>
}

impl<'a, T, const FORWARD: bool> LinkedVecIter<'a, T, FORWARD> {
    #[inline]
    pub fn from_head(linked_vec: &'a LinkedVec<T>) -> Self {
        return Self::from_node(linked_vec, linked_vec.head);
    }
    
    pub fn from_node(linked_vec: &'a LinkedVec<T>, node: Option<Link>) -> Self {
        return Self {
            linked_vec,
            current: node
        };
    }
}

impl<'a, T, const FORWARD: bool> Iterator for LinkedVecIter<'a, T, FORWARD> {
    type Item = Link;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        return self.current.map(|current| {
            let node = self.linked_vec.node(current);

            if FORWARD {
                self.current = node.next;
            } else {
                self.current = node.prev;
            }

            return current;
        });
    }
}

impl<'a, T, const FORWARD: bool> ExactSizeIterator for LinkedVecIter<'a, T, FORWARD> {
    #[inline]
    fn len(&self) -> usize {
        return self.linked_vec.len();
    }
}

#[cfg(test)]
mod tests {
    use super::{LinkedVec, Link};

    #[test]
    fn test_linked_vec_push_front() {
        let mut ll = LinkedVec::new();
        assert_eq!(ll.push_front(1), Link(0));
        assert_eq!(ll.push_front(2), Link(1));
        assert_eq!(ll.push_front(3), Link(2));

        assert!(ll.values().eq([3, 2, 1].iter()));
    }

    #[test]
    fn test_linked_vec_push_front_reuse_free() {
        let mut ll = LinkedVec::new();
        ll.push_front(0);
        ll.push_front(1);
        ll.push_front(2);

        ll.pop_front(); // 2
        ll.pop_front(); // 1
        assert_eq!(ll.push_front(3), Link(1));
        assert_eq!(ll.push_front(4), Link(2));
    }

    #[test]
    fn test_linked_vec_pop_front() {
        let mut ll = LinkedVec::new();
        ll.push_front(1);
        ll.push_front(2);
        ll.push_front(3);

        let v = ll.pop_front();
        assert_eq!(v, Some(&3));

        let v = ll.pop_front();
        assert_eq!(v, Some(&2));

        let v = ll.pop_front();
        assert_eq!(v, Some(&1));

        let v = ll.pop_front();
        assert_eq!(v, None);
    }    
    
    #[test]
    fn test_linked_vec_push_back() {
        let mut ll = LinkedVec::new();
        ll.push_back(1);
        ll.push_back(2);
        ll.push_back(3);

        assert!(ll.values().eq([1, 2, 3].iter()));
    }

    #[test]
    fn test_linked_vec_pop_back() {
        let mut ll = LinkedVec::new();
        ll.push_back(1);
        ll.push_back(2);
        ll.push_back(3);

        let v = ll.pop_back();
        assert_eq!(v, Some(&3));

        let v = ll.pop_back();
        assert_eq!(v, Some(&2));

        let v = ll.pop_back();
        assert_eq!(v, Some(&1));

        let v = ll.pop_back();
        assert_eq!(v, None);
    }

    #[test]
    fn test_linked_vec_insert_after() {
        let mut ll = LinkedVec::new();
        let first = ll.push_back(1);

        ll.insert_after(first, 2);
        ll.insert_after(first, 3); // 1, 3, 2

        let v = ll.pop_back();
        assert_eq!(v, Some(&2));

        let v = ll.pop_back();
        assert_eq!(v, Some(&3));

        let v = ll.pop_back();
        assert_eq!(v, Some(&1));
    }

    #[test]
    fn test_linked_vec_insert_before() {
        let mut ll = LinkedVec::new();
        let first = ll.push_back(1);

        ll.insert_before(first, 2);
        ll.insert_before(first, 3); // 2, 3, 1

        let v = ll.pop_front();
        assert_eq!(v, Some(&2));

        let v = ll.pop_front();
        assert_eq!(v, Some(&3));

        let v = ll.pop_front();
        assert_eq!(v, Some(&1));
    }

    #[test]
    fn test_linked_vec_remove() {
        let mut ll = LinkedVec::new();
        ll.push_back(0);
        ll.push_back(1);
        ll.push_back(2);
        ll.push_back(3);

        ll.remove(Link(2));
        assert!(ll.values().eq([0, 1, 3].iter()));

        ll.remove(Link(3));
        assert!(ll.values().eq([0, 1].iter()));

        ll.remove(Link(0));
        assert!(ll.values().eq([1].iter()));
    }
}
