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
pub struct LinkedList<T> {
    head: Option<Link>, // First node
    tail: Option<Link>, // Last node
    free: Option<Link>, // Singly-linked list of free nodes
    vec: Vec<Node<T>>,  // Nodes storage
    len: usize
}

impl<T> LinkedList<T> {
    /// Creates empty linked list
    pub fn new() -> Self {
        return Self { 
            head: None, 
            tail: None,
            free: None,
            vec: Vec::new(),
            len: 0
        };
    }

    /// Returns number of elements in list
    #[inline]
    pub fn len(&self) -> usize {
        return self.len;
    }

    /// Reserve storage for additional number of elements 
    #[inline]
    pub fn reserve(&mut self, additional: usize) {
        self.vec.reserve(additional);
    }
    
    /// Removes all elements from list
    #[inline]
    pub fn clear(&mut self) {
        self.head = None;
        self.tail = None;
        self.free = None;
        self.vec.clear();
    }

    /// Returns head (first element) of linked list
    #[inline]
    pub fn head(&self) -> Option<Link> {
        return self.head;
    }

    /// Returns tail (last element) of linked list
    #[inline]
    pub fn tail(&self) -> Option<Link> {
        return self.tail;
    }

    /// Returns element after `element`
    #[inline]
    pub fn next(&self, element: Link) -> Option<Link> {
        debug_assert!(!self.is_free(element));
        return self.node(element).next;
    }
    
    /// Returns element after `element`. If `element` is last element in the list returns head.
    #[inline]
    pub fn next_circular(&self, element: Link) -> Option<Link> {
        debug_assert!(!self.is_free(element));
        return self.node(element).next.or(self.head);
    }

    /// Returns element before `element`
    #[inline]
    pub fn prev(&self, element: Link) -> Option<Link> {
        debug_assert!(!self.is_free(element));
        return self.node(element).prev;
    }

    /// Returns element before `element`. If `element` is first element in the list returns tail.
    #[inline]
    pub fn prev_circular(&self, element: Link) -> Option<Link> {
        debug_assert!(!self.is_free(element));
        return self.node(element).prev.or(self.tail);
    }
    
    /// Returns value saved in `element`
    #[inline]
    pub fn value(&self, element: Link) -> &T {
        debug_assert!(!self.is_free(element));
        return &self.node(element).value;
    }

    /// Insert new element at the beginning of the list. Returns link to created element
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

    /// Remove first element in the list
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
    
    /// Insert new element at the end of the list. Returns link to created element
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

    /// Remove last element in the list
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

    /// Insert new element right after `element`. Returns link to created element
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

    /// Insert new element right before `element`. Returns link to created element
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


    /// Remove `element` from the list
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


    /// Returns iterator over all element in the list. From head to tail
    #[inline]
    pub fn iter(&self) -> ForwardIter<T> {
        return LinkedVecIter::from_head(self);
    }

    /// Returns iterator over all values in the list. From head to tail
    #[inline]
    pub fn values(&self) -> impl Iterator<Item = &T> {
        return self.iter().map(|l| &self[l]);
    }

    /// Returns iterator over all element before `element` (including it). From `element` to head
    #[inline]
    pub fn before(&self, element: Link) -> BackwardIter<T> {
        return LinkedVecIter::from_node(self, Some(element));
    }

    /// Returns iterator over all element after `element` (including it). From `element` to tail
    #[inline]
    pub fn after(&self, element: Link) -> ForwardIter<T> {
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

    /// Create new node between `prev` and `next`
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

    /// Release given node
    fn release(&mut self, link: Link) {
        // Remove element from the list and insert into free nodes list
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

impl<T> Index<Link> for LinkedList<T> {
    type Output = T;

    #[inline]
    fn index(&self, index: Link) -> &Self::Output {
        return self.value(index);
    }
}

const FORWARD: bool = true;
const BACKWARD: bool = false;

/// Directional iterator over linked list
pub struct LinkedVecIter<'a, T, const FORWARD: bool> {
    linked_vec: &'a LinkedList<T>,
    current: Option<Link>
}

impl<'a, T, const FORWARD: bool> LinkedVecIter<'a, T, FORWARD> {
    #[inline]
    pub fn from_head(linked_vec: &'a LinkedList<T>) -> Self {
        return Self::from_node(linked_vec, linked_vec.head);
    }
    
    pub fn from_node(linked_vec: &'a LinkedList<T>, node: Option<Link>) -> Self {
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

pub type ForwardIter<'a, T> = LinkedVecIter<'a, T, FORWARD>;
pub type BackwardIter<'a, T> = LinkedVecIter<'a, T, BACKWARD>;

#[cfg(test)]
mod tests {
    use super::{LinkedList, Link};

    #[test]
    fn test_linked_vec_push_front() {
        let mut ll = LinkedList::new();
        assert_eq!(ll.push_front(1), Link(0));
        assert_eq!(ll.push_front(2), Link(1));
        assert_eq!(ll.push_front(3), Link(2));

        assert!(ll.values().eq([3, 2, 1].iter()));
    }

    #[test]
    fn test_linked_vec_push_front_reuse_free() {
        let mut ll = LinkedList::new();
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
        let mut ll = LinkedList::new();
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
        let mut ll = LinkedList::new();
        ll.push_back(1);
        ll.push_back(2);
        ll.push_back(3);

        assert!(ll.values().eq([1, 2, 3].iter()));
    }

    #[test]
    fn test_linked_vec_pop_back() {
        let mut ll = LinkedList::new();
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
        let mut ll = LinkedList::new();
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
        let mut ll = LinkedList::new();
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
        let mut ll = LinkedList::new();
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
