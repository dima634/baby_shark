use super::{Traverse, Accessor, Child, Tile, Leaf};

pub struct LeafsIter<'a, TLeaf: Accessor> {
    stack: Vec<Box<ChildsIter<'a, TLeaf>>>,  // Traversing stack
}

type ChildsIter<'a, TLeaf> = dyn Iterator<Item = Child<'a, TLeaf>> + 'a;

impl<'a, TLeaf: Accessor> LeafsIter<'a, TLeaf> {
    pub fn new(node: &'a dyn Traverse<TLeaf>) -> Self {
        let stack = vec![
            node.childs()
        ];

        Self { stack }
    }
}

impl<'a, TLeaf: Accessor> Iterator for LeafsIter<'a, TLeaf> {
    type Item = Leaf<'a, TLeaf>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.stack.is_empty() {  
            return None;
        }

        let it = self.stack.last_mut().unwrap();
        let next = it.next();

        if next.is_none() {
            self.stack.pop();
            return self.next();
        }

        let next_child = next.unwrap();
        
        match next_child {
            Child::Branch(branch) => {
                self.stack.push(branch.childs());
                self.next()
            }
            Child::Leaf(leaf_node) => Some(Leaf::Dense(leaf_node)),
            Child::Tile(tile) => Some(Leaf::Tile(tile)),
        }
    }
}
