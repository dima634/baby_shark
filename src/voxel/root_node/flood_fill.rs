use super::*;

impl<TChild: TreeNode> FloodFill for RootNode<TChild>
where
    TChild: FloodFill,
    TChild::Value: Signed,
{
    fn flood_fill(&mut self) {
        if self.root.is_empty() {
            return;
        }

        self.root.values_mut().for_each(|c| c.flood_fill());
        let child_origins: Vec<_> = self.root.keys().copied().collect();
        let child_res = TChild::resolution() as isize;

        for (a, b) in child_origins.iter().zip(child_origins.iter().skip(1)) {
            // Check if tiles are on same z-line
            if a.x() != b.x() || a.y() != b.y() || b.z() == a.z() + child_res {
                continue;
            }

            let a_sign = self.root[a].last_value_sign();
            let b_sign = self.root[b].first_value_sign();

            // Is inside?
            if a_sign != Sign::Negative || b_sign != Sign::Negative {
                continue;
            }

            // Add tiles between inside childs
            let mut tile_origin = a.0 + Vec3i::new(0, 0, child_res);

            while tile_origin.z < b.z() {
                let key = Self::root_key(&tile_origin);
                let mut node = TChild::empty(key.0);
                node.fill_with_sign(Sign::Negative);
                self.root.insert(key, node);
                tile_origin.z += child_res;
            }
        }
    }

    fn fill_with_sign(&mut self, _: Sign) {
        unimplemented!("Unsupported operation")
    }

    fn first_value_sign(&self) -> Sign {
        unimplemented!("Unsupported operation")
    }

    fn last_value_sign(&self) -> Sign {
        unimplemented!("Unsupported operation")
    }

    #[inline]
    fn sign_at(&self, index: &Vec3i) -> Sign {
        let root_key = Self::root_key(index);
        self.root
            .get(&root_key)
            .map(|child| child.sign_at(index))
            .unwrap_or(Sign::Positive)
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        dynamic_vdb,
        helpers::aliases::Vec3i,
        voxel::{utils::region, *},
    };

    #[test]
    fn test_flood_fill() {
        type Tree = dynamic_vdb!(f32, 1);

        let mut tree = Tree::empty(Vec3i::zeros());

        let o1 = Vec3i::new(0, 4, 4);
        for i in region(o1, o1.add_scalar(2)) {
            tree.insert(&i, -1.0);
        }

        let o2 = Vec3i::new(0, 4, 10);
        for i in region(o2, o2.add_scalar(2)) {
            tree.insert(&i, -1.0);
        }

        tree.flood_fill();

        for i in region(o1, o2.add_scalar(2)) {
            assert_eq!(tree.sign_at(&i), Sign::Negative);
        }
    }
}
