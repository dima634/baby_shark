use crate::{helpers::aliases::Vec3i, voxel::{utils::CUBE_OFFSETS, Signed, TreeNode, Visitor}};

pub struct KeepSignChangeCubes<'tree, T: TreeNode> {
    tree: &'tree T,
    sign_changes: Box<T>,
}

impl<'tree, T: TreeNode> KeepSignChangeCubes<'tree, T> {
    pub fn new(tree: &'tree T) -> Self {
        Self {
            tree,
            sign_changes: T::empty(Vec3i::zeros()),
        }
    }

    pub fn sign_changes(self) -> Box<T> {
        self.sign_changes
    }
}

impl<'tree, TTree> Visitor<TTree::Leaf> for KeepSignChangeCubes<'tree, TTree>
where
    TTree: TreeNode,
    TTree::Value: Signed
{
    fn tile(&mut self, _: crate::voxel::Tile<<TTree::Leaf as TreeNode>::Value>) {
        todo!()
    }

    fn dense(&mut self, dense: &TTree::Leaf) {
        let min = dense.origin();
        let size = TTree::Leaf::resolution() as isize;
        let max = Vec3i::new(min.x + size, min.y + size, min.z + size);

        for x in min.x..max.x {
            for y in min.y..max.y {
                'next_voxel: for z in min.z..max.z {
                    let voxel_idx = Vec3i::new(x, y, z);
                    let value = match self.tree.at((&voxel_idx)) {
                        Some(v) => *v,
                        None => continue,
                    };
                    let sign = value.sign();

                    let mut has_sign_change = false;
                    for off in CUBE_OFFSETS {
                        let neighbor_idx = voxel_idx + off;

                        match self.tree.at(&neighbor_idx) {
                            Some(neighbor) => {
                                let neighbor_sign = neighbor.sign();
                                if  sign != neighbor_sign {
                                    has_sign_change = true;
                                    break;
                                }
                            },
                            None => continue 'next_voxel,
                        };
                    }

                    if !has_sign_change {
                        continue;
                    }

                    for off in CUBE_OFFSETS {
                        let neighbor_idx = voxel_idx + off;

                        match self.tree.at(&neighbor_idx) {
                            Some(neighbor) => {
                                // println!("Inserting sign change at {:?}", neighbor_idx);
                                self.sign_changes.insert(&neighbor_idx, *neighbor);
                            },
                            None => continue 'next_voxel,
                        };
                    }
                }
            }
        }
    }
}