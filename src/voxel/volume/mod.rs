pub mod builder;

use std::fs::File;
use std::marker::PhantomData;
use std::path::Path;

use crate::io::stl::StlWriter;
use crate::mesh::polygon_soup::data_structure::PolygonSoup;
use crate::voxel::*;
use crate::{dynamic_vdb, helpers::aliases::Vec3f};

use self::fast_sweep::FastSweeping;
use self::meshing::ActiveVoxelsMesher;

pub(super) type VolumeGrid = dynamic_vdb!(f32, par 5, 4, 3);

#[derive(Debug)]
pub struct Volume {
    grid: Box<VolumeGrid>,
    voxel_size: f32,
}

impl Volume {
    #[inline]
    pub(super) fn new(grid: Box<VolumeGrid>, voxel_size: f32) -> Self {
        Self { grid, voxel_size }
    }

    ///
    /// Creates new SDF grid by evaluating given function on each grid point.
    /// Inside is negative.
    ///
    pub fn from_fn<TFn: Fn(&Vec3f) -> f32>(
        voxel_size: f32,
        min: Vec3f,
        max: Vec3f,
        narrow_band_width: usize,
        func: TFn,
    ) -> Self {
        let mut grid = VolumeGrid::empty(Vec3i::zeros());

        let narrow_band_width = (narrow_band_width + 1) as f32 * voxel_size;
        let min = (min / voxel_size).map(|x| x.floor() as isize);
        let max = (max / voxel_size).map(|x| x.ceil() as isize);

        for x in min.x..=max.x {
            for y in min.y..=max.y {
                for z in min.z..=max.z {
                    let idx = Vec3i::new(x, y, z);
                    let grid_point = idx.cast() * voxel_size;
                    let value = func(&grid_point);

                    if value.abs() > narrow_band_width {
                        continue;
                    }

                    grid.insert(&idx, value);
                }
            }
        }

        // TODO: prune

        Self { grid , voxel_size }
    }

    pub fn union(mut self, mut other: Self) -> Self {
        self.grid.flood_fill();
        other.grid.flood_fill();
        self.grid.union(other.grid);
        self
    }

    pub fn intersect(mut self, mut other: Self) -> Self {
        self.grid.flood_fill();
        other.grid.flood_fill();
        self.grid.intersect(other.grid);
        self
    }

    pub fn subtract(mut self, mut other: Self) -> Self {
        self.grid.flood_fill();
        other.grid.flood_fill();
        self.grid.subtract(other.grid);
        self
    }

    pub fn offset(mut self, distance: f32) -> Self {

        let mut active = ActiveVoxelsMesher::default();
        let mesh = active.mesh(self.grid.as_ref()).into_iter().map(|v| v.cast::<f32>()).collect::<Vec<_>>();
        let ps = PolygonSoup::from_vertices(mesh);
       
        StlWriter::new()
            .write_stl_to_file(&ps, Path::new("active_before.stl"))
            .expect("Write mesh");

        let mut min_max = MinMax::<VolumeGrid> {
            max: f32::MIN,
            min: f32::MAX,
            _tree: PhantomData,
        };
        self.grid.visit_leafs(&mut min_max);

        println!("Min: {}, Max: {}", min_max.min, min_max.max);

        
        let mut iso_level = SelectSignChangeCubes::new(self.grid.as_ref());
        self.grid.visit_leafs(&mut iso_level);
        let new_grid = iso_level.sign_changes();
        self.grid = new_grid;

        //volume_to_nrrd(&self.grid, &Path::new("original.nrrd"));
        
        let sweep = FastSweeping;
        // self.grid.flood_fill();
        sweep.fast_sweeping(self.grid.as_mut(), self.voxel_size, distance + self.voxel_size + self.voxel_size);
    
        let mut offset = Offset::<VolumeGrid>{ value: distance, _tree: PhantomData};
        self.grid.visit_values_mut(&mut offset);


        //volume_to_nrrd(&self.grid, &Path::new("offset.nrrd"));

        let mut active = ActiveVoxelsMesher::default();
        let mesh = active.mesh(self.grid.as_ref()).into_iter().map(|v| v.cast::<f32>()).collect::<Vec<_>>();
        let ps = PolygonSoup::from_vertices(mesh);
       
        StlWriter::new()
            .write_stl_to_file(&ps, Path::new("active.stl"))
            .expect("Write mesh");

        self
    }

    pub(in crate::voxel) fn grid(&self) -> &VolumeGrid {
        // HIDE
        &self.grid
    }
}

fn volume_to_nrrd(volume: &VolumeGrid, path: &Path) {
    use rusty_nrrd::*;

    let mut min_max_idx = MinMaxIdx::<VolumeGrid>::new();
    volume.visit_leafs(&mut min_max_idx);
    let MinMaxIdx { min, max, .. } = min_max_idx;

    println!("Min: {:?}, Max: {:?}", min, max);

    let min = Vec3i::new(-152, -120, -16);
    let max =Vec3i::new(152, 120, 272);

    let sizes = max - min + Vec3i::new(1, 1, 1);
    let background = 1000.0;
    let image = Image::<f32, 3>::new(background, [sizes.x as usize, sizes.y as usize, sizes.z as usize]);

    let mut visitor = VolumeToImage::<VolumeGrid> {
        image,
        min,
        background,
        _tree: PhantomData,
    };

    volume.visit_leafs(&mut visitor);

    let nrrd = Nrrd::try_from(&visitor.image).unwrap();
    write_nrrd(&nrrd, File::create(path).unwrap()).expect("write nrrd");
}

struct Offset<T: TreeNode> {
    value: f32,
    _tree: PhantomData<T>
}

impl<T: TreeNode<Value = f32>> ValueVisitorMut<T::Value> for Offset<T> {
    fn value(&mut self, value: &mut T::Value) {
        *value -= self.value;
    }
}

struct MinMax<T: TreeNode<Value = f32>> {
    min: f32,
    max: f32,
    _tree: PhantomData<T>

}

impl<T: TreeNode<Value = f32>> Visitor<T::Leaf> for MinMax<T> {
    fn tile(&mut self, tile: Tile<<T::Leaf as TreeNode>::Value>) {
        // todo!()
    }

    fn dense(&mut self, dense: &T::Leaf) {
        for val in dense.values() {
            if let Some(val) = val {
                self.min = self.min.min(val);
                self.max = self.max.max(val);
            }
        }
    }
}

use rusty_nrrd::*;
use volume::visitors::select_sign_change_cubes::SelectSignChangeCubes;

struct VolumeToImage<T: TreeNode<Value = f32>> {
    image: Image<f32, 3>,
    min: Vec3i,
    background: f32,
    _tree: PhantomData<T>

}

impl<T: TreeNode<Value = f32>> Visitor<T::Leaf> for VolumeToImage<T> {
    fn tile(&mut self, tile: Tile<<T::Leaf as TreeNode>::Value>) {
        for x in 0..tile.size {
            for y in 0..tile.size {
                for z in 0..tile.size {
                    // if tile.value.sign() == Sign::Positive {
                    //     self.image[[x, y, z]] = 1.0;
                    // } else {
                    //     self.image[[x, y, z]] = -1.0;
                    // }

                    self.image[[x, y, z]] = tile.value;
                }
            }
        }
    }

    fn dense(&mut self, dense: &T::Leaf) {
        for x in 0..T::Leaf::resolution() {
            for y in 0..T::Leaf::resolution() {
                for z in 0..T::Leaf::resolution() {
                    let idx = dense.origin() + Vec3i::new(x as isize, y as isize, z as isize);
                    let shifted = idx - self.min;
                    let shifted_usize = shifted.map(|x| x as usize);

                    let val = dense.at(&idx).copied();
                    // let val = dense.at(&idx).copied().map(|v| if v.sign() == Sign::Positive { 1.0 } else { -1.0 });

                    self.image[[shifted_usize.x, shifted_usize.y, shifted_usize.z]] = val.unwrap_or(self.background);
                }
            }
        }
    }
}


struct MinMaxIdx<T: TreeNode<Value = f32>> {
    min: Vec3i,
    max: Vec3i,
    _tree: PhantomData<T>

}

impl<T: TreeNode<Value = f32>> MinMaxIdx<T> {
    fn new() -> Self {
        Self {
            min: Vec3i::new(isize::MAX, isize::MAX, isize::MAX),
            max: Vec3i::new(isize::MIN, isize::MIN, isize::MIN),
            _tree: PhantomData,
        }
    }
}

impl<T: TreeNode<Value = f32>> Visitor<T::Leaf> for MinMaxIdx<T> {
    fn tile(&mut self, tile: Tile<<T::Leaf as TreeNode>::Value>) {
        todo!()
    }

    fn dense(&mut self, dense: &T::Leaf) {
        self.min = nalgebra_glm::min2(&self.min, &dense.origin());
        self.max = nalgebra_glm::max2(&self.max, &(dense.origin() + Vec3i::new(T::Leaf::resolution() as isize, T::Leaf::resolution() as isize, T::Leaf::resolution() as isize)));
    }
}
