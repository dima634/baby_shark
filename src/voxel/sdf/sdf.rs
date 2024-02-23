use std::path::Path;

use crate::io::stl::StlWriter;
use crate::mesh::polygon_soup::data_structure::PolygonSoup;
use crate::{dynamic_vdb, helpers::aliases::Vec3f};
use crate::voxel::*;

use self::meshing::ActiveVoxelsMesher;

pub type SdfGrid = dynamic_vdb!(f32, par 5, 4, 3);

#[derive(Debug)]
pub struct Sdf {
    grid: Box<SdfGrid>,
}

impl Sdf {
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
        let mut grid = SdfGrid::empty(Vec3i::zeros());

        let narrow_band_width = (narrow_band_width + 1) as f32 * voxel_size;
        let min = (min / voxel_size).map(|x| x.floor() as isize);
        let max = (max / voxel_size).map(|x| x.ceil() as isize);

        for x in min.x..max.x {
            for y in min.y..max.y {
                for z in min.z..max.z {
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

        Self { grid }
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

    pub(crate) fn grid(&self) -> &SdfGrid { // HIDE
        &self.grid
    }

    pub fn save_active(&self) {
        let mut active = ActiveVoxelsMesher::new();
        let active_voxels = PolygonSoup::from_vertices(active.mesh(self.grid.as_ref()).into_iter().map(|p| p.cast::<f32>()).collect());
        let writer = StlWriter::new();
        writer.write_stl_to_file(&active_voxels, Path::new("active.stl")).expect("Failed to write active.stl");
    }
}

impl From<Box<SdfGrid>> for Sdf {
    #[inline]
    fn from(grid: Box<SdfGrid>) -> Self {
        Self { grid }
    }
}
