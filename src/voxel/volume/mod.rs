pub mod builder;

use self::fast_sweep::FastSweeping;
use self::visitors::ValueMutVisitor;
use crate::voxel::*;
use crate::{dynamic_vdb, helpers::aliases::Vec3f};

pub(super) type VolumeGrid = dynamic_vdb!(f32, par 5, 4, 3);

#[derive(Debug)]
pub struct Volume {
    grid: Box<VolumeGrid>,
    voxel_size: f32,
}

impl Volume {
    /// Creates empty volume with given voxel size.
    #[inline]
    pub fn with_voxel_size(voxel_size: f32) -> Self {
        Self {
            voxel_size,
            grid: VolumeGrid::empty(Vec3i::zeros()),
        }
    }

    #[inline]
    pub(super) fn new(grid: Box<VolumeGrid>, voxel_size: f32) -> Self {
        Self { grid, voxel_size }
    }

    #[inline]
    pub fn voxel_size(&self) -> f32 {
        self.voxel_size
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

        Self { grid, voxel_size }
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
        self.grid.remove_if(|val| val.abs() > self.voxel_size * 2.0);

        let mut extension_distance = distance.abs() + self.voxel_size + self.voxel_size;
        extension_distance.set_sign(distance.sign());

        let mut sweep = FastSweeping::new(self.voxel_size, extension_distance);
        sweep.fast_sweep(self.grid.as_mut());

        let mut offset = ValueMutVisitor::<VolumeGrid, _>::from_fn(|v| *v -= distance);
        self.grid.visit_values_mut(&mut offset);

        self
    }

    pub(in crate::voxel) fn grid(&self) -> &VolumeGrid {
        // HIDE
        &self.grid
    }
}

impl Clone for Volume {
    fn clone(&self) -> Self {
        Self {
            grid: self.grid.clone(),
            voxel_size: self.voxel_size,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use crate::{
        io::stl::StlReader,
        mesh::polygon_soup::data_structure::PolygonSoup,
        voxel::{meshing::MarchingCubesMesher, prelude::MeshToVolume},
    };

    #[test]
    fn test_volume_offset() {
        let mut reader = StlReader::new();
        let mesh: PolygonSoup<f32> = reader
            .read_stl_from_file(Path::new("./assets/box2.stl"))
            .expect("Read mesh");

        let volume = MeshToVolume::default()
            .with_voxel_size(0.2)
            .convert(&mesh)
            .unwrap()
            .offset(0.5);

        let vertices: Vec<_> = MarchingCubesMesher::default()
            .with_voxel_size(volume.voxel_size())
            .mesh(&volume);

        assert_eq!(vertices.len(), 7944);
    }
}
