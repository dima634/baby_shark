use nalgebra::Vector3;

use crate::dynamic_vdb;

use super::{Accessor, Scalar, TreeNode, SMALL_SCALAR};

pub(super) type SdfGrid = dynamic_vdb!(Scalar, par 5, 4, 3);

pub struct Sdf {
    grid: Box<SdfGrid>,
}

impl Sdf {
    ///
    /// Creates new SDF grid by evaluating given function on each grid point.
    /// Inside is negative.
    ///
    pub fn from_fn<TFn: Fn(&Vector3<f32>) -> f32>(
        grid_size: usize,
        min: f32,
        max: f32,
        narrow_band_width: usize,
        func: TFn,
    ) -> Self {
        let mut grid = SdfGrid::empty(Vector3::zeros());

        let origin = Vector3::new(min, min, min);
        let spacing = (max - min) / grid_size as f32;
        let narrow_band_width = (narrow_band_width + 1) as f32 * spacing;

        for x in 0..grid_size {
            for y in 0..grid_size {
                for z in 0..grid_size {
                    let idx: Vector3<isize> = Vector3::new(x, y, z).cast();

                    let grid_point = origin + idx.cast() * spacing;
                    let value = func(&grid_point);

                    if value.abs() > narrow_band_width {
                        continue;
                    }

                    let val = Scalar { value };
                    grid.insert(&idx, val);
                }
            }
        }

        grid.prune(SMALL_SCALAR);

        Self { grid }
    }

    pub fn grid(&self) -> &SdfGrid {
        &self.grid
    }
}

impl From<Box<SdfGrid>> for Sdf {
    #[inline]
    fn from(grid: Box<SdfGrid>) -> Self {
        Self { grid }
    }
}
