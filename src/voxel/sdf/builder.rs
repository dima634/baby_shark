use crate::{geometry::primitives::box3::Box3, helpers::aliases::Vec3f};

use super::Sdf;

pub struct SdfBuilder {
    voxel_size: f32,
}

impl SdfBuilder {
    #[inline]
    pub fn with_voxel_size(voxel_size: f32) -> Self {
        SdfBuilder { voxel_size }
    }

    pub fn sphere(&self, radius: f32, origin: Vec3f) -> Sdf {
        let band_width = 1;
        let offset = radius + band_width as f32 * self.voxel_size;
        let min = origin.add_scalar(-offset);
        let max = origin.add_scalar(offset);

        Sdf::from_fn(self.voxel_size, min, max, band_width, |p| {
            (p - origin).norm() - radius
        })
    }

    pub fn cuboid(&self, min: Vec3f, max: Vec3f) -> Sdf {
        let band_width = 1;
        let offset = band_width as f32 * self.voxel_size;
        let grid_min = min.add_scalar(-offset);
        let grid_max = max.add_scalar(offset);
        let box3 = Box3::new(min, max);

        Sdf::from_fn(self.voxel_size, grid_min, grid_max, band_width, |p| {
            if box3.contains_point(&p) {
                -(p.x - min.x)
                    .min(max.x - p.x)
                    .min(p.y - min.y)
                    .min(max.y - p.y)
                    .min(p.z - min.z)
                    .min(max.z - p.z)
            } else {
                box3.squared_distance(&p).sqrt()
            }
        })
    }
}
