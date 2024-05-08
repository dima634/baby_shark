use super::Volume;
use crate::{geometry::primitives::box3::Box3, helpers::aliases::Vec3f};

/// Helper for building primitives
pub struct VolumeBuilder {
    voxel_size: f32,
}

impl VolumeBuilder {
    #[inline]
    pub fn with_voxel_size(mut self, voxel_size: f32) -> Self {
        self.set_voxel_size(voxel_size);
        self
    }

    #[inline]
    pub fn set_voxel_size(&mut self, voxel_size: f32) {
        self.voxel_size = voxel_size;
    }

    pub fn sphere(&self, radius: f32, origin: Vec3f) -> Volume {
        let band_width = 1;
        let offset = radius + band_width as f32 * self.voxel_size;
        let min = origin.add_scalar(-offset);
        let max = origin.add_scalar(offset);

        Volume::from_fn(self.voxel_size, min, max, band_width, |p| {
            (p - origin).norm() - radius
        })
    }

    pub fn cuboid(&self, min: Vec3f, max: Vec3f) -> Volume {
        let band_width = 1;
        let offset = band_width as f32 * self.voxel_size;
        let grid_min = min.add_scalar(-offset);
        let grid_max = max.add_scalar(offset);
        let box3 = Box3::new(min, max);

        Volume::from_fn(self.voxel_size, grid_min, grid_max, band_width, |p| {
            if box3.contains_point(p) {
                -(p.x - min.x)
                    .min(max.x - p.x)
                    .min(p.y - min.y)
                    .min(max.y - p.y)
                    .min(p.z - min.z)
                    .min(max.z - p.z)
            } else {
                box3.squared_distance(p).sqrt()
            }
        })
    }

    /// IWP TPMS
    pub fn iwp(&self, min: Vec3f, max: Vec3f, cell_size: f32) -> Volume {
        let cell_size_inv = 1.0 / cell_size;
        let sampling_bbox = Box3::new(min, max);
        let iwp_bbox = Box3::new(
            min.add_scalar(-self.voxel_size),
            max.add_scalar(self.voxel_size),
        );

        Volume::from_fn(self.voxel_size, *iwp_bbox.get_min(), *iwp_bbox.get_max(), 2, |p| {
            let x = p.x * cell_size_inv;
            let y = p.y * cell_size_inv;
            let z = p.z * cell_size_inv;
            let v = -(x.cos() + y.cos() + z.cos()
                - 0.51 * (x.cos() * y.cos() + y.cos() * z.cos() + z.cos() * x.cos())
                - 1.0);

            if !sampling_bbox.contains_point(p) {
                iwp_bbox.squared_distance(p).sqrt()
            } else {
                v * cell_size
            }
        })
    }
}

impl Default for VolumeBuilder {
    #[inline]
    fn default() -> Self {
        Self { voxel_size: 1.0 }
    }
}
