use crate::{
    geometry::{
        primitives::{box3::Box3, sphere3::Sphere3, triangle3::Triangle3},
        traits::*,
    },
    helpers::aliases::Vec3,
    mesh::traits::Triangles,
};
use nalgebra::Vector3;
use num_traits::AsPrimitive;
use std::collections::HashMap;

type Cell = Vector3<isize>;
type CellRange = Box3<isize>;

pub struct Grid<TObject: HasBBox3> {
    pub cells: HashMap<Cell, Vec<usize>>,
    objects: Vec<TObject>,
    cell_size: Vector3<TObject::Scalar>,
}

///
/// Infinite grid build over hash table
///
impl<TObject: HasBBox3> Grid<TObject> {
    pub fn new(objects: Vec<TObject>) -> Self {
        let cell_size = Self::calculate_cell_size(&objects);
        Self::with_cell_size(objects, cell_size)
    }

    pub fn empty() -> Self {
        Self {
            cell_size: Vector3::zeros(),
            cells: HashMap::new(),
            objects: Vec::new(),
        }
    }

    pub fn with_cell_size(objects: Vec<TObject>, cell_size: Vector3<TObject::Scalar>) -> Self {
        assert!(
            cell_size.iter().all(|d| *d > TObject::Scalar::zero()),
            "Cell size must be greater than zero, got {:?}",
            cell_size
        );

        let mut grid = Self {
            cells: HashMap::new(),
            cell_size,
            objects,
        };

        // Insert objects
        for i in 0..grid.objects.len() {
            grid.insert_object_at_index(i);
        }

        grid
    }

    /// Insert object to grid
    pub fn insert(&mut self, object: TObject) {
        let object_index = self.objects.len();
        self.objects.push(object);
        self.insert_object_at_index(object_index);
    }

    /// Inserts object that already exist in internal objects vector
    fn insert_object_at_index(&mut self, object_index: usize) {
        let bbox = self.objects[object_index].bbox();
        let cells = self.box_to_cell_range(&bbox);

        // Insert object in all cells that are intersected by it`s bbox
        for i in cells.get_min().x..=cells.get_max().x {
            for j in cells.get_min().y..=cells.get_max().y {
                for k in cells.get_min().z..=cells.get_max().z {
                    let cell = Cell::new(i, j, k);

                    if let Some(objects) = self.cells.get_mut(&cell) {
                        objects.push(object_index);
                    } else {
                        self.cells.insert(cell, vec![object_index]);
                    }
                }
            }
        }
    }

    #[inline]
    fn point_to_cell(&self, point: &Vec3<TObject::Scalar>) -> Cell {
        let cell_div = point.component_div(&self.cell_size);
        let mut cell = Vec3::new(cell_div.x.as_(), cell_div.y.as_(), cell_div.z.as_());

        if point.x < TObject::Scalar::zero() {
            cell.x -= 1;
        }

        if point.y < TObject::Scalar::zero() {
            cell.y -= 1;
        }

        if point.z < TObject::Scalar::zero() {
            cell.z -= 1;
        }

        cell
    }

    #[inline]
    fn box_to_cell_range(&self, bbox: &Box3<TObject::Scalar>) -> CellRange {
        return CellRange::new(
            self.point_to_cell(bbox.get_min()),
            self.point_to_cell(bbox.get_max()),
        );
    }

    /// Compute a reasonable size for the cell such that
    /// the number of cells is the same of the numbers of elements to be inserted in the grid
    fn calculate_cell_size(objects: &Vec<TObject>) -> Vector3<TObject::Scalar> {
        // Compute bbox of all objects
        let mut bbox = objects[0].bbox();
        for object in objects {
            bbox.union_box(&object.bbox());
        }

        let mut x = bbox.size_x().as_();
        let mut y = bbox.size_y().as_();
        let mut z = bbox.size_z().as_();

        let eps = 1e-6;
        let eps_scalar = TObject::Scalar::f64(eps);
        if x < eps && y < eps && z < eps {
            return Vector3::new(eps_scalar, eps_scalar, eps_scalar);
        }

        let min_size = min_value_bigger_than_eps(&[x, y, z], eps);
        x = x.clamp(min_size, x.max(min_size));
        y = y.clamp(min_size, y.max(min_size));
        z = z.clamp(min_size, z.max(min_size));

        let number_of_cells = objects.len() as f64;
        let cell_volume = (x * y * z) / number_of_cells;
        let cell_size = cell_volume.cbrt();

        Vector3::new(
            TObject::Scalar::f64(cell_size),
            TObject::Scalar::f64(cell_size),
            TObject::Scalar::f64(cell_size),
        )
    }
}

fn min_value_bigger_than_eps(values: &[f64], eps: f64) -> f64 {
    let mut min = f64::INFINITY;
    for value in values {
        if *value > eps && *value < min {
            min = *value;
        }
    }

    assert!(min.is_finite());
    min
}

impl<TObject> Grid<TObject>
where
    TObject: HasBBox3 + ClosestPoint3,
    TObject::Scalar: RealNumber,
{
    pub fn closest_point(
        &self,
        point: &Vec3<TObject::Scalar>,
        max_distance: TObject::Scalar,
    ) -> Option<Vec3<TObject::Scalar>> {
        let search_sphere = Sphere3::new(*point, max_distance);
        let sphere_bbox = search_sphere.bbox();

        // intersected cells
        let cells = self.box_to_cell_range(&sphere_bbox);
        let mut distance_squared = TObject::Scalar::inf();
        let mut closest_point = Vec3::zeros();

        // Search for closest point
        for i in cells.get_min().x..=cells.get_max().x {
            for j in cells.get_min().y..=cells.get_max().y {
                for k in cells.get_min().z..=cells.get_max().z {
                    let cell = Cell::new(i, j, k);
                    let cell_bbox = self.cell_to_box(&cell);

                    // Reject cells outside sphere
                    if !cell_bbox.intersects_sphere3(&search_sphere) {
                        continue;
                    }

                    // Reject cells that are farther than already found closest point
                    if !cell_bbox.contains_point(point)
                        && cell_bbox.squared_distance(point) > distance_squared
                    {
                        continue;
                    }

                    if let Some(objects_in_cell) = self.cells.get(&cell) {
                        // Find closest object in cell
                        for obj_index in objects_in_cell {
                            let object = &self.objects[*obj_index];
                            let new_closest = object.closest_point(point);
                            let new_distance_squared = (new_closest - point).norm_squared();

                            if new_distance_squared < distance_squared {
                                distance_squared = new_distance_squared;
                                closest_point = new_closest;
                            }
                        }
                    }
                }
            }
        }

        if !distance_squared.is_finite() {
            return None;
        }

        Some(closest_point)
    }

    pub fn cell_to_box(&self, cell: &Cell) -> Box3<TObject::Scalar> {
        let min = self.cell_size.component_mul(&Vec3::new(
            TObject::Scalar::isize(cell.x),
            TObject::Scalar::isize(cell.y),
            TObject::Scalar::isize(cell.z),
        ));
        let max = self.cell_size.component_mul(&Vec3::new(
            TObject::Scalar::isize(cell.x + 1),
            TObject::Scalar::isize(cell.y + 1),
            TObject::Scalar::isize(cell.z + 1),
        ));
        Box3::new(min, max)
    }
}

impl<S: RealNumber> Grid<Triangle3<S>> {
    /// Create grid from faces of triangular mesh
    pub fn from_mesh<TMesh: Triangles<Scalar = S>>(mesh: &TMesh) -> Self {
        let faces: Vec<Triangle3<S>> = mesh.triangles().collect();

        Self::new(faces)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{helpers::aliases::Vec3f, mesh::corner_table::CornerTableF};

    #[test]
    fn test_grid_creation_zero_volume_cases() {
        let expected_cell_size = Vec3f::new(0.793700516, 0.793700516, 0.793700516);

        let x_0 = CornerTableF::from_vertex_and_face_slices(
            &vec![
                Vec3f::new(0.0, 0.0, 0.0),
                Vec3f::new(0.0, 1.0, 0.0),
                Vec3f::new(0.0, 1.0, 1.0),
                Vec3f::new(0.0, 0.0, 1.0),
            ],
            &vec![0, 1, 2, 0, 2, 3],
        );

        let grid = Grid::from_mesh(&x_0);
        assert!((expected_cell_size - grid.cell_size).norm() < 1e-6);

        let y_0 = CornerTableF::from_vertex_and_face_slices(
            &vec![
                Vec3f::new(0.0, 0.0, 0.0),
                Vec3f::new(1.0, 0.0, 0.0),
                Vec3f::new(1.0, 0.0, 1.0),
                Vec3f::new(0.0, 0.0, 1.0),
            ],
            &vec![0, 1, 2, 0, 2, 3],
        );

        let grid = Grid::from_mesh(&y_0);
        assert!((expected_cell_size - grid.cell_size).norm() < 1e-6);

        let z_0 = CornerTableF::from_vertex_and_face_slices(
            &vec![
                Vec3f::new(0.0, 0.0, 0.0),
                Vec3f::new(1.0, 0.0, 0.0),
                Vec3f::new(1.0, 1.0, 0.0),
                Vec3f::new(0.0, 1.0, 0.0),
            ],
            &vec![0, 1, 2, 0, 2, 3],
        );

        let grid = Grid::from_mesh(&z_0);
        assert!((expected_cell_size - grid.cell_size).norm() < 1e-6);
    }
}
