use std::collections::HashMap;

use nalgebra::Vector3;
use num_traits::{cast, Float, Zero};

use crate::{
    geometry::{
        traits::{
            HasBBox3, 
            ClosestPoint3, 
            RealNumber
        }, 
        primitives::{
            box3::Box3, 
            sphere3::Sphere3, 
            triangle3::Triangle3
        }
    }, 
    mesh::traits::Mesh, algo::utils, helpers::aliases::Vec3
};

type Cell = Vector3<isize>;
type CellRange = Box3<isize>;

pub struct Grid<TObject: HasBBox3> {
    pub cells: HashMap<Cell, Vec<usize>>,
    objects: Vec<TObject>,
    cell_size: Vector3<TObject::ScalarType>
}

///
/// Infinite grid build over hash table
/// 
impl<TObject: HasBBox3> Grid<TObject> {
    pub fn new(objects: Vec<TObject>) -> Self {
        let cell_size = Self::calculate_cell_size(&objects);
        return Self::with_cell_size(objects, cell_size);
    }

    pub fn empty() -> Self {
        return Self {
            cell_size: Vector3::zeros(),
            cells: HashMap::new(),
            objects: Vec::new()
        };
    }

    pub fn with_cell_size(objects: Vec<TObject>, cell_size: Vector3<TObject::ScalarType>) -> Self {
        let mut grid = Self {
            cells: HashMap::new(),
            cell_size,
            objects
        };

        // Insert objects
        for i in 0..grid.objects.len() {
            grid.insert_object_at_index(i);
        }
        
        return grid;
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
    fn point_to_cell(&self, point: &Vec3<TObject::ScalarType>) -> Cell {
        let mut cell = utils::cast(&point.component_div(&self.cell_size));

        if point.x < Zero::zero() {
            cell.x -= 1;
        }

        if point.y < Zero::zero() {
            cell.y -= 1;
        }

        if point.z < Zero::zero() {
            cell.z -= 1;
        }

        return cell;
    }

    #[inline]
    fn box_to_cell_range(&self, bbox: &Box3<TObject::ScalarType>) -> CellRange {
        return CellRange::new(
            self.point_to_cell(bbox.get_min()).into(),
            self.point_to_cell(bbox.get_max()).into() 
        );
    }

    /// Compute a reasonable size for the cell such that 
    /// the number of cells is the same of the numbers of elements to be inserted in the grid
    fn calculate_cell_size(objects: &Vec<TObject>) -> Vector3<TObject::ScalarType> {
        // Compute bbox of all objects
        let mut bbox = objects[0].bbox();
        for object in objects {
            bbox.add_box3(&object.bbox());
        }

        let number_of_cells = objects.len() as f64;
        let cell_volume = cast::<TObject::ScalarType, f64>(bbox.volume()).unwrap() / number_of_cells;
        let cell_size = cast(cell_volume.cbrt()).unwrap();

        return Vector3::new(cell_size, cell_size, cell_size);        
    }
}

impl<TObject> Grid<TObject> 
where
    TObject: HasBBox3 + ClosestPoint3,
    TObject::ScalarType: RealNumber,
{
    pub fn closest_point(&self, point: &Vec3<TObject::ScalarType>, max_distance: TObject::ScalarType) -> Option<Vec3<TObject::ScalarType>> {
        let search_sphere = Sphere3::new(*point, max_distance);
        let sphere_bbox = search_sphere.bbox();

        // intersected cells
        let cells = self.box_to_cell_range(&sphere_bbox);
        let mut distance_squared = Float::infinity();
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
                    if !cell_bbox.contains_point(point) && cell_bbox.squared_distance(point) > distance_squared {
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

        if distance_squared.is_infinite() {
            return None;
        }

        return Some(closest_point);
    }

    #[inline]
    pub fn cell_to_box(&self, cell: &Cell) -> Box3<TObject::ScalarType> {
        return Box3::new(
            self.cell_size.component_mul(&utils::cast(cell)).into(),
            self.cell_size.component_mul(&utils::cast(&cell.add_scalar(1))).into(),
        );
    }
}

impl<TScalar: RealNumber> Grid<Triangle3<TScalar>>{
    /// Create grid from faces of triangular mesh
    pub fn from_mesh<TMesh: Mesh<ScalarType = TScalar>>(mesh: &TMesh) -> Self {
        let faces: Vec<Triangle3<TScalar>> = mesh.faces()
            .map(|face| mesh.face_positions(&face))
            .collect();

        return Self::new(faces);
    }
}
