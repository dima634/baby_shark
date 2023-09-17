use std::collections::HashMap;

use nalgebra::{Vector3, Point3};

use crate::{mesh::traits::Mesh, geometry::primitives::box3::Box3, algo::utils::cast, voxel::TreeNode};


pub fn uniform<TMesh: Mesh, TGrid: TreeNode>(grid: &TGrid) -> TMesh {
    let mut vertices: Vec<Vector3<isize>> = Vec::new();
    let mut indices: Vec<usize> = Vec::new();
    let mut index_vertex_map = HashMap::<Vector3<isize>, usize>::new();

    let bbox = Box3::new(Point3::new(0, 0, 0), Point3::new(1, 1, 1));
    let v0 = bbox.vertex(0).coords;
    let v1 = bbox.vertex(1).coords;
    let v2 = bbox.vertex(2).coords;
    let v3 = bbox.vertex(3).coords;
    let v4 = bbox.vertex(4).coords;
    let v5 = bbox.vertex(5).coords;
    let v6 = bbox.vertex(6).coords;
    let v7 = bbox.vertex(7).coords;

    grid.voxels(&mut |voxel| {
        let top_index     = voxel + Vector3::new(0, 0, 1);
        let bottom_index  = voxel + Vector3::new(0, 0, -1);
        let left_index    = voxel + Vector3::new(-1, 0, 0);
        let right_index   = voxel + Vector3::new(1, 0, 0);
        let front_index   = voxel + Vector3::new(0, 1, 0);
        let back_index    = voxel + Vector3::new(0, -1, 0);

        let top     = grid.at(&top_index);
        let bottom  = grid.at(&bottom_index);
        let left    = grid.at(&left_index);
        let right   = grid.at(&right_index);
        let front   = grid.at(&front_index);
        let back    = grid.at(&back_index);

        if !top {
            let faces = [
                voxel + v4,
                voxel + v7,
                voxel + v6,

                voxel + v4,
                voxel + v5,
                voxel + v7,
            ];

            add_faces(&mut index_vertex_map, &mut vertices, &faces, &mut indices);
        }

        if !bottom {
            let faces = [
                voxel + v0,
                voxel + v2,
                voxel + v3,

                voxel + v0,
                voxel + v3,
                voxel + v1,
            ];

            add_faces(&mut index_vertex_map, &mut vertices, &faces, &mut indices);
        }

        if !left {
            let faces = [
                voxel + v0,
                voxel + v4,
                voxel + v6,

                voxel + v0,
                voxel + v6,
                voxel + v2,
            ];

            add_faces(&mut index_vertex_map, &mut vertices, &faces, &mut indices);
        }

        if !right {
            let faces = [
                voxel + v1,
                voxel + v7,
                voxel + v5,

                voxel + v1,
                voxel + v3,
                voxel + v7,
            ];

            add_faces(&mut index_vertex_map, &mut vertices, &faces, &mut indices);
        }

        if !front {
            let faces = [
                voxel + v2,
                voxel + v6,
                voxel + v7,

                voxel + v2,
                voxel + v7,
                voxel + v3,
            ];

            add_faces(&mut index_vertex_map, &mut vertices, &faces, &mut indices);
        }

        if !back {
            let faces = [
                voxel + v0,
                voxel + v5,
                voxel + v4,

                voxel + v0,
                voxel + v1,
                voxel + v5,
            ];

            add_faces(&mut index_vertex_map, &mut vertices, &faces, &mut indices);
        }
    });

    let vertices: Vec<_> = vertices.into_iter().map(|v| cast(&v).into()).collect();
    TMesh::from_vertices_and_indices(vertices.as_slice(), &indices)
}

fn add_faces(
    map: &mut HashMap::<Vector3<isize>, usize>,
    vertices: &mut Vec<Vector3<isize>>,
    faces: &[Vector3<isize>],
    face_indices: &mut Vec<usize>
) {
    for face in faces {
        let idx = get_or_insert_vertex(*face, map, vertices);
        face_indices.push(idx);
    }
}

fn get_or_insert_vertex(
    vertex: Vector3<isize>, 
    map: &mut HashMap::<Vector3<isize>, usize>,
    vertices: &mut Vec<Vector3<isize>>,
) -> usize {
    if let Some(idx) = map.get(&vertex) {
        *idx
    } else {
        let idx = vertices.len();
        vertices.push(vertex);
        map.insert(vertex, idx);
        idx
    }
} 
