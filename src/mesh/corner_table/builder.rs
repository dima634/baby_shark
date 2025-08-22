use super::{traversal::*, *};
use crate::{
    algo::merge_points::merge_points,
    geometry::traits::RealNumber,
    helpers::aliases::Vec3,
    io::{self, Builder, IndexedBuilder as _, SoupBuilder as _},
    mesh::traits::stats::IDEAL_INTERIOR_VERTEX_VALENCE,
};
use std::collections::{BTreeSet, HashMap};

impl<TScalar: RealNumber> Default for CornerTable<TScalar> {
    #[inline]
    fn default() -> Self {
        Self {
            vertices: Vec::new(),
            corners: Vec::new(),
        }
    }
}

impl<TScalar: RealNumber> CornerTable<TScalar> {
    #[inline]
    pub fn new() -> Self {
        Default::default()
    }

    #[inline]
    pub fn with_capacity(num_vertices: usize, num_faces: usize) -> Self {
        Self {
            vertices: Vec::with_capacity(num_vertices),
            corners: Vec::with_capacity(num_faces * 3),
        }
    }

    #[inline]
    pub(super) fn create_vertex(
        &mut self,
        corner: Option<CornerId>,
        position: Vec3<TScalar>,
    ) -> VertexId {
        let idx = self.vertices.len();
        let vertex = Vertex::new(CornerId::from_option(corner), position);
        self.vertices.push(vertex);
        VertexId::new(u32::try_from(idx).expect("number of vertices is too big"))
    }

    #[inline]
    pub(super) fn create_corner(&mut self, vertex: VertexId) -> (CornerId, &mut Corner) {
        let idx = u32::try_from(self.corners.len())
            .expect("number of corners is too big");
        self.corners.push(Corner::new(None, vertex));
        (CornerId::new(idx), &mut self.corners[idx as usize])
    }

    fn corner_from(
        &mut self,
        edge_opposite_corner_map: &mut HashMap<Edge, CornerId>,
        edge: Edge,
        vertex_id: VertexId,
    ) -> CornerId {
        let (corner_id, corner) = self.create_corner(vertex_id);

        // Find opposite corner
        let opposite_corner_index = edge_opposite_corner_map.get(&edge.flipped());

        if let Some(&opposite_corner_id) = opposite_corner_index {
            // Set opposite for corners
            corner.set_opposite_corner(Some(opposite_corner_id));
            let opposite_corner = &mut self[opposite_corner_id];

            opposite_corner.set_opposite_corner(Some(corner_id));

            edge_opposite_corner_map.insert(edge, corner_id);
        } else {
            // Save directed edge and it`s opposite corner
            edge_opposite_corner_map.insert(edge, corner_id);
        }

        // Set corner index for vertex
        let vertex = &mut self[vertex_id];
        vertex.set_corner(corner_id);

        corner_id
    }
}

impl<R: RealNumber> CornerTable<R> {
    #[inline]
    pub fn from_vertex_and_face_slices(vertices: &[Vec3<R>], faces: &[usize]) -> Self {
        Self::from_vertex_and_face_iters(vertices.iter().cloned(), faces.iter().cloned())
    }

    pub fn from_vertex_and_face_iters(
        vertices: impl Iterator<Item = Vec3<R>>,
        mut faces: impl Iterator<Item = usize>,
    ) -> Self {
        let mut builder = Self::builder_indexed();
        let Ok(_) = builder.add_vertices(vertices) else { return Self::default(); };

        loop {
            let Some(v1) = faces.next() else { break };
            let Some(v2) = faces.next() else { break };
            let Some(v3) = faces.next() else { break };
            let Ok(_) = builder.add_face(v1, v2, v3) else { return Self::default(); };
        }

        builder.finish().unwrap_or_default()
    }

    pub fn from_triangles_soup(triangles: impl Iterator<Item = Vec3<R>>) -> Self {
        let mut builder = Self::builder_soup();
        let _ = builder.add_faces(triangles);
        builder.finish().unwrap_or_default()
    }
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, Copy)]
pub struct Edge {
    start_vertex: VertexId,
    end_vertex: VertexId,
}

impl Edge {
    #[inline]
    pub fn new(start: VertexId, end: VertexId) -> Self {
        Self {
            start_vertex: start,
            end_vertex: end,
        }
    }

    #[inline]
    pub fn flipped(&self) -> Self {
        Self {
            start_vertex: self.end_vertex,
            end_vertex: self.start_vertex,
        }
    }
}

#[derive(Debug)]
struct IndexedBuilder<R: RealNumber> {
    corner_table: CornerTable<R>,
    edge_opposite_corner_map: HashMap<Edge, CornerId>,
    vertex_corners: HashMap<VertexId, BTreeSet<CornerId>>,
}

impl<R: RealNumber> Default for IndexedBuilder<R> {
    #[inline]
    fn default() -> Self {
        Self {
            corner_table: CornerTable::new(),
            edge_opposite_corner_map: HashMap::new(),
            vertex_corners: HashMap::new(),
        }
    }
}

impl<R: RealNumber> io::IndexedBuilder<R, CornerTable<R>> for IndexedBuilder<R> {
    #[inline]
    fn add_vertex<T: Into<[R; 3]>>(&mut self, vertex: T) -> Result<usize, io::BuildError> {
        let idx = self
            .corner_table
            .create_vertex(None, vertex.into().into())
            .index();
        Ok(idx)
    }

    fn add_face(&mut self, v1: usize, v2: usize, v3: usize) -> Result<(), io::BuildError> {
        if v1 >= self.corner_table.vertices.len()
            || v2 >= self.corner_table.vertices.len()
            || v3 >= self.corner_table.vertices.len()
        {
            return Err(io::BuildError::InvalidVertex);
        }

        if v1 > u32::MAX as usize || v2 > u32::MAX as usize || v3 > u32::MAX as usize {
            return Err(io::BuildError::TooBig);
        }

        let v1_id = VertexId::new(v1 as u32);
        let v2_id = VertexId::new(v2 as u32);
        let v3_id = VertexId::new(v3 as u32);

        let edge1 = Edge::new(v2_id, v3_id);
        let edge2 = Edge::new(v3_id, v1_id);
        let edge3 = Edge::new(v1_id, v2_id);

        // If edge already exist in map then it is non manifold. For now we will skip faces that introduce non-manifoldness.
        if self.edge_opposite_corner_map.contains_key(&edge1)
            || self.edge_opposite_corner_map.contains_key(&edge2)
            || self.edge_opposite_corner_map.contains_key(&edge3)
        {
            return Ok(());
        }

        let c1 = self.corner_table
            .corner_from(&mut self.edge_opposite_corner_map, edge1, v1_id);
        let c2 = self.corner_table
            .corner_from(&mut self.edge_opposite_corner_map, edge2, v2_id);
        let c3 = self.corner_table
            .corner_from(&mut self.edge_opposite_corner_map, edge3, v3_id);

        self.vertex_corners.entry(v1_id).or_default().insert(c1);
        self.vertex_corners.entry(v2_id).or_default().insert(c2);
        self.vertex_corners.entry(v3_id).or_default().insert(c3);

        Ok(())
    }

    #[inline]
    fn set_num_vertices(&mut self, count: usize) {
        self.corner_table.vertices.reserve(count);
        self.vertex_corners.reserve(count);
    }

    #[inline]
    fn set_num_faces(&mut self, count: usize) {
        self.corner_table.corners.reserve(count);
        self.edge_opposite_corner_map.reserve(count);
    }

    fn finish(mut self) -> Result<CornerTable<R>, io::BuildError> {
        // Delete isolated vertices
        for vertex in &mut self.corner_table.vertices {
            if !vertex.corner().is_valid() {
                vertex.set_deleted(true);
            }
        }

        // Duplicate non-manifold vertices
        for (&vertex_id, corners) in self.vertex_corners.iter_mut() {
            if self.corner_table[vertex_id].is_deleted() {
                continue;
            }

            self.corner_table
                .corners_around_vertex(vertex_id, |corner_index| {
                    corners.remove(&corner_index);
                });

            let vertex_pos = *self.corner_table[vertex_id].position();

            // Duplicate vertex for each "corner fan"
            while let Some(corner_idx) = corners.pop_first() {
                let duplicate_vertex = self.corner_table
                    .create_vertex(Some(corner_idx), vertex_pos);

                for adj_corner in collect_corners_around_vertex(&self.corner_table, duplicate_vertex) {
                    self.corner_table[adj_corner].set_vertex(duplicate_vertex);
                    corners.remove(&adj_corner);
                }
            }
        }

        Ok(self.corner_table)
    }
}

#[derive(Debug)]
struct SoupBuilder<R: RealNumber> {
    triangles: Vec<Vec3<R>>,
}

impl<R: RealNumber> Default for SoupBuilder<R> {
    #[inline]
    fn default() -> Self {
        Self {
            triangles: Vec::new(),
        }
    }
}

impl<R: RealNumber> io::SoupBuilder<R, CornerTable<R>> for SoupBuilder<R> {
    fn add_face<T: Into<[R; 3]>>(&mut self, v1: T, v2: T, v3: T) -> Result<(), io::BuildError> {
        self.triangles.push(v1.into().into());
        self.triangles.push(v2.into().into());
        self.triangles.push(v3.into().into());
        Ok(())
    }

    #[inline]
    fn set_num_faces(&mut self, count: usize) {
        self.triangles.reserve(count * 3);
    }

    fn finish(self) -> Result<CornerTable<R>, io::BuildError> {
        let num_vertices = self.triangles.len();
        let indexed = merge_points(self.triangles.into_iter());

        let mut indexed_builder = IndexedBuilder::default();
        indexed_builder.set_num_vertices(num_vertices / IDEAL_INTERIOR_VERTEX_VALENCE); // Estimate number of unique vertices
        indexed_builder.set_num_faces(num_vertices / 3);

        for vertex in indexed.points {
            indexed_builder.add_vertex(vertex)?;
        }

        for face in indexed.indices.chunks_exact(3) {
            indexed_builder.add_face(face[0], face[1], face[2])?;
        }

        indexed_builder.finish()
    }
}

impl<R: RealNumber> io::Builder for CornerTable<R> {
    type Scalar = R;
    type Mesh = CornerTable<R>;

    #[inline]
    fn builder_indexed() -> impl io::IndexedBuilder<Self::Scalar, Self::Mesh> {
        IndexedBuilder::default()
    }

    #[inline]
    fn builder_soup() -> impl io::SoupBuilder<Self::Scalar, Self::Mesh> {
        SoupBuilder::default()
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        helpers::aliases::{Vec3, Vec3f},
        mesh::corner_table::{
            test_helpers::{assert_mesh_eq, create_unit_square_mesh},
            *,
        },
    };

    #[test]
    fn from_vertices_and_indices() {
        let mesh = create_unit_square_mesh();

        let expected_vertices = vec![
            Vertex::new(CornerId::new(5), Vec3f::new(0.0, 1.0, 0.0)),
            Vertex::new(CornerId::new(1), Vec3f::new(0.0, 0.0, 0.0)),
            Vertex::new(CornerId::new(3), Vec3f::new(1.0, 0.0, 0.0)),
            Vertex::new(CornerId::new(4), Vec3f::new(1.0, 1.0, 0.0)),
        ];

        let expected_corners = vec![
            Corner::new(None, VertexId::new(0)),
            Corner::new(Some(CornerId::new(4)), VertexId::new(1)),
            Corner::new(None,                   VertexId::new(2)),

            Corner::new(None,                   VertexId::new(2)),
            Corner::new(Some(CornerId::new(1)), VertexId::new(3)),
            Corner::new(None, VertexId::new(0)),
        ];

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn should_remove_face_that_introduces_non_manifold_edge() {
        let mesh = CornerTableF::from_vertex_and_face_slices(&[
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(0.0, 0.0, -1.0),
        ], &[
            0, 1, 2,
            0, 1, 4,
            0, 3, 1,
            3, 5, 1,
            1, 5, 2,
        ]);

        assert!(mesh.faces().count() == 4);
    }

    #[test]
    fn should_fix_non_manifold_topology() {
        let mesh = CornerTableF::from_vertex_and_face_slices(
            &vec![
                Vec3f::new(0.8834067, 0.62114143, 0.41104388),
                Vec3f::new(0.95254904, 0.56330395, 0.24630469),
                Vec3f::new(0.8628991, 0.6317301, -0.0009160638),
                Vec3f::new(0.34230882, 0.53532004, -0.85211533),
                Vec3f::new(0.3680833, 0.38523912, -0.84645087),
                Vec3f::new(0.30370504, 0.34734392, -0.7940565),
                Vec3f::new(0.24977475, 0.15333056, -0.64143056),
                Vec3f::new(0.045607388, 0.19006538, -0.63903505),
                Vec3f::new(0.45543462, 0.22949314, -0.69263667),
                Vec3f::new(0.45149833, 0.32491922, -0.82496625),
                Vec3f::new(0.16190892, 0.45134497, -0.71620816),
                Vec3f::new(0.26690322, 0.61624765, -0.6935876),
                Vec3f::new(0.06485206, 0.01302433, -0.52222604),
                Vec3f::new(-0.071166694, 0.25918913, -0.5588313),
                Vec3f::new(0.012033761, 0.43446326, -0.5318709),
                Vec3f::new(0.1047948, 0.52119493, -0.5164339),
                Vec3f::new(-0.3353879, -0.007911205, -0.39118212),
                Vec3f::new(-0.16893119, -0.11193609, -0.40532357),
                Vec3f::new(-0.26785487, 0.17082739, -0.39922267),
                Vec3f::new(0.36443406, 0.1300087, -0.44807464),
                Vec3f::new(0.4858721, 0.27225876, -0.44158846),
                Vec3f::new(0.26519567, 0.6003003, -0.46935147),
                Vec3f::new(0.10446578, 0.48864722, -0.36064798),
                Vec3f::new(0.6970952, 0.8061323, 0.030847728),
                Vec3f::new(0.745894, 0.76371956, 0.45065618),
                Vec3f::new(-0.057397544, -0.1629417, -0.21870333),
                Vec3f::new(0.09102613, -0.06893039, -0.3253036),
                Vec3f::new(-0.051894844, 0.39716768, -0.3253036),
                Vec3f::new(0.25763875, 0.46904802, -0.3212275),
                Vec3f::new(0.3807624, 0.4538355, -0.3418706),
                Vec3f::new(0.57614785, 0.8629222, 0.20697945),
                Vec3f::new(0.6542867, 0.8435347, 0.37989998),
                Vec3f::new(0.18667108, 0.0451324, -0.23886198),
                Vec3f::new(-0.2841255, 0.23004746, -0.19915086),
                Vec3f::new(-0.44643182, 0.108356, -0.19611543),
                Vec3f::new(0.059900105, 0.29487085, -0.18818039),
                Vec3f::new(0.05176288, 0.08898044, -0.09317893),
                Vec3f::new(0.0054710507, 0.26432276, -0.12414271),
                Vec3f::new(0.01870805, -0.026912212, -0.023123085),
                Vec3f::new(-0.14745063, -0.2108159, -0.018603861),
                Vec3f::new(-0.41003102, 0.11480737, 0.047084033),
                Vec3f::new(-0.17251796, 0.23065639, -0.018603861),
                Vec3f::new(-0.0014564395, 0.27398062, -0.020393431),
                Vec3f::new(0.14822704, 0.11363363, 0.0033800006),
                Vec3f::new(0.22450429, 0.30367017, -0.016710103),
                Vec3f::new(0.3086589, 0.18566132, 0.037852466),
                Vec3f::new(0.67204887, 0.4667387, 0.026692808),
                Vec3f::new(0.19839746, 0.43664527, 0.016994178),
                Vec3f::new(0.5925439, 0.57969975, -0.0123294),
                Vec3f::new(0.44081622, 0.6455002, 0.026015222),
                Vec3f::new(-0.045694053, -0.14503074, 0.13664407),
                Vec3f::new(0.22991878, 0.040702343, 0.17186683),
                Vec3f::new(0.4790824, 0.24588966, 0.10792464),
                Vec3f::new(-0.23558539, 0.26461792, 0.1789195),
                Vec3f::new(0.016513169, 0.42079568, 0.108267486),
                Vec3f::new(0.7962783, 0.47271585, 0.11347264),
                Vec3f::new(0.3188656, 0.6375952, 0.10491818),
                Vec3f::new(0.15554553, 0.5591631, 0.20997256),
                Vec3f::new(0.49671823, 0.7393873, 0.06656808),
                Vec3f::new(0.08202773, -0.08054447, 0.2267558),
                Vec3f::new(0.49583656, 0.23262191, 0.27301043),
                Vec3f::new(0.7926627, 0.43568754, 0.2267558),
                Vec3f::new(0.013941586, -0.21383882, 0.38984036),
                Vec3f::new(-0.1695506, -0.30226946, 0.30199146),
                Vec3f::new(-0.06693143, -0.36968112, 0.41554427),
                Vec3f::new(-0.0207991, 0.012731791, 0.43987632),
                Vec3f::new(0.11826736, -0.014563084, 0.33671856),
                Vec3f::new(0.29027635, 0.16070127, 0.3833468),
                Vec3f::new(0.16373092, 0.117931366, 0.4237094),
                Vec3f::new(0.47959548, 0.2834401, 0.37649322),
                Vec3f::new(-0.19974011, 0.25659537, 0.34162855),
                Vec3f::new(0.028450787, 0.3770585, 0.39198232),
                Vec3f::new(0.8177584, 0.4846413, 0.3246882),
                Vec3f::new(0.1612733, 0.5412743, 0.31483912),
                Vec3f::new(0.6712621, 0.48579454, 0.42190504),
                Vec3f::new(0.46876317, 0.7466214, 0.3372848),
                Vec3f::new(0.23257285, 0.50125647, 0.4107461),
                Vec3f::new(0.18332511, -0.25458288, 0.52854323),
                Vec3f::new(0.22413331, -0.06275678, 0.5360899),
                Vec3f::new(0.12074882, 0.017737627, 0.475976),
                Vec3f::new(-0.4246766, -0.0013813972, 0.4441998),
                Vec3f::new(-0.11923951, 0.19975066, 0.43242645),
                Vec3f::new(0.25517446, 0.3343401, 0.4543321),
                Vec3f::new(0.4869507, 0.6543765, 0.4263327),
                Vec3f::new(0.65388423, 0.6103699, 0.45511413),
                Vec3f::new(0.08468515, -0.38390625, 0.60776496),
                Vec3f::new(0.46977264, -0.22713304, 0.7447612),
                Vec3f::new(0.4426244, -0.13895798, 0.65781426),
                Vec3f::new(-0.34487838, -0.039580107, 0.5870192),
                Vec3f::new(-0.04760188, 0.054908752, 0.68121386),
                Vec3f::new(0.1981762, 0.059452772, 0.62453127),
                Vec3f::new(-0.039135635, 0.02654314, 0.823158),
                Vec3f::new(-0.18757552, -0.096291065, 0.8113942),
                Vec3f::new(0.1764105, 0.11895847, 0.73188853),
                Vec3f::new(0.25517446, -0.3134513, 0.89247036),
                Vec3f::new(-0.051524818, -0.43365586, 0.6599939),
                Vec3f::new(0.05864364, -0.26413333, 0.92179966),
                Vec3f::new(0.44143897, -0.27763057, 0.8350637),
                Vec3f::new(-0.007884204, -0.14083815, 0.9356606),
                Vec3f::new(2.711546, -1.7519422, -0.013143648),
            ],
            &vec![
                1, 2, 3, 4, 5, 6, 6, 7, 8, 6, 9, 7, 9, 6, 10, 11, 6, 8, 6, 5, 10, 11, 4, 6, 12, 4,
                11, 8, 7, 13, 11, 8, 14, 15, 11, 14, 15, 12, 11, 16, 12, 15, 17, 8, 18, 8, 13, 18,
                19, 8, 17, 19, 14, 8, 20, 7, 9, 20, 9, 21, 22, 12, 16, 16, 23, 22, 12, 9, 5, 21, 9,
                12, 12, 22, 21, 3, 24, 25, 1, 3, 25, 26, 18, 13, 27, 26, 13, 7, 27, 13, 7, 20, 27,
                15, 14, 19, 28, 16, 15, 23, 16, 28, 22, 23, 29, 22, 29, 30, 22, 30, 21, 4, 12, 5,
                31, 32, 25, 9, 10, 5, 24, 31, 25, 33, 27, 20, 34, 19, 35, 15, 19, 34, 15, 34, 28,
                33, 20, 21, 33, 21, 36, 36, 21, 30, 29, 36, 30, 23, 36, 29, 33, 26, 27, 37, 26, 33,
                36, 37, 33, 36, 28, 34, 38, 37, 36, 36, 23, 28, 39, 40, 26, 39, 26, 37, 34, 35, 41,
                34, 41, 42, 36, 34, 42, 37, 38, 43, 38, 36, 42, 38, 42, 43, 37, 44, 39, 45, 37, 43,
                45, 44, 37, 45, 46, 44, 45, 47, 46, 48, 45, 43, 49, 45, 48, 49, 47, 45, 50, 49, 48,
                49, 3, 47, 24, 49, 50, 24, 3, 49, 39, 51, 40, 44, 51, 39, 44, 52, 51, 53, 52, 44,
                53, 44, 46, 41, 54, 42, 55, 42, 54, 55, 43, 42, 47, 53, 46, 55, 48, 43, 47, 56, 53,
                57, 48, 55, 57, 50, 48, 58, 57, 55, 59, 50, 57, 3, 56, 47, 3, 2, 56, 59, 24, 50,
                31, 24, 59, 52, 60, 51, 53, 61, 52, 53, 62, 61, 58, 55, 54, 56, 62, 53, 2, 62, 56,
                31, 59, 57, 31, 57, 58, 51, 63, 64, 64, 63, 65, 63, 51, 60, 63, 60, 66, 67, 66, 60,
                68, 60, 52, 68, 67, 60, 69, 67, 68, 70, 68, 52, 61, 70, 52, 58, 54, 71, 58, 71, 72,
                61, 62, 70, 73, 70, 62, 74, 58, 72, 75, 70, 73, 2, 73, 62, 58, 74, 76, 74, 77, 76,
                1, 73, 2, 31, 58, 76, 31, 76, 32, 63, 78, 65, 79, 63, 66, 79, 78, 63, 80, 66, 67,
                80, 79, 66, 71, 81, 66, 71, 66, 82, 72, 71, 82, 83, 82, 66, 83, 66, 69, 83, 69, 68,
                72, 82, 83, 72, 83, 77, 83, 68, 70, 83, 70, 75, 77, 74, 72, 77, 83, 84, 85, 84, 83,
                85, 83, 75, 84, 76, 77, 75, 73, 1, 85, 75, 1, 84, 85, 25, 25, 85, 1, 32, 76, 84,
                32, 84, 25, 78, 86, 65, 78, 87, 86, 78, 88, 87, 79, 88, 78, 66, 81, 89, 90, 66, 89,
                91, 66, 90, 79, 80, 91, 92, 89, 93, 90, 89, 92, 90, 94, 91, 95, 96, 86, 95, 97, 96,
                87, 95, 86, 87, 98, 95, 99, 97, 94, 94, 90, 92, 94, 92, 99, 92, 93, 99, 88, 79, 95,
                79, 91, 94, 94, 97, 79, 97, 95, 79, 99, 93, 96, 81, 71, 51, 96, 89, 65, 51, 64, 81,
                89, 81, 64, 65, 86, 96, 64, 65, 89, 40, 51, 41, 54, 41, 51, 26, 40, 35, 93, 89, 96,
                41, 35, 40, 71, 54, 51, 17, 18, 26, 96, 97, 99, 35, 19, 17, 17, 26, 35, 69, 80, 67,
                95, 87, 88, 66, 91, 80,
            ]
            .iter()
            .map(|v| v - 1) // convert to 0-based index
            .collect::<Vec<_>>(),
        );

        let num_faces = mesh.faces().count();
        let num_vertices = mesh.vertices().count();

        assert_eq!(num_faces, 192, "should remove non-manifold faces");
        assert_eq!(
            num_vertices, 100,
            "should duplicate non-manifold vertices and remove isolated ones"
        );
    }
}
