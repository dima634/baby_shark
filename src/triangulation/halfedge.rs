#[derive(Debug)]
pub struct HalfedgeMesh {
    outgoing_he: Vec<usize>,      // halfedges outgoing from vertex
    triangles: Vec<usize>,        // list of triangle indices
    he_twins: Vec<Option<usize>>, // list of halfedge twins
}

impl HalfedgeMesh {
    pub fn new() -> Self {
        return Self { outgoing_he: Vec::new(), triangles: Vec::new(), he_twins: Vec::new() };
    }

    #[inline]
    pub fn triangles(&self) -> &Vec<usize> {
        return &self.triangles;
    }

    #[inline]
    pub fn clear(&mut self) {
        self.triangles.clear();
        self.he_twins.clear();
        self.outgoing_he.clear();
    }

    #[inline]
    pub fn reserve(&mut self, num_faces: usize, num_vertices: usize) {
        let halfedges_count = num_faces * 3;
        self.triangles.reserve(halfedges_count);
        self.he_twins.reserve(halfedges_count);
        self.outgoing_he.resize(num_vertices, usize::MAX);
    }

    #[inline]
    pub fn halfedge_vertices(&self, he: usize) -> (usize, usize) {
        return (self.triangles[he], self.triangles[next_halfedge(he)]);
    }

    #[inline]
    pub fn opposite_halfedge(&self, he: usize) -> Option<usize> {
        return self.he_twins[he];
    }

    /// Insert new triangle to triangulation
    pub fn add_triangle(&mut self, v1: usize, v2: usize, v3: usize, he1: Option<usize>, he2: Option<usize>, he3: Option<usize>) -> usize {
        let triangle_idx = self.triangles.len();

        self.triangles.push(v1);
        self.triangles.push(v2);
        self.triangles.push(v3);

        self.outgoing_he[v1] = triangle_idx;
        self.outgoing_he[v2] = triangle_idx + 1;
        self.outgoing_he[v3] = triangle_idx + 2;
        
        self.add_halfedge(he1);
        self.add_halfedge(he2);
        self.add_halfedge(he3);

        return triangle_idx;
    }
    
    #[inline]
    pub fn outgoing_halfedge(&self, vertex: usize) -> usize {
        return self.outgoing_he[vertex];
    }

    pub fn outgoing_border_halfedge(&self, v: usize) -> Option<usize> {
        let start_he = self.outgoing_halfedge(v);
        let mut current_he = start_he;

        loop {
            let twin = self.he_twins[current_he];

            if twin.is_none() {
                return Some(current_he);
            }

            current_he = next_halfedge(twin.unwrap());

            if start_he == current_he {
                return None;
            }
        }
    }

    pub fn flip_edge(&mut self, he: usize) -> usize {
        let he_next = next_halfedge(he);
        let he_opposite = self.opposite_halfedge(he).unwrap();
        let he_opposite_next = next_halfedge(he_opposite);

        let op1 = self.opposite_halfedge(he_next);
        let he_next_next = next_halfedge(he_next);
        let op2 = self.opposite_halfedge(he_next_next);
    
        let op3 = self.opposite_halfedge(he_opposite_next);
        let he_opposite_next_next = next_halfedge(he_opposite_next);
        let op4 = self.opposite_halfedge(he_opposite_next_next);

        let (v2, v3) = self.halfedge_vertices(he_next);
        let (v1, v4) = self.halfedge_vertices(he_opposite_next);

        self.triangles[he_next] = v4;
        self.triangles[he_opposite_next] = v3;

        self.make_opposite(he, op3);
        self.make_opposite(he_next, Some(he_opposite_next));
        self.make_opposite(he_next_next, op2);

        self.make_opposite(he_opposite, op1);
        self.make_opposite(he_opposite_next, Some(he_next));
        self.make_opposite(he_opposite_next_next, op4);

        self.outgoing_he[v1] = he;
        self.outgoing_he[v2] = he_opposite;
        self.outgoing_he[v3] = he_next_next;
        self.outgoing_he[v4] = he_opposite_next_next;

        return he_next;
    }

    pub fn split_edge(&mut self, he: usize, v: usize) {
        let he_next = next_halfedge(he);
        let he_prev = prev_halfedge(he);
        let he_opp = self.opposite_halfedge(he);
        let he_next_opp = self.opposite_halfedge(he_next);

        let v1 = self.triangles[he_next];
        let v2 = self.triangles[he_prev];

        // Split face incident to he
        self.triangles[he_next] = v;
        let new_face1 = self.triangles.len();
        self.triangles.push(v);
        self.triangles.push(v1);
        self.triangles.push(v2);

        self.add_halfedge(None); // v, v1 !!!! update later if he_opp exist  
        self.add_halfedge(he_next_opp);
        self.add_halfedge(Some(he_next));
        // self.he_twins[he_next] = Some(new_face1 + 2);
        self.make_opposite(he_next, Some(new_face1 + 2));

        if let Some(he_opp) = he_opp {
            // Slit face opposite to he
            let he_opp_prev = prev_halfedge(he_opp);
            let he_opp_prev_opp = self.opposite_halfedge(he_opp_prev);
            let v3 = self.triangles[he_opp_prev];

            self.triangles[he_opp] = v;
            let new_face2 = self.triangles.len();
            self.triangles.push(v);
            self.triangles.push(v3);
            self.triangles.push(v1);

            self.add_halfedge(Some(he_opp_prev));
            self.add_halfedge(he_opp_prev_opp);
            self.add_halfedge(Some(new_face1));
            // self.he_twins[he_opp_prev] = Some(new_face2);
            self.make_opposite(he_opp_prev, Some(new_face2));
            // self.he_twins[new_face1] = Some(new_face2 + 2);
            self.make_opposite(new_face1, Some(new_face2 + 2));
        }

        self.outgoing_he.push(he_next);
        self.outgoing_he[v1] = new_face1 + 1;
    }

    /// Returns halfedge going from vertex with index `v_start` to `v_end`
    pub fn connection_halfedge(&self, v_start: usize, v_end: usize) -> Option<usize> {
        let start_he = self.outgoing_halfedge(v_start);
        let mut current_he = start_he;

        loop {
            let (_, v2) = self.halfedge_vertices(current_he);

            if v2 == v_end {
                return Some(current_he);
            }

            if let Some(opp) = self.opposite_halfedge(current_he) {
                current_he = next_halfedge(opp);

                if current_he == start_he {
                    return None;
                }
            } else {
                break;
            }
        }

        current_he = prev_halfedge(start_he);

        loop {
            let (v1, _) = self.halfedge_vertices(current_he);

            if v1 == v_end {
                return Some(current_he);
            }

            if let Some(opp) = self.opposite_halfedge(current_he) {
                current_he = prev_halfedge(opp);
            } else {
                break;
            }
        }

        return None;
    }

    #[inline]
    fn add_halfedge(&mut self, opposite: Option<usize>) {
        self.he_twins.push(opposite);
        if let Some(opposite) = opposite {
            self.he_twins[opposite] = Some(self.he_twins.len() - 1);
        }
    }

    #[inline]
    fn make_opposite(&mut self, he1: usize, he2: Option<usize>) {
        self.he_twins[he1] = he2;

        if let Some(he2) = he2 {
            self.he_twins[he2] = Some(he1);
        }
    }
}

#[inline]
pub fn next_halfedge(he: usize) -> usize {
    return if (he % 3) == 2 { he - 2 } else { he + 1 };
}

#[inline]
pub fn prev_halfedge(he: usize) -> usize {
    return if (he % 3) == 0 { he + 2 } else { he - 1 };
}

#[cfg(test)]
mod tests {
    use super::HalfedgeMesh;

    #[test]
    fn test_split_edge() {
        let mut mesh = HalfedgeMesh::new();
        mesh.reserve(2, 4);
        mesh.add_triangle(0, 1, 2, None, None, None);
        mesh.add_triangle(1, 0, 3, Some(0), None, None);

        mesh.split_edge(0, 4);
        
        assert_eq!(mesh.triangles, vec![
            0, 4, 2, // 0
            4, 0, 3, // 3
            4, 1, 2, // 6
            4, 3, 1  // 9
        ]);
        assert_eq!(mesh.he_twins, vec![
            Some(3), Some(8), None,
            Some(0), None, Some(9),
            Some(11), None, Some(1),
            Some(5), None, Some(6)
        ]);
    }
}
