
## TODO:
- [ ] Mesh trait: move vertex normal to topological mesh
- [ ] Mesh trait: default implementation for edge/face position using get_edge_vertices/get_face_vertices
- [ ] Remesher/Corner table: handle edge boundary collapses
- [ ] CornerTable/Remesher - preallocate estimated amount of elements for internal arrays
- [ ] Reusable vectors across app to reduce allocations for iter macro
- [ ] Corner table: better index typing
- [ ] AABB tree: pre allocate memory during construction
- [ ] Grid: consider exploiting min-max distance or incremental sphere growth
- [ ] Reeb graph cleanup
- [ ] Triangle-triangle intersection: check sign by product that can be reused later
- [ ] Reexport `triangle3`, `plane3`... in `primitives` module, replace static methods with regular functions
- [ ] Generic `intersects` trait, replacement for all `intersects_primitive` traits
- [ ] Corner table: improve handling of non-manifold edges by duplication vertices instead of removing whole face
- [ ] Replace `num_traits::cast` with corresponding `to_primitive` methods

### VBD
- [ ] Replace `Box` with index in `Vec` and check performance (cache misses)
- [ ] Add assertions for BRANCHING (>=2 for leaf, decreasing for tree), SIZE
- [ ] Root is not tree node?
- [ ] Visualization
- [ ] Insert
    - [x] Full node
    - [x] There should not be a pruning on insert
- [ ] Cached accessor
    - Typed list to avoid vtable calls https://crates.io/crates/frunk#hlist
    - Fix issue with caching
- [ ] Stencil accessor
- [ ] Sequential accessor
- [ ] BitSet implementation
- [ ] Cubes iter:
    - Iterate over no-overlaping cubes
    - Iterate over seams cubes
    - Only on DEPTH - 1 level? skip rest. THIS ONE. Only active tiles/leaf should be considered. How to avoid duplicates?
    - AAAHHHHHHH create a separate tree containing intersected voxels (make borders active) and iterate over it
    - Make border active???
- [x] Internal node size: vec of union
- [ ] Marching cubes: intersection grid
- [ ] Ability to pick between BTree and HashMap for root node
- [x] Prune on demand (within tolerance?)
- [ ] Type list: change to enum. Each enum value is type at {idx} of list. Will compiler optimize match expr? Implement the same ways as TreeNode
- [ ] Cubes meshing: optimize by testing only boundary of tile nodes
- [ ] Remove redundant total branching from leaf node
- [ ] Internal node: merge childs and values into union to reduce memory footprint
- [x] Artifacts during meshing
- [ ] Leaf node: bit mask for inside/outside
- [ ] Mesh to SDF: implement union for tree nodes to speed up stuff???
- [ ] Marching cubes: verify cases handling, especially subconfig usage
- [ ] Fast winding numbers: order3 approx


TPMS:
```rust
        let cell_size = 3.0_f32;
        let density = 0.2_f32;
        let x = 2.0 * PI * p.x / cell_size;
        let y = 2.0 * PI * p.y / cell_size;
        let z = 2.0 * PI * p.z / cell_size;


        // Tub-Primitive 
        //-(x.cos() + y.cos() + z.cos() - 0.51 * (x.cos() * y.cos() + y.cos() * z.cos() + z.cos() * x.cos()) - 1.0)

        // Schoen-Gyroid
        // (x.sin() * y.cos() + y.sin() * z.cos() + z.sin() * x.cos() - density)

        // Schoen-I-WP (IWP)
        // (
        //     x.cos() * y.cos() + y.cos() * z.cos() + x.cos() * z.cos() - 
        //     0.5 * ((x * 2.0).cos() + (y * 2.0).cos() + (z * 2.0).cos() - density)
        // )

        // if bbox.contains_point(&((*p).into())) {
        //     f32::MIN
        // } else {
        //     bbox.squared_distance(&((*p).into())).sqrt()
        // }

        // (p - Vector3::new(0.0, 0.0, -5.0)).magnitude_squared().sqrt() - 3.0 // circle


        // let p = (*p).into();
        // let sphere = Sphere3::new(Point3::new(0.0, 0.0, -5.0), 3.0);
        // let d = sphere.squared_distance_to_point(&p);

        // if d < 0.0 {
        //     // println!("neg");
        //     -d.sqrt()
        // } else {
        //     d.sqrt()
        // }

        // p.y

        p.x.cos() + p.y.cos() + p.z.cos()
```
