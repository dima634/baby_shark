
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
- [ ] Insert (full node)
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
- [ ] Internal node size: vec of union
