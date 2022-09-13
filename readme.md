# Geometry processing library in pure Rust

### Features
- Corner table implementation for efficient mesh traversal
- STL reader/writer
- Bounding volume hierarchy of axis aligned bounding boxes
- Remeshing

### TODO:
- [ ] Mesh trait: move vertex normal to topological mesh
- [ ] Mesh trait: default implementation for edge/face position using get_edge_vertices/get_face_vertices
- [ ] Remesher/Corner table: flip
- [ ] Remesher/Corner table: handle edge boundary collapse
- [ ] StlWriter - calculate normal from vertices (now iterator over face vertices are executed twice)
- [ ] CornerTable/Remesher - preallocate estimated amount of elements for internal arrays
- [ ] Reusable vectors across app to reduce allocations for iter macro
- [ ] Corner table: better index typing
- [ ] AABB tree: pre allocate memory during construction
- [ ] Grid: consider exploiting min-max distance or incremental sphere growth