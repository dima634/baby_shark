# Geometry processing library in pure Rust

## Features
- Corner table implementation for efficient mesh traversal
- STL reader/writer
- Remeshing
- Spatial index
    * Bounding volume hierarchy of axis aligned bounding boxes
    * Infinite grid

## Isotropic remeshing
This algorithm incrementally performs simple operations such as edge splits, edge collapses, edge flips, and Laplacian smoothing. 
All the vertices of the remeshed patch are reprojected to 
the original surface to keep a good approximation of the input.
Any of those operations can be turned off using appropriate method (`with_<operation>(false)`).

![image](https://user-images.githubusercontent.com/48240075/191063968-b985b2c1-ab6c-46b4-88ef-fd637fe3323b.png)
### Example
```rust
let remesher = IncrementalRemesher::new()
    .with_iterations_count(10)
    .with_split_edges(true)
    .with_collapse_edges(true)
    .with_flip_edges(true)
    .with_shift_vertices(true)
    .with_project_vertices(true);
remesher.remesh(&mut mesh, 0.002f32);
```

## TODO:
- [ ] Mesh trait: move vertex normal to topological mesh
- [ ] Mesh trait: default implementation for edge/face position using get_edge_vertices/get_face_vertices
- [ ] Remesher: check quality of new faces on split/collapse/flip
- [ ] Remesher/Corner table: handle edge boundary collapses
- [ ] CornerTable/Remesher - preallocate estimated amount of elements for internal arrays
- [ ] Reusable vectors across app to reduce allocations for iter macro
- [ ] Corner table: better index typing
- [ ] AABB tree: pre allocate memory during construction
- [ ] Grid: consider exploiting min-max distance or incremental sphere growth
