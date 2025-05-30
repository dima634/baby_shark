## RUST unstable
 - [ ] VDB: const expressions with generics
 - [ ] VDB: tree traverse using coroutines/generators

## TODO/IDEAS:
- [ ] Reuse deleted corners/vertices
- [ ] Avoid creating degenerate triangles during remeshing and compare normal to epsilon
- [ ] Refactor prelude
- [ ] Use u32 as index for vertices and corners
- [ ] `vertices_around_vertex` - return control flow

- [ ] Add lightweighting to README
- [ ] Prelude module
- [ ] Fix clippy warnings
- [ ] Replace code examples with links to examples
- [ ] Volume booleans: check that volumes have the same resolution
- [ ] Volume mesher: get voxels size from the volume
- [ ] Dual contouring docs
- [ ] Default constructor for grid
- [ ] Remove sub from grid value
- [ ] Voxel remeshing, add options for feature-preserving meshing
- [ ] Cached accessor
    - Typed list to avoid vtable calls https://crates.io/crates/frunk#hlist
    - Fix issue with caching
- [ ] Stencil accessor
- [ ] Sequential accessor
- [ ] Add assertions for BRANCHING (>=2 for leaf, decreasing for tree), SIZE
- [ ] Replace `Box` with index in `Vec` and check performance (cache misses)
- [ ] Visualization
- [ ] Ability to pick between BTree and HashMap for root node
- [ ] Type list: change to enum. Each enum value is type at {idx} of list. Will compiler optimize match expr? Implement the same ways as TreeNode
- [ ] Cubes meshing: optimize by testing only boundary of tile nodes
- [ ] Remove redundant total branching from leaf node
- [ ] Marching cubes: verify cases handling, especially subconfig usage
- [ ] Fast winding numbers: order3 approx
- [ ] AABB tree optimizations: pre-compute bbox centers etc
