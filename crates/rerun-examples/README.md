### Running examples

# Bounding sphere simplification

` cargo run --release --example bsphere_simplification -- -i assets/surface.stl --color 10,133,199 --xdiff 100 --ydiff 50  --origin 2,0,6 --radii-error 12:0.001,20:1 --output-file assets/surface-bsphere.stl`


## Simplification
To run, viewing the results of an error value of 0.5, 10, and 50, viewing wireframes

` cargo run --release --example simplification -- --input-file assets/stanford-bunny.stl -e 0.5 -e 10 -e 50 --mesh-offset 50 --wireframe-offset 100 --wireframe`