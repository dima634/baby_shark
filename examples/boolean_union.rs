use std::path::Path;

use baby_shark::{io::stl::StlWriter, mesh::polygon_soup::data_structure::PolygonSoup, voxel::{meshing::MarchingCubesMesher, sdf::{self, Sdf}}};
use nalgebra_glm::Vec3;


fn main() {
    let voxel_size = 0.02;
    let mut mesher = MarchingCubesMesher::default()
        .with_voxel_size(voxel_size);

    let create_sphere = || {
        let sphere1 = sdf::builder::SdfBuilder::with_voxel_size(voxel_size)
            //.cuboid(Vec3::zeros(), Vec3::new(1.0, 1.0, 1.0));
            .sphere(1.5, Vec3::new(1.0, 1.0, 1.0));

        let sphere2 = sdf::builder::SdfBuilder::with_voxel_size(voxel_size)
            //.cuboid(Vec3::new(0.5, 0.5, 0.5), Vec3::new(1.5, 1.5, 1.5));
            .sphere(1.5, Vec3::new(1.5, 1.0, 1.0));

        (sphere1, sphere2)
    };
    let create_cubes = || {
        let sphere1 = sdf::builder::SdfBuilder::with_voxel_size(voxel_size)
            .cuboid(Vec3::new(-3.0, -3.0, -3.0), Vec3::new(1.0, 1.0, 1.0));

        let sphere2 = sdf::builder::SdfBuilder::with_voxel_size(voxel_size)
            .cuboid(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(2.5, 2.5, 2.5));

        (sphere1, sphere2)
    };
    let writer = StlWriter::new();

    
    
    // let (sphere1, sphere2) = create_sphere();
    // let sphere1_mesh = PolygonSoup::from_vertices(mesher.mesh(sphere1));
    // let sphere2_mesh = PolygonSoup::from_vertices(mesher.mesh(sphere2));

    // writer.write_stl_to_file(&sphere1_mesh, Path::new("sphere1.stl")).expect("Failed to write sphere1.stl");
    // writer.write_stl_to_file(&sphere2_mesh, Path::new("sphere2.stl")).expect("Failed to write sphere2.stl");

    // let (sphere1, sphere2) = create_cubes();
    // let sphere1_mesh = PolygonSoup::from_vertices(mesher.mesh(sphere1));
    // let sphere2_mesh = PolygonSoup::from_vertices(mesher.mesh(sphere2));

    // writer.write_stl_to_file(&sphere1_mesh, Path::new("cube1.stl")).expect("Failed to write sphere1.stl");
    // writer.write_stl_to_file(&sphere2_mesh, Path::new("cube2.stl")).expect("Failed to write sphere2.stl");

    // let (sphere1, sphere2) = create();
    // let sphere1_mesh = PolygonSoup::from_vertices(mesher.mesh(sphere1));
    // let sphere2_mesh = PolygonSoup::from_vertices(mesher.mesh(sphere2));

    // let writer = StlWriter::new();
    // writer.write_stl_to_file(&sphere1_mesh, Path::new("sphere1.stl")).expect("Failed to write sphere1.stl");
    // writer.write_stl_to_file(&sphere2_mesh, Path::new("sphere2.stl")).expect("Failed to write sphere2.stl");

    // let builder = sdf::builder::SdfBuilder::new(voxel_size);
    // let ball1 = builder.sphere(1.0, Vec3::new(-0.8, 0.0, 0.0));
    // let ball2 = builder.sphere(1.0, Vec3::new(0.8, 0.0, 0.0));
    // let dick = builder.cuboid(Vec3::new(-0.6, -0.8, 0.0), Vec3::new(0.6, 0.8, 4.5));
    // let end = builder.sphere(0.8, Vec3::new(0.0, 0.0, 4.5));

    // let union = ball1.union(ball2).union(dick).union(end);
    // let union_mesh = PolygonSoup::from_vertices(mesher.mesh(union));

    // writer.write_stl_to_file(&union_mesh, Path::new("dick_union.stl")).expect("Failed to write union.stl");

    let (sphere1, sphere2) = create_sphere();
    let union = sphere2.union(sphere1);
    let union_mesh = PolygonSoup::from_vertices(mesher.mesh(union));
    writer.write_stl_to_file(&union_mesh, Path::new("spheres_union.stl")).expect("Failed to write union.stl");

    let (sphere1, sphere2) = create_sphere();
    let union = sphere2.intersect(sphere1);
    let union_mesh = PolygonSoup::from_vertices(mesher.mesh(union));
    writer.write_stl_to_file(&union_mesh, Path::new("spheres_inter.stl")).expect("Failed to write union.stl");

    let (sphere1, sphere2) = create_sphere();
    let union = sphere2.subtract(sphere1);
    let union_mesh = PolygonSoup::from_vertices(mesher.mesh(union));
    writer.write_stl_to_file(&union_mesh, Path::new("spheres_sub.stl")).expect("Failed to write union.stl");

    let (sphere1, sphere2) = create_cubes();
    let union = sphere2.union(sphere1);
    let union_mesh = PolygonSoup::from_vertices(mesher.mesh(union));
    writer.write_stl_to_file(&union_mesh, Path::new("cubes_union.stl")).expect("Failed to write union.stl");

    let (sphere1, sphere2) = create_cubes();
    let union = sphere2.subtract(sphere1);
    let union_mesh = PolygonSoup::from_vertices(mesher.mesh(union));
    writer.write_stl_to_file(&union_mesh, Path::new("cubes_sub.stl")).expect("Failed to write union.stl");

    let (sphere1, sphere2) = create_cubes();
    let union = sphere2.intersect(sphere1);
    let union_mesh = PolygonSoup::from_vertices(mesher.mesh(union));
    writer.write_stl_to_file(&union_mesh, Path::new("cubes_inter.stl")).expect("Failed to write union.stl");

    let s = sdf::builder::SdfBuilder::with_voxel_size(voxel_size)
        .sphere(1.0, Vec3::new(0.4, 0.4, 0.4));
    let c = sdf::builder::SdfBuilder::with_voxel_size(voxel_size)
        .cuboid(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(1.0, 1.0, 1.0));
    let union = c.subtract(s);
    let union_mesh = PolygonSoup::from_vertices(mesher.mesh(union));
    writer.write_stl_to_file(&union_mesh, Path::new("cube_sphere.stl")).expect("Failed to write union.stl");

    // let radius = 1.5;
    // let origin1 = Vec3::new(1.0, 1.0, 1.0);
    // let origin2 = Vec3::new(1.5, 1.0, 1.0);

    // let s = Sdf::from_fn(voxel_size, Vec3::new(-3.0, -3.0, -3.0), Vec3::new(5.0, 5.0, 5.0), 1, |p| {
    //     let d1 = (p - origin1).norm() - radius;
    //     let d2 = (p - origin2).norm() - radius;

    //     d1.max(-d2)
    // });
    // let s = PolygonSoup::from_vertices(mesher.mesh(s));

    // writer.write_stl_to_file(&s, Path::new("aaaa.stl")).expect("Failed to write sphere1.stl");

    // let cube = sdf::builder::SdfBuilder::new(voxel_size)
    //     .cuboid(Vec3::new(-2.0, -2.0, -2.0), Vec3::new(2.0, 2.0, 2.0));
    // let iwp = sdf::builder::SdfBuilder::new(voxel_size)
    //     .iwp(Vec3::new(-3.0, -3.0, -3.0), Vec3::new(3.0, 3.0, 3.0));
    // let iwp = cube.subtract(iwp);
    // let iwp = PolygonSoup::from_vertices(mesher.mesh(iwp));
    // writer.write_stl_to_file(&iwp, Path::new("iwp.stl")).expect("Failed to write union.stl");
}
