extern crate marching_cubes;
use marching_cubes::marching_cubes_fidget;
use nalgebra::point;
use std::time::Instant;

fn main() {
    let now = Instant::now();
    let expr = "x*x + y*y + z*z - 2500";

    let mesh = marching_cubes_fidget(
        &expr,                    // expression to evaluate
        point![-100., -100., -100.], // minimum bounding box point
        200,                      // x count
        200,                      // y count
        200,                      // z count
        0.,                       // isosurface value
        1.,                       // scale
    );
    // // exporting to stl
    let file_path = "examples/evaluated.stl";
    mesh.export_stl(file_path);

    let elapsed = now.elapsed().as_secs_f64();
    let s = elapsed % 60.;
    let min = (elapsed / 60.).floor() as u8;
    // println!("Exported: {}", file_path);
    println!("Time: {} min {:.2?} seconds\n", min, s);
}
