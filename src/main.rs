use marching_cubes::marching_cubes_evaluated;
use nalgebra::point;
use std::time::Instant;
use std::env;

fn main() {
    let now = Instant::now();

    let args: Vec<String> = env::args().collect();
    let expr = &args[1];

    let mesh = marching_cubes_evaluated(
        &expr,                       // function to evaluate
        point![-25., -25., -25.],    // minimum bounding box point
        100,                         // x count
        100,                         // y count
        100,                         // z count
        0.,                          // isosurface value
        1.,                          // scale
    );

    // // exporting to stl
    let file_path = "marched.stl";
    mesh.export_stl(file_path);

    let elapsed = now.elapsed().as_secs_f64();
    let s = elapsed % 60.;
    let min = (elapsed / 60.).floor() as u8;
    println!("Exported: {}", file_path);
    println!("Time: {} min {:.2?} seconds\n", min, s);
}