use marching_cubes::{import_expression, marching_cubes_compiled, marching_cubes_evaluated, CompiledFunction, Mesh, Point, SDF};
use nalgebra::{point, vector};
use std::{sync::Mutex, time::Instant};
#[allow(dead_code)]

fn main() {
    let now = Instant::now();

    let mesh = evaluated_string_example();

    // // exporting to stl
    let file_path = "marched.stl";
    mesh.export_stl(file_path);

    let elapsed = now.elapsed().as_secs_f64();
    let s = elapsed % 60.;
    let min = (elapsed / 60.).floor() as u8;
    println!("Exported: {}", file_path);
    println!("Time: {} min {:.2?} seconds\n", min, s);
}

fn evaluated_string_example() -> Mesh {
    let expr = &import_expression("expr.txt").expect("Could not import expression.");

    marching_cubes_evaluated(
        &expr,            // function to evaluate
        point![-25., -25., -25.], // minimum bounding box point
        100,                         // x count
        100,                         // y count
        100,                         // z count
        0.,                          // isosurface value
        1.,                          // scale
    )
}
