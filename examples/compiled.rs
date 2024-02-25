extern crate marching_cubes;
use marching_cubes::{marching_cubes_compiled, CompiledFunction, Point, SDF};
use nalgebra::{point, vector};
use std::{sync::Mutex, time::Instant};

fn main() {
    let now = Instant::now();

    // The function that gets marched
    fn map(p: Point) -> f64 {
        let s = SDF::sphere(p, point![30., 30., 30.], 65.0);
        let b = SDF::rounded_box(p, point![-30., -30., -30.], vector![60., 60., 60.], 10.);
        SDF::boolean_union(b - 1. * (0.5 * s).sin(), s, 20.)
    }

    // Create a closure that implements the CompiledFunction trait. This enables multi-threading
    let thread_safe_map: &Mutex<CompiledFunction> = &Mutex::new(Box::new(|point| map(point)));

    let mesh = marching_cubes_compiled(
        &thread_safe_map,            // function to evaluate
        point![-100., -100., -100.], // minimum bounding box point
        200,                         // x count
        200,                         // y count
        200,                         // z count
        0.,                          // isosurface value
        1.,                          // scale
    );

    // // exporting to stl
    let file_path = "examples/compiled.stl";
    mesh.export_stl(file_path);

    let elapsed = now.elapsed().as_secs_f64();
    let s = elapsed % 60.;
    let min = (elapsed / 60.).floor() as u8;
    println!("Exported: {}", file_path);
    println!("Time: {} min {:.2?} seconds\n", min, s);
}

// fn sinc(p: Point) -> f64 {
//     let f = 0.375; // frequency
//     let a = 20.;   // amplitude
//     return p.z - a * (f * (p.x.powf(2.) + p.y.powf(2.)).sqrt()).sinc()
//
