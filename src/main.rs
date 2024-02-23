use marching_cubes::{marching_cubes, Point};
use nalgebra::{point, vector};
use std::time::Instant;

#[allow(dead_code)]
mod sdf;

fn main() {
    let now = Instant::now();

    // The function that gets marched
    fn map(p: Point) -> f64 {
        let s = sdf::sphere(p, point![30., 30., 30.], 65.0);
        let b = sdf::rounded_box(p, point![-30., -30., -30.], vector![60., 60., 60.], 10.);
        sdf::boolean_union(b - 1. * (0.5 * s).sin(), s, 20.)
    }

    let mesh = marching_cubes(
        &map,                        // function to evaluate
        point![-100., -100., -100.], //minimum bounding box point
        200,                         // x count
        200,                         // y count
        200,                         // z count
        0.,                          // iso val
        1.,                          // scale
    );

    // exporting to stl
    mesh.export_stl("marched.stl");

    let elapsed = now.elapsed().as_secs_f64();
    let s = elapsed % 60.;
    let min = (elapsed / 60.).floor() as u8;
    println!("\n{} min {:.2?} seconds", min, s);
}
