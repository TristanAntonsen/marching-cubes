use marching_cubes::{export_stl, marching_cubes, Point};
use nalgebra::{point, vector};
use std::time::Instant;

#[allow(dead_code)]
mod sdf;

fn main() {
    let now = Instant::now();

    // The function that gets marched
    fn map(p: Point) -> f64 {

        let s1 = sdf::sphere(p, point![0., 0., 0.], 100.0);
        let s2 = sdf::sphere(p, point![0., 0., 0.], 75.0);
        let b = sdf::rounded_box(p, point![0., 0., 0.], vector![180., 180., 180.], 10.);
        let g = sdf::boolean_intersection(sdf::gyroid(p, 0.15, 0.5), s2, 4.);

        sdf::boolean_union(g, sdf::boolean_subtraction(b, s1, 15.), 10.0)
        
    }

    // eval function version
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
    export_stl("marched.stl", mesh);

    let elapsed = now.elapsed().as_secs_f64();
    let s = elapsed % 60.;
    let min = (elapsed / 60.).floor() as u8;
    println!("\n{} min {:.2?} seconds", min, s);
}


