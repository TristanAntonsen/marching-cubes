extern crate marching_cubes;
use marching_cubes::{marching_cubes_buffer, Buffer3D, Point, SDF};
use nalgebra::{point, vector};
use std::time::Instant;

fn main() {
    let now = Instant::now();

    // The function that gets marched
    fn map(p: Point) -> f64 {
        let s = SDF::sphere(p, point![30., 30., 30.], 65.0);
        let b = SDF::rounded_box(p, point![-30., -30., -30.], vector![60., 60., 60.], 10.);
        SDF::boolean_union(b - 1. * (0.5 * s).sin(), s, 20.)
    }
    
    let mut buffer = Buffer3D::new(200, 200, 200);
    buffer.fill(&map);
    let t1 = now.elapsed().as_secs_f64();

    let mesh = marching_cubes_buffer(
        &buffer,    // discrete 3D data
        0.          // isosurface value
    );

    // // exporting to stl
    let file_path = "examples/buffer.stl";
    mesh.export_stl(file_path);

    let t2 = now.elapsed().as_secs_f64();
    let s1 = (t1) % 60.;
    let min1 = ((t1) / 60.).floor() as u8;
    let s2 = (t2 - t1) % 60.;
    let min2 = ((t2 - t1) / 60.).floor() as u8;
    let s = (t2) % 60.;
    let min = ((t2) / 60.).floor() as u8;
    println!("Exported: {}", file_path);
    println!("Total time:            {} min {:.2?} seconds", min, s);
    println!("Time to create buffer: {} min {:.2?} seconds", min1, s1);
    println!("Marching cubes time:   {} min {:.2?} seconds", min2, s2);
}

// fn sinc(p: Point) -> f64 {
//     let f = 0.375; // frequency
//     let a = 20.;   // amplitude
//     return p.z - a * (f * (p.x.powf(2.) + p.y.powf(2.)).sqrt()).sinc()
//
