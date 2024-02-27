extern crate marching_cubes;
use marching_cubes::{
    marching_cubes_compiled, marching_cubes_evaluated, marching_cubes_fidget, CompiledFunction, Point, SDF
};
use nalgebra::point;
use std::{sync::Mutex, time::Instant};

fn main() {
    let now = Instant::now();

    print!("\nCompiled ahead of time:");
    sample_aot();
    let t1 = now.elapsed().as_secs_f64();
    print!("Symbolic (evalexpr)");
    sample_evalexpr();
    let t2 = now.elapsed().as_secs_f64() - t1;
    print!("Fidget");
    sample_fidget();
    let t3 = now.elapsed().as_secs_f64() - t2;

    let s1 = (t1) % 60.;
    let s2 = (t2) % 60.;
    let s3 = (t3) % 60.;

    println!("Total time:  {:.2?} seconds", s1 + s2 + s3);
    println!("Compiled:    {:.2?} seconds", s1);
    println!("evalexpr:    {:.2?} seconds", s2);
    println!("fidget:      {:.2?} seconds\n", s3);
}

const DOMAIN_SIZE: usize = 125;

fn sample_aot() {

    fn map(p: Point) -> f64 {
        SDF::sphere(p, point![0., 0., 0.], 50.0)
    }
    let half_size = DOMAIN_SIZE as f64 / 2.;
    let thread_safe_map: &Mutex<CompiledFunction> = &Mutex::new(Box::new(|point| map(point)));

    let mesh = marching_cubes_compiled(
        &thread_safe_map,            // function to evaluate
        point![-half_size, -half_size, -half_size], // minimum bounding box point
        DOMAIN_SIZE,                         // x count
        DOMAIN_SIZE,                         // y count
        DOMAIN_SIZE,                         // z count
        0.,                          // isosurface value
        1.,                          // scale
    );

    mesh.export_stl("examples/benchmark_aot.stl")
}

fn sample_evalexpr() {

    let expr = &"x*x+y*y+z*z-2500";
    let half_size = DOMAIN_SIZE as f64 / 2.;

    let mesh = marching_cubes_evaluated(
        &expr,                       // expression to evaluate
        point![-half_size, -half_size, -half_size], // minimum bounding box point
        DOMAIN_SIZE,                         // x count
        DOMAIN_SIZE,                         // y count
        DOMAIN_SIZE,                         // z count
        0.,                          // isosurface value
        1.,                          // scale
    );
    
    // // exporting to stl
    let file_path = "examples/benchmark_evalexpr.stl";
    mesh.export_stl(file_path);
}

fn sample_fidget() {

    let expr = &"x*x+y*y+z*z-2500";
    let half_size = DOMAIN_SIZE as f64 / 2.;

    let mesh = marching_cubes_fidget(
        &expr,                       // expression to evaluate
        point![-half_size, -half_size, -half_size], // minimum bounding box point
        DOMAIN_SIZE,                         // x count
        DOMAIN_SIZE,                         // y count
        DOMAIN_SIZE,                         // z count
        0.,                          // isosurface value
        1.,                          // scale
    );
    
    // // exporting to stl
    let file_path = "examples/benchmark_fidget.stl";
    mesh.export_stl(file_path);
}
