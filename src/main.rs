use clap::Parser;
use marching_cubes::{marching_cubes_evaluated, marching_cubes_fidget, Mesh};
use std::{string, time::Instant};
mod cli;
use cli::Args;

fn main() {
    let now = Instant::now();
    let args = Args::parse();
    let domain = args.construct_domain();
    let expr = args.expr;
    let file_path = &args.export_path;
    let mode = args.mode.as_str();

    let mesh: Mesh = match mode {
        "evalexpr" => marching_cubes_evaluated(
            &expr,            // function to evaluate
            domain.min_point, // minimum bounding box point
            domain.x,         // x count
            domain.y,         // y count
            domain.z,         // z count
            0.,               // isosurface value
            domain.scale,     // scale
        ),
        "fidget" => marching_cubes_fidget(
            &expr,            // function to evaluate
            domain.min_point, // minimum bounding box point
            domain.x,         // x count
            domain.y,         // y count
            domain.z,         // z count
            0.,               // isosurface value
            domain.scale,     // scale
        ),
        _ => panic!("Invalid mode flag."),
    };

    // exporting to stl
    mesh.export_stl(file_path);

    let elapsed = now.elapsed().as_secs_f64();
    let s = elapsed % 60.;
    let min = (elapsed / 60.).floor() as u8;
    println!("Exported: {}", file_path);
    println!("Time: {} min {:.2?} seconds\n", min, s);
}
