use clap::Parser;
use marching_cubes::marching_cubes_evaluated;
use std::time::Instant;
mod cli;
use cli::Args;

fn main() {
    let now = Instant::now();
    let args = Args::parse();
    let domain = args.construct_domain();
    let expr = args.expr;
    let file_path = &args.export_path;
    
    let mesh = marching_cubes_evaluated(
        &expr,                       // function to evaluate
        domain.min_point,    // minimum bounding box point
        domain.x,                         // x count
        domain.y,                         // y count
        domain.z,                         // z count
        0.,                          // isosurface value
        domain.scale,                          // scale
    );

    // // exporting to stl
    mesh.export_stl(file_path);

    let elapsed = now.elapsed().as_secs_f64();
    let s = elapsed % 60.;
    let min = (elapsed / 60.).floor() as u8;
    println!("Exported: {}", file_path);
    println!("Time: {} min {:.2?} seconds\n", min, s);
}

