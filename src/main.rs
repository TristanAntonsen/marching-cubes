use marching_cubes::{export_stl, marching_cubes, Point};
use nalgebra::{distance, point};
use std::time::Instant;

fn main() {
    let now = Instant::now();

    // eval function version
    let mesh = marching_cubes(
        &map,
        point![-100., -100., -100.],
        200,
        200,
        200,
        0.,
        1.,
    );

    // exporting to stl
    export_stl("marched.stl", mesh);

    let elapsed = now.elapsed().as_secs_f64();
    let s = elapsed % 60.;
    let min = (elapsed / 60.).floor() as u8;
    println!("\n{} min {:.2?} seconds", min, s);
}

// ===========================================================
// =========== Some Distance Equations to Test With ==========
// ===========================================================
// mostly thanks to https://iquilezles.org/

// The function that gets marched
fn map(p: Point) -> f64 {
    let s1 = _sphere(p, point![0., -25., -25.], 50.0);
    let s2 = _sphere(p, point![0., 25., 25.], 50.0);
    op_union(s1, s2, 15.0)
}

fn _sphere(p: Point, center: Point, r: f64) -> f64 {
    distance(&p, &center) - r
}

pub fn _gyroid(p: Point, center: Point, half_width: f64, f: f64, t: f64) -> f64 {
    // f: frequency
    // t: thickness

    let min_pt = point![
        center.x - half_width,
        center.y - half_width,
        center.z - half_width
    ];
    let max_pt = point![
        center.x + half_width,
        center.y + half_width,
        center.z + half_width
    ];

    if p.x < min_pt[0] || p.x > max_pt[0] {
        return 1.0;
    }
    if p.y < min_pt[1] || p.y > max_pt[1] {
        return 1.0;
    }
    if p.z < min_pt[2] || p.z > max_pt[2] {
        return 1.0;
    }

    let g = (f * p.x).sin() * (f * p.y).cos()
        + (f * p.y).sin() * (f * p.z).cos()
        + (f * p.z).sin() * (f * p.x).cos();

    // g + 0.75
    (g + f).abs() - t
}

////////// Boolean operations //////////
pub fn op_union(d1: f64, d2: f64, r: f64) -> f64 {
    // f64::min(d1, d2)
    smooth_min(d1, d2, r)
}

pub fn op_list_union(sdfs: Vec<f64>, r: f64) -> f64 {
    sdfs.iter().fold(f64::INFINITY, |a, &b| smooth_min(a, b, r))
}

pub fn op_subtraction(d1: f64, d2: f64, r: f64) -> f64 {
    // f64::max(d1, -d2)
    -smooth_min(-d1, d2, r)
}

pub fn op_intersection(d1: f64, d2: f64, r: f64) -> f64 {
    // f64::max(d1, d2)
    -smooth_min(-d1, -d2, r)
}

pub fn smooth_min(a: f64, b: f64, mut k: f64) -> f64 {
    // polynomial smooth min

    if k < 0.00001 {
        k = 0.00001
    }

    let h = (k - (a - b).abs()).max(0.0) / k;

    a.min(b) - h * h * k * (1.0 / 4.0)
}
