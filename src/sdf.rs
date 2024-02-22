use nalgebra::{point, distance};
use marching_cubes::Point;

// ===========================================================
// =========== Some Distance Equations to Test With ==========
// ===========================================================
// mostly thanks to https://iquilezles.org/


pub fn sphere(p: Point, center: Point, r: f64) -> f64 {
    distance(&p, &center) - r
}

pub fn gyroid(p: Point, center: Point, half_width: f64, f: f64, t: f64) -> f64 {
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
pub fn boolean_union(d1: f64, d2: f64, r: f64) -> f64 {
    // f64::min(d1, d2)
    smooth_min(d1, d2, r)
}

pub fn boolean_union_list(sdfs: Vec<f64>, r: f64) -> f64 {
    sdfs.iter().fold(f64::INFINITY, |a, &b| smooth_min(a, b, r))
}

pub fn boolean_subtraction(d1: f64, d2: f64, r: f64) -> f64 {
    // f64::max(d1, -d2)
    -smooth_min(-d1, d2, r)
}

pub fn boolean_intersection(d1: f64, d2: f64, r: f64) -> f64 {
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