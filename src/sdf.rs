use marching_cubes::{Point, Vector, remap};
use nalgebra::{distance, vector};

// ===========================================================
// =========== Some Distance Equations to Test With ==========
// ===========================================================
// mostly thanks to https://iquilezles.org/

pub fn sphere(p: Point, center: Point, r: f64) -> f64 {
    distance(&p, &center) - r
}

pub fn rounded_box(p: Point, c: Point, s: Vector, r: f64) -> f64 {
    // Modified to account for the radius without changing the size of the box
    // 
    let po = p - c;
    let pf: Vector = vector![po.x.abs(), po.y.abs(), po.z.abs()] - (s - vector![r, r, r]);
    return vector![pf.x.max(0.0), pf.y.max(0.0), pf.z.max(0.0)].norm()
        + pf.x.max(pf.y.max(pf.z)).min(0.0)
        - r;
}

pub fn gyroid(p: Point, f: f64, t: f64) -> f64 {
    // f: frequency
    // t: thickness

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

// (same as lerp)
pub fn mix(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

pub fn ramp(v: f64, in_min: f64, in_max: f64, out_min: f64, out_max: f64) -> f64 {
    if v < in_min {
        return out_min;
    } else if v > in_max {
        return out_max;
    } else {
        return remap(v, [in_min, in_max], [out_min, out_max]);
    }
}