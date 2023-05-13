use marching_cubes::{VoxelGrid, Point, ORIGIN, export_stl};
use nalgebra::{Point3, distance};

fn main() {

    let bounds = [
        Point3::new(-50., -50., -50.),
        Point3::new(50., 50., 50.)
    ];
    let mut voxels = VoxelGrid::new_from_aabb(bounds, 1.0);
    voxels.eval(&sdf);
    let mesh = voxels.march(0.0);
    export_stl("marched.stl", mesh);
    voxels.export_voxel_data("voxels.csv").expect("Could not write .csv");

}


fn sdf(p : Point) -> f64 {
    sphere(p, 10.0)
}

fn sphere(point : Point, r : f64) -> f64 {
    distance(&ORIGIN, &point) - r
}