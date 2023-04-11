use marching_cubes::{VoxelGrid, Vertex, ORIGIN};
use nalgebra::{Point3, distance};

fn main() {

    let bounds = [
        Point3::new(-50., -50., -50.),
        Point3::new(50., 50., 50.)

    ];
    let mut voxels = VoxelGrid::new_from_aabb(bounds, 1.0);
    voxels.eval(&sphere, 20.0);
    voxels.export_voxel_data("voxels.csv").expect("Could not write .csv");
    println!("{:?}", voxels);

}

fn sphere(point : Vertex, r : f64) -> f64 {
    distance(&ORIGIN, &point) - r
}