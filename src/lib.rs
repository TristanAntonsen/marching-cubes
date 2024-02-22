use byteorder::{LittleEndian, WriteBytesExt};
use csv;
use nalgebra::{point, Point3, Vector3};
use ndarray::Array3;
use std::{collections::HashMap, error::Error, fs, io::Write, ops::Add};

// ==========================================================
// ======================= Data types =======================
// ==========================================================
pub type Point = Point3<f64>;
pub type Vector = Vector3<f64>;
pub const ORIGIN: Point3<f64> = Point3::new(0.0, 0.0, 0.0);

// ===========================================================
// ================= Voxels & Marching Cubes =================
// ===========================================================

#[derive(Debug)]
pub struct VoxelGrid {
    pub values: Array3<f64>, // scalar values for each voxel
    pub x_count: usize,
    pub y_count: usize,
    pub z_count: usize,
    pub size: f64, // voxel resolution
    pub aabb: [Point3<f64>; 2],
    pub points: Array3<Point>, // xyz point coordinates
}

impl VoxelGrid {
    pub fn new_from_aabb(aabb: [Point3<f64>; 2], size: f64) -> Self {
        let x_count = ((aabb[1].x.floor() - aabb[0].x.floor()) / size) as usize;
        let y_count = ((aabb[1].y.floor() - aabb[0].y.floor()) / size) as usize;
        let z_count = ((aabb[1].z.floor() - aabb[0].z.floor()) / size) as usize;
        let zeros = Array3::<f64>::zeros((x_count, y_count, z_count));

        // scaled point coordinates in 3D space

        // vector from first quadrant to aabb
        let x0 = aabb[0].x;
        let y0 = aabb[0].y;
        let z0 = aabb[0].z;
        let xf = aabb[1].x;
        let yf = aabb[1].y;
        let zf = aabb[1].z;

        let mut points = Array3::<Point>::default((x_count, y_count, z_count));

        for x in 0..x_count {
            for y in 0..y_count {
                for z in 0..z_count {
                    let p = point![
                        remap(x as f64, [0.0, x_count as f64], [x0, xf]), // moves from first quadrant to aabb
                        remap(y as f64, [0.0, y_count as f64], [y0, yf]),
                        remap(z as f64, [0.0, z_count as f64], [z0, zf])
                    ];
                    points[[x, y, z]] = p;
                }
            }
        }

        Self {
            values: zeros,
            x_count: x_count,
            y_count: y_count,
            z_count: z_count,
            size: size,
            aabb: aabb,
            points: points,
        }
    }

    // Get the point coordinates at the 8 vertices of the cube (voxel version)
    pub fn get_corner_positions(&self, x: usize, y: usize, z: usize) -> Vec<Point> {
        // could be consolidated/more idiomatic
        let p0 = self.points[[x, y, z]];
        let p1 = self.points[[x + 1, y, z]];
        let p2 = self.points[[x + 1, y + 1, z]];
        let p3 = self.points[[x, y + 1, z]];
        let p4 = self.points[[x, y, z + 1]];
        let p5 = self.points[[x + 1, y, z + 1]];
        let p6 = self.points[[x + 1, y + 1, z + 1]];
        let p7 = self.points[[x, y + 1, z + 1]];

        let corner_points = vec![p0, p1, p2, p3, p4, p5, p6, p7];

        corner_points
    }

    // Get the values at the 8 vertices of the cube (voxel version)
    pub fn get_corner_values(&self, x: usize, y: usize, z: usize) -> Vec<f64> {
        // could be consolidated/more idiomatic
        let v0 = self.values[[x, y, z]];
        let v1 = self.values[[x + 1, y, z]];
        let v2 = self.values[[x + 1, y + 1, z]];
        let v3 = self.values[[x, y + 1, z]];
        let v4 = self.values[[x, y, z + 1]];
        let v5 = self.values[[x + 1, y, z + 1]];
        let v6 = self.values[[x + 1, y + 1, z + 1]];
        let v7 = self.values[[x, y + 1, z + 1]];

        let corner_vals = vec![v0, v1, v2, v3, v4, v5, v6, v7];

        corner_vals
    }

    // Marching cubes algorithm
    pub fn marching_cubes(&mut self, threshold: f64) -> Mesh {
        let mut target_mesh = Mesh::new_empty();

        let mut cube_count = 0;

        let edge_table = &EDGE_TABLE.map(|e| format!("{:b}", e));

        for x in 0..self.x_count - 1 {
            for y in 0..self.y_count - 1 {
                for z in 0..self.z_count - 1 {
                    // corner positions
                    let corner_positions = self.get_corner_positions(x, y, z);
                    // voxel values (evaluated sdf)
                    let eval_corners = self.get_corner_values(x, y, z);

                    // Calculating state
                    let state = get_state(&eval_corners, threshold);

                    // edges
                    // Example: 11001100
                    // Edges 2, 3, 6, 7 are intersected
                    let edges_bin_string = &edge_table[state];

                    // Indices of edge endpoints (List of pairs)
                    let (endpoint_indices, edges_to_use) =
                        get_edge_endpoints(edges_bin_string, &CORNER_POINT_INDICES);

                    // finding midpoints of edges
                    let edge_points = get_edge_midpoints(
                        endpoint_indices,
                        edges_to_use,
                        corner_positions,
                        eval_corners,
                        threshold,
                    );

                    // triangles
                    // Example: [7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
                    // Triangles: [p7, p3, p2], [p6, p7, p2]
                    let tris = &TRI_TABLE[state];

                    // adding triangle verts
                    for tri in tris {
                        if tri != &-1 {
                            let new_vert = Point3::new(
                                //converting Vec to array
                                edge_points[&(*tri as usize)][2],
                                edge_points[&(*tri as usize)][1],
                                edge_points[&(*tri as usize)][0],
                            );
                            target_mesh.vertices.push(new_vert);
                        }
                    }
                    cube_count += 1
                }
            }
        }
        // creating triangles
        let mut v = 0;
        while v < target_mesh.vertices.len() {
            target_mesh.triangle_from_verts(v, v + 1, v + 2);
            v += 3
        }
        println!("\nCube count: {}", cube_count);
        return target_mesh;
    }

    pub fn export_voxel_data(&self, path: &str) -> Result<(), Box<dyn Error>> {
        // https://levelup.gitconnected.com/working-with-csv-data-in-rust-7258163252f8
        // Creates new `Writer` for `stdout`
        let mut writer = csv::Writer::from_path(path)?;
        let mut point;
        for _x in 0..self.x_count - 1 {
            for _y in 0..self.y_count - 1 {
                for _z in 0..self.z_count - 1 {
                    point = self.points[[_x, _y, _z]];
                    writer
                        .write_record(&[
                            point.x.to_string(),
                            point.y.to_string(),
                            point.z.to_string(),
                            self.values[[_x, _y, _z]].to_string(),
                        ])
                        .expect("Something went wrong.");
                }
            }
        }

        // A CSV writer maintains an internal buffer, so it's important
        // to flush the buffer when you're done.
        writer.flush()?;

        Ok(())
    }
}

// ===========================================================
// ======================= Marching cubes ====================
// ===========================================================

fn fun_test(value: i32, f: &dyn Fn(i32) -> i32) -> i32 {
    println!("{}", f(value));
    value
}

fn times2(value: i32) -> i32 {
    2 * value
}

// Marching cubes algorithm
pub fn marching_cubes(
    eval_function: &dyn Fn(Point) -> f64,
    min_point: Point,
    x_count: usize,
    y_count: usize,
    z_count: usize,
    threshold: f64,
    scale: f64,
) -> Mesh {
    let mut target_mesh = Mesh::new_empty();

    let mut cube_count = 0;

    let edge_table = &EDGE_TABLE.map(|e| format!("{:b}", e));

    for x in 0..x_count - 1 {
        for y in 0..y_count - 1 {
            for z in 0..z_count - 1 {
                // corner positions
                let corner_positions = get_corner_positions(min_point, x, y, z, scale);

                // voxel values (evaluated sdf)
                let eval_corners = corner_positions.iter().map(|p| eval_function(*p)).collect();

                // Calculating state
                let state = get_state(&eval_corners, threshold);

                // edges
                // Example: 11001100
                // Edges 2, 3, 6, 7 are intersected
                let edges_bin_string = &edge_table[state];

                // Indices of edge endpoints (List of pairs)
                let (endpoint_indices, edges_to_use) =
                    get_edge_endpoints(edges_bin_string, &CORNER_POINT_INDICES);

                // finding midpoints of edges
                let edge_points = get_edge_midpoints(
                    endpoint_indices,
                    edges_to_use,
                    corner_positions,
                    eval_corners,
                    threshold,
                );

                // triangles
                // Example: [7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
                // Triangles: [p7, p3, p2], [p6, p7, p2]
                let tris = &TRI_TABLE[state];

                // adding triangle verts
                for tri in tris {
                    if tri != &-1 {
                        let new_vert = Point3::new(
                            //converting Vec to array
                            edge_points[&(*tri as usize)][2],
                            edge_points[&(*tri as usize)][1],
                            edge_points[&(*tri as usize)][0],
                        );
                        target_mesh.vertices.push(new_vert);
                    }
                }
                cube_count += 1
            }
        }
    }
    // creating triangles
    let mut v = 0;
    while v < target_mesh.vertices.len() {
        target_mesh.triangle_from_verts(v, v + 1, v + 2);
        v += 3
    }
    println!("\nCube count: {}", cube_count);
    return target_mesh;
}

// ===========================================================
// ============== Marching cubes helper functions ============
// ===========================================================

// Get the point coordinates at the 8 vertices of the cube (voxel version)
pub fn get_corner_positions(
    min_point: Point,
    x: usize,
    y: usize,
    z: usize,
    scale: f64,
) -> Vec<Point> {
    let xf = scale * x as f64;
    let yf = scale * y as f64;
    let zf = scale * z as f64;

    // could be consolidated/more idiomatic
    let p0 = point![xf, yf, zf];
    let p1 = point![xf + scale, yf, zf];
    let p2 = point![xf + scale, yf + scale, zf];
    let p3 = point![xf, yf + scale, zf];
    let p4 = point![xf, yf, zf + scale];
    let p5 = point![xf + scale, yf, zf + scale];
    let p6 = point![xf + scale, yf + scale, zf + scale];
    let p7 = point![xf, yf + scale, zf + scale];

    let mut corner_points = vec![p0, p1, p2, p3, p4, p5, p6, p7];

    // Translating points to bounding box space
    corner_points = corner_points
        .iter()
        .map(|p| add_points(*p, min_point))
        .collect();

    corner_points
}

fn add_points(p1: Point, p2: Point) -> Point {
    point![p1.x + p2.x, p1.y + p2.y, p1.z + p2.z]
}

// Return min and max bounding box points from a center point and box dimensions
pub fn center_box(center: Point, dims: Vector) -> [Point; 2] {
    let min_point = point![
        center.x - dims.x / 2.0,
        center.y - dims.y / 2.0,
        center.z - dims.z / 2.0
    ];
    let max_point = point![
        center.x + dims.x / 2.0,
        center.y + dims.y / 2.0,
        center.z + dims.z / 2.0
    ];
    [min_point, max_point]
}

// get the state of the 8 vertices of the cube
pub fn get_state(eval_corners: &Vec<f64>, threshold: f64) -> usize {
    // assumes eval_corners.len() == 8; Need to check for this
    // 0 if <= threshold, 1 if > threshold

    let states = eval_corners.iter().map(|x| state_function(*x, threshold));

    let mut i = 1.0;
    let mut final_state = 0.0;
    for s in states {
        final_state += s * i;
        i *= 2.0;
    }

    return final_state as usize;
}

// Function to determine state of each corner
pub fn state_function(v: f64, threshold: f64) -> f64 {
    if v <= threshold {
        1.0
    } else {
        0.0
    }
}

// Get the midpoints of the edges of the cube
pub fn get_edge_midpoints(
    endpoint_indices: Vec<[i8; 2]>,
    edges_to_use: Vec<usize>,
    corner_positions: Vec<Point>,
    corner_values: Vec<f64>,
    threshold: f64,
) -> HashMap<usize, Vec<f64>> {
    let (mut pair, mut edge);
    let (mut pi, mut pf, mut pe);
    let (mut vi, mut vf, mut t);

    let mut edge_points: HashMap<usize, Vec<f64>> = HashMap::new();

    for i in 0..endpoint_indices.len() {
        pair = endpoint_indices[i];
        edge = edges_to_use[i];
        if pair.len() > 0 {
            // finding points corresponding to endpoint indices
            vi = corner_values[pair[0] as usize];
            vf = corner_values[pair[1] as usize];
            pi = corner_positions[pair[0] as usize];
            pf = corner_positions[pair[1] as usize];

            t = find_t(vi, vf, threshold);

            pe = interpolate_points(pi, pf, t); // midpoint/interpolated point
            edge_points.insert(edge, pe);
        }
    }
    edge_points
}

// Return pairs of endpoints per edge of the cube
pub fn get_edge_endpoints(
    edges: &String,
    point_indices: &[[i8; 2]; 12],
) -> (Vec<[i8; 2]>, Vec<usize>) {
    // returns the endpoints of edges from EdgeTable lookup
    let mut edge_points = Vec::new();

    // prepare for the check to see if each character = 1
    // (doesn't seem like the right way to do this)

    // looping through binary string of yes/no for each edge
    let edges_to_use = edges_from_lookup(edges);
    for e in edges_to_use.clone() {
        edge_points.push(point_indices[e]);
    }

    (edge_points, edges_to_use)
}

// Return the edges that contain triangle vertices
pub fn edges_from_lookup(edges: &String) -> Vec<usize> {
    let use_edge = "1".chars().next().unwrap(); // edgeTable[8] = 100000001100 -> Edges 2, 3, 11 intersected
    let mut i = (edges.len() - 1) as i32;
    let mut edges_to_use = Vec::new();

    for char in edges.chars() {
        if char == use_edge {
            edges_to_use.push(i as usize)
        }
        i -= 1;
    }

    edges_to_use
}

// ===========================================================
// ====================== Interpolation ======================
// ===========================================================

// linearly map a number from one range to another
pub fn remap(s: f64, range_in: [f64; 2], range_out: [f64; 2]) -> f64 {
    range_out[0] + (s - range_in[0]) * (range_out[1] - range_out[0]) / (range_in[1] - range_in[0])
}

// Return the interpolation factor t corresponding to iso_val
pub fn find_t(v0: f64, v1: f64, iso_val: f64) -> f64 {
    (iso_val - v0) / (v1 - v0)
}

// Linear interpolation
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

// Linearly interpolate between two points by factor t
pub fn interpolate_points(p0: Point, p1: Point, t: f64) -> Vec<f64> {
    // may need to make this an array not a vector
    let pf: Vec<f64> = p0
        .iter()
        .zip(p1.iter())
        .map(|p| lerp(*p.0, *p.1, t))
        .collect();
    pf
}

// ==========================================================
// ========================= Meshes =========================
// ==========================================================

#[derive(Clone)]
pub struct Mesh {
    // vertices : [[x1, y1, z1], [x2, y2, z2]...]
    pub vertices: Vec<Point>,

    // tris : [[v0, v1, v2], [v3, v4, v5]]
    pub tris: Vec<[usize; 3]>, // new triangles
}

impl Mesh {
    //create a new empty Mesh
    pub fn new_empty() -> Self {
        Self {
            vertices: Vec::new(),
            tris: Vec::new(),
        }
    }

    //create a triangle from Point indices
    pub fn triangle_from_verts(&mut self, x: usize, y: usize, z: usize) {
        // x/y/z are indices
        // create a triangle from existing vertices (assumes already has vertices)
        let tri = [x, y, z];

        self.tris.push(tri)
    }

    //return triangle Point coordinates
    pub fn tri_coords(&self, tri: usize) -> Vec<Point> {
        let va = self.vertices[self.tris[tri][0]];
        let vb = self.vertices[self.tris[tri][1]];
        let vc = self.vertices[self.tris[tri][2]];

        vec![va, vb, vc]
    }

    //return triangle normal
    pub fn tri_normal(&self, tri: usize) -> Vector3<f64> {
        //tri = starting Point index

        let va = self.vertices[self.tris[tri][0]];
        let vb = self.vertices[self.tris[tri][1]];
        let vc = self.vertices[self.tris[tri][2]];

        let _a = Vector3::new(va[0], va[1], va[2]);
        let _b = Vector3::new(vb[0], vb[1], vb[2]);
        let _c = Vector3::new(vc[0], vc[1], vc[2]);

        let v_a_b = _b - _a;
        let v_b_c = _c - _b;

        let cross = v_a_b.cross(&v_b_c);

        cross / cross.norm() //normal vector
    }
}

//writing mesh to STL:
pub fn export_stl(path: &str, mesh: Mesh) {
    //based on: https://en.wikipedia.org/wiki/STL_(file_format)

    let mut writer = vec![];
    let mut normal;
    //Writing STL header UINT8[80] – Header - 80 bytes
    let header = [0u8; 80];
    writer.write_all(&header).expect("Error");

    //Writing tri count
    let tri_count = mesh.tris.len();
    writer
        .write_u32::<LittleEndian>(tri_count as u32)
        .expect("Error");

    //FOR EACH TRIANGLE
    let mut tri = 0;
    // let tri_vert_count = mesh.triangles.len();
    let tri_count = mesh.tris.len();
    while tri < tri_count {
        //write triangle normal
        normal = mesh.tri_normal(tri); //calculate normal
        writer
            .write_f32::<LittleEndian>((normal.x) as f32)
            .expect("Error"); // write normal values
        writer
            .write_f32::<LittleEndian>((normal.y) as f32)
            .expect("Error");
        writer
            .write_f32::<LittleEndian>((normal.z) as f32)
            .expect("Error");

        //write each Point
        let vertices = mesh.tri_coords(tri);
        for point in vertices {
            // write Point coordinates
            writer
                .write_f32::<LittleEndian>(point[0] as f32)
                .expect("Error");
            writer
                .write_f32::<LittleEndian>(point[1] as f32)
                .expect("Error");
            writer
                .write_f32::<LittleEndian>(point[2] as f32)
                .expect("Error");
        }
        //write attribute byte count
        writer.write_u16::<LittleEndian>(0).expect("Error");
        tri += 1;
    }
    //write final stl
    fs::write(path, writer).expect("Something went wrong.");

    println!("Vertices: {:?}", mesh.vertices.len());
    println!("Triangles: {:?}\n", tri_count);
}

// =========================================================
// ========================= TABLES ========================
// =========================================================

// =============== EDGE TABLE ===============
const EDGE_TABLE: [i32; 256] = [
    0x0, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c, 0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03,
    0xe09, 0xf00, 0x190, 0x99, 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c, 0x99c, 0x895, 0xb9f,
    0xa96, 0xd9a, 0xc93, 0xf99, 0xe90, 0x230, 0x339, 0x33, 0x13a, 0x636, 0x73f, 0x435, 0x53c,
    0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30, 0x3a0, 0x2a9, 0x1a3, 0xaa, 0x7a6,
    0x6af, 0x5a5, 0x4ac, 0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0, 0x460, 0x569,
    0x663, 0x76a, 0x66, 0x16f, 0x265, 0x36c, 0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69,
    0xb60, 0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff, 0x3f5, 0x2fc, 0xdfc, 0xcf5, 0xfff, 0xef6,
    0x9fa, 0x8f3, 0xbf9, 0xaf0, 0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55, 0x15c, 0xe5c,
    0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950, 0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf,
    0x1c5, 0xcc, 0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0, 0x8c0, 0x9c9, 0xac3,
    0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc, 0xcc, 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
    0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c, 0x15c, 0x55, 0x35f, 0x256, 0x55a,
    0x453, 0x759, 0x650, 0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc, 0x2fc, 0x3f5,
    0xff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0, 0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65,
    0xc6c, 0x36c, 0x265, 0x16f, 0x66, 0x76a, 0x663, 0x569, 0x460, 0xca0, 0xda9, 0xea3, 0xfaa,
    0x8a6, 0x9af, 0xaa5, 0xbac, 0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa, 0x1a3, 0x2a9, 0x3a0, 0xd30,
    0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c, 0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33,
    0x339, 0x230, 0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c, 0x69c, 0x795, 0x49f,
    0x596, 0x29a, 0x393, 0x99, 0x190, 0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
    0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0,
];

const CORNER_POINT_INDICES: [[i8; 2]; 12] = [
    [0, 1],
    [1, 2],
    [2, 3],
    [3, 0],
    [4, 5],
    [5, 6],
    [6, 7],
    [7, 4],
    [0, 4],
    [1, 5],
    [2, 6],
    [3, 7],
];

// =============== TRI TABLE ===============

const TRI_TABLE: [[i8; 16]; 256] = [
    [
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    ],
    [0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1],
    [3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1],
    [3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1],
    [3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1],
    [9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1],
    [1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1],
    [9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1],
    [2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1],
    [8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1],
    [9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1],
    [4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1],
    [3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1],
    [1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1],
    [4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1],
    [4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1],
    [9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1],
    [1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1],
    [5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1],
    [2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1],
    [9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1],
    [0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1],
    [2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1],
    [10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1],
    [4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1],
    [5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1],
    [5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1],
    [9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1],
    [0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1],
    [1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1],
    [10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1],
    [8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1],
    [2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1],
    [7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1],
    [9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1],
    [2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1],
    [11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1],
    [9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1],
    [5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1],
    [11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1],
    [11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1],
    [1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1],
    [9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1],
    [5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1],
    [2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1],
    [0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1],
    [5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1],
    [6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1],
    [0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1],
    [3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1],
    [6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1],
    [5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1],
    [1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1],
    [10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1],
    [6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1],
    [1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1],
    [8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1],
    [7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1],
    [3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1],
    [5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1],
    [0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1],
    [9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1],
    [8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1],
    [5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1],
    [0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1],
    [6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1],
    [10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1],
    [10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1],
    [8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1],
    [1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1],
    [3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1],
    [0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1],
    [10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1],
    [0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1],
    [3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1],
    [6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1],
    [9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1],
    [8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1],
    [3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1],
    [6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1],
    [0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1],
    [10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1],
    [10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1],
    [1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1],
    [2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1],
    [7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1],
    [7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1],
    [2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1],
    [1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1],
    [11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1],
    [8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1],
    [0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1],
    [7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1],
    [10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1],
    [2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1],
    [6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1],
    [7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1],
    [2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1],
    [1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1],
    [10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1],
    [10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1],
    [0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1],
    [7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1],
    [6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1],
    [8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1],
    [9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1],
    [6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1],
    [1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1],
    [4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1],
    [10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1],
    [8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1],
    [0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1],
    [1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1],
    [8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1],
    [10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1],
    [4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1],
    [10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1],
    [5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1],
    [11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1],
    [9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1],
    [6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1],
    [7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1],
    [3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1],
    [7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1],
    [9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1],
    [3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1],
    [6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1],
    [9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1],
    [1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1],
    [4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1],
    [7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1],
    [6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1],
    [3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1],
    [0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1],
    [6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1],
    [1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1],
    [0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1],
    [11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1],
    [6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1],
    [5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1],
    [9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1],
    [1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1],
    [1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1],
    [10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1],
    [0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1],
    [5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1],
    [10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1],
    [11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1],
    [0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1],
    [9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1],
    [7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1],
    [2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1],
    [8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1],
    [9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1],
    [9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1],
    [1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1],
    [9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1],
    [9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1],
    [5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1],
    [0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1],
    [10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1],
    [2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1],
    [0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1],
    [0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1],
    [9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1],
    [5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1],
    [3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1],
    [5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1],
    [8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1],
    [0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1],
    [9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1],
    [0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1],
    [1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1],
    [3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1],
    [4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1],
    [9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1],
    [11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1],
    [11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1],
    [2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1],
    [9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1],
    [3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1],
    [1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1],
    [4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1],
    [4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1],
    [0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1],
    [3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1],
    [3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1],
    [0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1],
    [9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1],
    [1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    ],
];
