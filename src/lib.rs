use byteorder::{LittleEndian, WriteBytesExt};
use evalexpr::{eval_with_context, ContextWithMutableVariables, HashMapContext, Value};
use nalgebra::{distance, point, vector, Point3, Vector3};
use rayon::prelude::*;
use std::{collections::HashMap, fs, io::Write, sync::Mutex};

// ==========================================================
// ======================= Data types =======================
// ==========================================================

pub type Point = Point3<f64>;
pub type Vector = Vector3<f64>;
pub const ORIGIN: Point3<f64> = Point3::new(0.0, 0.0, 0.0);
pub type CompiledFunction = dyn Fn(Point) -> f64 + Sync;
pub type SymbolicExpression<'a> = &'a str;

pub fn import_expression(filepath: &str) -> Result<String, Box<dyn std::error::Error>> {
    let data = fs::read_to_string(filepath)?;
    Ok(data)
}

fn init_context() -> HashMapContext {
    let mut context = HashMapContext::new();
    context
        .set_value(String::from('x'), Value::from(0.))
        .unwrap();
    context
        .set_value(String::from('y'), Value::from(0.))
        .unwrap();
    context
        .set_value(String::from('z'), Value::from(0.))
        .unwrap();

    context
}

fn update_context(context: &mut HashMapContext, point: Point) -> &HashMapContext {
    context
        .set_value(String::from('x'), Value::from(point.x))
        .unwrap();
    context
        .set_value(String::from('y'), Value::from(point.y))
        .unwrap();
    context
        .set_value(String::from('z'), Value::from(point.z))
        .unwrap();

    context
}

pub struct Domain {
    pub x: usize,
    pub y: usize,
    pub z: usize,
    pub scale: f64,
    pub min_point: Point,
}

impl Domain {
    pub fn new(span: [usize; 3], scale: f64) -> Self {
        let min_point = point![
            -1. * (span[0] as f64) * scale / 2.,
            -1. * (span[1] as f64) * scale / 2.,
            -1. * (span[2] as f64) * scale / 2.
        ];

        Self {
            x: span[0],
            y: span[1],
            z: span[2],
            scale: scale,
            min_point: min_point,
        }
    }
}

// Marching cubes algorithm (discrete data version)
pub fn marching_cubes_buffer(buffer: &Buffer3D, threshold: f64) -> Mesh {
    let mut target_mesh = Mesh::new_empty();

    let edge_table = &EDGE_TABLE.map(|e| format!("{:b}", e));

    let vertices = (0..buffer.size_x-1)
        .into_par_iter()
        .map(|x| {
            (0..buffer.size_y-1)
                .map(|y| {
                    (0..buffer.size_z-1)
                        .map(|z| {
                            // corner positions
                            // There's some redundancy/overlap that could be optimized
                            let corner_indices = buffer.voxel_corner_indices(x, y, z);

                            let corner_values = corner_indices
                                .iter()
                                .map(|p| buffer.get(p[0], p[1], p[2]))
                                .collect::<Vec<f64>>();

                            let corner_positions = corner_indices
                                .iter()
                                .map(|p| {
                                    add_points(
                                        buffer.min_point,
                                        point![p[0] as f64, p[1] as f64, p[2] as f64]
                                            * buffer.scale,
                                    )
                                })
                                .collect::<Vec<Point>>();

                            // Calculating state
                            let state =
                                get_state(&corner_values, threshold).expect("Could not get state");

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
                                corner_values,
                                threshold,
                            );

                            // adding triangle verts
                            let new_verts = triangle_verts_from_state(edge_points, state);
                            new_verts
                        })
                        .flatten()
                        .collect::<Vec<Point>>()
                })
                .flatten()
                .collect::<Vec<Point>>()
        })
        .flatten()
        .collect::<Vec<Point>>();

    // Adding vertices to the mesh
    target_mesh.set_vertices(vertices);

    // Creating triangles from the vertices
    target_mesh.create_triangles();

    println!(
        "\nCube count: {}",
        buffer.size_x * buffer.size_y * buffer.size_z
    );
    return target_mesh;
}
// Marching cubes algorithm (evaluated version)
pub fn marching_cubes_evaluated(
    expression: SymbolicExpression,
    min_point: Point,
    x_count: usize,
    y_count: usize,
    z_count: usize,
    threshold: f64,
    scale: f64,
) -> Mesh {
    let mut target_mesh = Mesh::new_empty();

    let edge_table = &EDGE_TABLE.map(|e| format!("{:b}", e));

    let vertices = (0..x_count)
        .into_par_iter()
        .map(|x| {
            let mut context = init_context(); // Avoid thread confusion by creating here
            (0..y_count)
                .map(|y| {
                    (0..z_count)
                        .map(|z| {
                            // corner positions
                            // There's some redundancy/overlap that could be optimized
                            let corner_positions = get_corner_positions(min_point, x, y, z, scale);

                            // voxel values (evaluated sdf)
                            let eval_corners = corner_positions
                                .iter()
                                .map(|p| {
                                    update_context(&mut context, *p);
                                    eval_with_context(&expression, &context)
                                        .unwrap()
                                        .as_float()
                                        .unwrap()
                                })
                                .collect();

                            // Calculating state
                            let state =
                                get_state(&eval_corners, threshold).expect("Could not get state");

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

                            // adding triangle verts
                            let new_verts = triangle_verts_from_state(edge_points, state);
                            new_verts
                        })
                        .flatten()
                        .collect::<Vec<Point>>()
                })
                .flatten()
                .collect::<Vec<Point>>()
        })
        .flatten()
        .collect::<Vec<Point>>();

    // Adding vertices to the mesh
    target_mesh.set_vertices(vertices);

    // Creating triangles from the vertices
    target_mesh.create_triangles();

    println!("\nCube count: {}", x_count * y_count * z_count);
    return target_mesh;
}

// Marching cubes algorithm (compiled version)
pub fn marching_cubes_compiled(
    function: &Mutex<CompiledFunction>,
    min_point: Point,
    x_count: usize,
    y_count: usize,
    z_count: usize,
    threshold: f64,
    scale: f64,
) -> Mesh {
    let mut target_mesh = Mesh::new_empty();
    let function = function.lock().unwrap();

    let edge_table = &EDGE_TABLE.map(|e| format!("{:b}", e));

    let vertices = (0..x_count)
        .into_par_iter()
        .map(|x| {
            (0..y_count)
                .map(|y| {
                    (0..z_count)
                        .map(|z| {
                            // corner positions
                            // There's some redundancy/overlap that could be optimized
                            let corner_positions = get_corner_positions(min_point, x, y, z, scale);

                            // voxel values (evaluated sdf)
                            let eval_corners =
                                corner_positions.iter().map(|p| function(*p)).collect();

                            // Calculating state
                            let state =
                                get_state(&eval_corners, threshold).expect("Could not get state");

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

                            // adding triangle verts
                            let new_verts = triangle_verts_from_state(edge_points, state);
                            new_verts
                        })
                        .flatten()
                        .collect::<Vec<Point>>()
                })
                .flatten()
                .collect::<Vec<Point>>()
        })
        .flatten()
        .collect::<Vec<Point>>();

    // Adding vertices to the mesh
    target_mesh.set_vertices(vertices);

    // Creating triangles from the vertices
    target_mesh.create_triangles();

    println!("\nCube count: {}", x_count * y_count * z_count);
    return target_mesh;
}

// ===========================================================
// ============== Marching cubes helper functions ============
// ===========================================================

fn triangle_verts_from_state(edge_points: HashMap<usize, Vec<f64>>, state: usize) -> Vec<Point> {
    // triangles (TRI_TABLE[state])
    // Example: [7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
    // Triangles: [p7, p3, p2], [p6, p7, p2]
    let new_verts = TRI_TABLE[state]
        .iter()
        .filter(|v| v != &&-1)
        .map(|t| {
            let new_vert = Point3::new(
                //converting Vec to array
                edge_points[&(*t as usize)][2],
                edge_points[&(*t as usize)][1],
                edge_points[&(*t as usize)][0],
            );
            // target_mesh.vertices.push(new_vert);
            new_vert
        })
        .collect::<Vec<Point>>();
    new_verts
}

// Get the point coordinates at the 8 vertices of the cube
fn get_corner_positions(min_point: Point, x: usize, y: usize, z: usize, scale: f64) -> Vec<Point> {
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
pub fn get_state(eval_corners: &Vec<f64>, threshold: f64) -> Result<usize, MarchingCubesError> {
    // Make sure eval_corners contains exactly 8 values
    if eval_corners.len() != 8 {
        return Err(MarchingCubesError);
    }

    // 0 if <= threshold, 1 if > threshold
    let states = eval_corners.iter().map(|x| state_function(*x, threshold));

    let mut i = 1.0;
    let mut final_state = 0.0;
    for s in states {
        final_state += s * i;
        i *= 2.0;
    }

    return Ok(final_state as usize);
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
// ======================= Voxel Grid =======================
// ==========================================================

pub struct Buffer3D {
    pub size_x: usize,
    pub size_y: usize,
    pub size_z: usize,
    pub min_point: Point,
    pub scale: f64,
    pub values: Vec<Vec<Vec<f64>>>,
}

impl Buffer3D {
    pub fn new(size_x: usize, size_y: usize, size_z: usize) -> Self {
        let scale = 1.;
        let min_point = point![size_x as f64 / 2., size_y as f64 / 2., size_z as f64 / 2.];
        let values = vec![vec![vec![0.; size_x]; size_y]; size_z];
        Self {
            size_x,
            size_y,
            size_z,
            min_point,
            scale,
            values,
        }
    }
    pub fn get(&self, x: usize, y: usize, z: usize) -> f64 {
        self.values[z][y][x]
    }
    pub fn set(&mut self, x: usize, y: usize, z: usize, v: f64) {
        self.values[z][y][x] = v
    }
    pub fn voxel_corner_indices(&self, x: usize, y: usize, z: usize) -> [[usize; 3]; 8] {
        // could be consolidated/more idiomatic
        let c0 = [x, y, z];
        let c1 = [x + 1, y, z];
        let c2 = [x + 1, y + 1, z];
        let c3 = [x, y + 1, z];
        let c4 = [x, y, z + 1];
        let c5 = [x + 1, y, z + 1];
        let c6 = [x + 1, y + 1, z + 1];
        let c7 = [x, y + 1, z + 1];
        return [c0, c1, c2, c3, c4, c5, c6, c7];
    }
    pub fn fill(&mut self, function: &CompiledFunction) {
        (0..self.size_x).for_each(|x| {
            (0..self.size_y).for_each(|y| {
                (0..self.size_z).for_each(|z| {
                    let p = add_points(
                        point![x as f64, y as f64, z as f64] * self.scale,
                        self.min_point,
                    );
                    self.set(x as usize, y as usize, z as usize, function(p))
                })
            })
        });
    }
}

// ===========================================================
// =========== Some Distance Equations to Test With ==========
// ===========================================================
// mostly thanks to https://iquilezles.org/

pub struct SDF {}

impl SDF {
    pub fn new() {}

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

#[derive(Debug)]
pub struct MarchingCubesError;

impl Mesh {
    //create a new empty Mesh
    pub fn new_empty() -> Self {
        Self {
            vertices: Vec::new(),
            tris: Vec::new(),
        }
    }

    //create a triangle from Point indices
    pub fn triangle_from_verts(
        &mut self,
        x: usize,
        y: usize,
        z: usize,
    ) -> Result<(), MarchingCubesError> {
        // Need to make sure mesh isn't empty
        if self.vertices.len() <= x.max(y.max(z)) {
            return Err(MarchingCubesError);
        }

        // x/y/z are indices that form a triangle
        self.tris.push([x, y, z]);

        Ok(())
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

    pub fn create_triangles(&mut self) -> () {
        let mut v = 0;
        while v < self.vertices.len() {
            self.triangle_from_verts(v, v + 1, v + 2)
                .expect("Could not create triangle.");
            v += 3
        }
    }

    pub fn set_vertices(&mut self, vertices: Vec<Point>) -> () {
        self.vertices = vertices
    }

    //writing mesh to STL:
    pub fn export_stl(&self, path: &str) {
        //based on: https://en.wikipedia.org/wiki/STL_(file_format)

        let mut writer = vec![];
        let mut normal;
        //Writing STL header UINT8[80] – Header - 80 bytes
        let header = [0u8; 80];
        writer.write_all(&header).expect("Error");

        //Writing tri count
        let tri_count = self.tris.len();
        writer
            .write_u32::<LittleEndian>(tri_count as u32)
            .expect("Error");

        //FOR EACH TRIANGLE
        let mut tri = 0;
        // let tri_vert_count = mesh.triangles.len();
        let tri_count = self.tris.len();
        while tri < tri_count {
            //write triangle normal
            normal = self.tri_normal(tri); //calculate normal
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
            let vertices = self.tri_coords(tri);
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

        println!("Vertices: {:?}", self.vertices.len());
        println!("Triangles: {:?}\n", tri_count);
    }
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
