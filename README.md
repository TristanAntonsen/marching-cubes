# A simple marching cubes implementation
||||
|:-:|:-:|:-:|
<img src="https://github.com/TristanAntonsen/marching-cubes/blob/main/img/coarse_sphere.png">|<img src="https://github.com/TristanAntonsen/marching-cubes/blob/main/img/sinc.png">|<img src="https://github.com/TristanAntonsen/marching-cubes/blob/main/img/ripple_sphere_cube.png">
$x^2+y^2+z^2-2500=0$|Plane (z) modulated with a sinc function|(SDFs) A sphere smoothly united with a cube and offset by a sinusoidal function based off of the cube
10x10x10 grid|200x200x200 grid|200x200x200 grid
### Usage:

The main program uses a simple CLI to extract the 0 isosurface from a symbolic expression.

```zsh
>cargo build --release
>./target/release/marching_cubes --expr="x^2+y^2+z^2-2500"

Cube count: 1000000
Vertices: 282336
Triangles: 94112

Exported: marched.stl
Time: 0 min 3.10 seconds
```
|Argument|Short|Long|Default|
|:--|:--:|:--:|:--:|
|Expression|-e|--expr||
|.stl path|-e|--export-path|examples/marched.stl|
|Grid scale|-s|--scale|1.|
|Domain (centered)|-d|--domain|"[100, 100, 100]"|

### Variations
The crate provides four versions of marching cubes. Each one of these has an example file that can be run with:

```zsh
cargo run --release --example <example>
```

<u>1. Symbolic Expression (evalexpr)</u>:

Evaluate a symbolic expression at each grid point using the evalexpr crate. Multithreaded with [Rayon](https://crates.io/crates/rayon).

x, y, and z will be updated at each sample point. Operators in the [evalexpr](https://crates.io/crates/evalexpr) crate are supported.

```rust
let expr = &"(.25*x)^2+(.25*y)^2-z-10";

let mesh = marching_cubes_evaluated(
    &expr,                    // expression to evaluate
    point![-25., -25., -25.], // minimum bounding box point
    100,                      // x count
    100,                      // y count
    100,                      // z count
    0.,                       // isosurface value
    1.,                       // scale
);
```
<u>2. Precompiled</u>:
Evaluate a precompiled function, `map()`. Very fast for obvious reasons (also multithreaded with [Rayon](https://crates.io/crates/rayon)).

`map()` could be anything that returns a value, but some signed distance functions are provided as samples. The code will extract the isosurface where `map(Point)` = 0 by default).

```rust
let mesh = marching_cubes_compiled(
    &thread_safe_map,            // function to evaluate
    point![-100., -100., -100.], // minimum bounding box point
    200,                         // x count
    200,                         // y count
    200,                         // z count
    0.,                          // isosurface value
    1.,                          // scale
);
```
This sample function will make a cube smoothly united with a sphere:
```rust
fn map(p: Point) -> f64 {
    let s = sdf::sphere(p, point![30., 30., 30.], 65.0);
    let b = sdf::rounded_box(p, point![-30., -30., -30.], vector![60., 60., 60.], 10.);
    sdf::boolean_union(b, s, 20.)
}

```

<u>3. Discrete Data</u>:

A 3D buffer of discrete data is sampled for the algorithm. In the example, a `map(Point)` function is used to populate the voxels, but any discrete data could be used.

```rust
let mesh = marching_cubes_buffer(
    &buffer,    // discrete 3D data
    0.          // isosurface value
);
```
<u>4. Symbolic Expression (fidget)</u>:

Evaluate a symbolic expression at each grid point using Matt Keeter's [Fidget](https://github.com/mkeeter/fidget). Multithreaded with [Rayon](https://crates.io/crates/rayon).

x, y, and z will be updated at each sample point. Operators in the [rhai](https://crates.io/crates/rhai) crate are supported.

```rust
let expr = "x*x + y*y + z*z - 2500";

let mesh = marching_cubes_fidget(
    &expr,                    // expression to evaluate
    point![-100., -100., -100.], // minimum bounding box point
    200,                      // x count
    200,                      // y count
    200,                      // z count
    0.,                       // isosurface value
    1.,                       // scale
);
```
---
### Why?
Volumetric information (e.g. implicit surfaces) -> Surface of triangles

### How?:
1. The algorithm iterates through a uniform 3D grid, sampling 8 points (cube vertices) of $f{(x,y,z)}$ at once
2. There are 256 possible binary (in or out of the isosurface) states of the 8 vertices. Clever lookup tables indexed by from 8 bit lookup indices return vertex configurations to form the corresponding triangles.

Much better explanations can be found here:

- [Original paper](https://dl.acm.org/doi/pdf/10.1145/37402.37422)
- [Helpful write-up](https://paulbourke.net/geometry/polygonise/)

---
### Future improvements
- Optimize to reduce obviously redundant queries (overlapping corners)
- Refactor for less redundancy
- Multithread the discrete data version
- More idiomatic Rust