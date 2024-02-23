# A simple marching cubes implementation
||||
|:-:|:-:|:-:|
<img src="https://github.com/TristanAntonsen/marching-cubes/blob/main/img/coarse_sphere.png">|<img src="https://github.com/TristanAntonsen/marching-cubes/blob/main/img/sinc.png">|<img src="https://github.com/TristanAntonsen/marching-cubes/blob/main/img/ripple_sphere_cube.png">
$\sqrt{x^2+y^2+z^2}-r=0$|$z - Asinc(f\sqrt{x^2+y^2}) = 0$|(SDFs) A sphere smoothly united with a cube and offset by a sinusoidal function based off of the cube
10x10x10 grid|200x200x200 grid|200x200x200 grid
### Usage:
The map() function will be evaluated at each point in the grid. This could be anything that returns a value, but some signed distance functions are provided as samples. The code will extract the isosurface $f{(x,y,z)=0}$ by default).

This function will make a cube smoothly united with a sphere:
```rust
fn map(p: Point) -> f64 {
    let s = sdf::sphere(p, point![30., 30., 30.], 65.0);
    let b = sdf::rounded_box(p, point![-30., -30., -30.], vector![60., 60., 60.], 10.);
    sdf::boolean_union(b, s, 20.)
}
```

This function will plot the surface where $z - Asinc(f\sqrt{x^2+y^2}) = 0$.
```rust
fn map(p: Point) -> f64 {
    let f = 0.375; // frequency
    let a = 20.;   // amplitude
    return p.z - a * (f * (p.x.powf(2.) + p.y.powf(2.)).sqrt()).sinc()
}
```

To run the program:
```zsh
>cargo run --release

Cube count: 7880599
Vertices: 943872
Triangles: 314624

Exported: marched.stl
Time: 0 min 1.48 seconds
```
---
### Why?
Volumetric information -> Surface of triangles

#### How?:
1. The algorithm iterates through a uniform 3D grid, sampling 8 points (cube vertices) of $f{(x,y,z)}$ at once
2. There are 256 possible binary (in or out of the isosurface) states of the 8 vertices. Clever lookup tables indexed by from 8 bit lookup indices return vertex configurations to form the corresponding triangles.

Much better explanations can be found here:

- [Original paper](https://dl.acm.org/doi/pdf/10.1145/37402.37422)
- [Helpful write-up](https://paulbourke.net/geometry/polygonise/)

---
### Future improvements
- Multiprocessing with [Rayon](https://crates.io/crates/rayon)
- Optimize to reduce redundant queries
- Expression evaluator with something like [evalexpr](https://crates.io/crates/evalexpr)
- More idiomatic Rust