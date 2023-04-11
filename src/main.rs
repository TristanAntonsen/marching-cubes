fn main() {

    for n in 0..10 {
        println!("{}", fun_test(n as f64, &square))
    }

    for n in 0..10 {
        println!("{}", fun_test(n as f64, &halve))
    }

}

fn fun_test(n : f64, f: &dyn Fn(f64) -> f64) -> f64 {
    f(n)
}

fn square(n : f64) -> f64 {
    n * n
}

fn halve(n : f64) -> f64 {
    n / 2.0
}