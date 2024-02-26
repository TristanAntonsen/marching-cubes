use clap::Parser;
use marching_cubes::Domain;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
pub struct Args {
    #[arg(short, long)]
    pub expr: String,

    #[arg(short, long, default_value = "examples/marched.stl")]
    pub export_path: String,

    #[arg(short, long, default_value = "1.")]
    pub scale: String,

    #[arg(short, long, default_value = "[100, 100, 100]")]
    pub domain: String,
}

impl Args {
    pub fn construct_domain(&self) -> Domain {
        let stripped = self.domain.replace(&['[', ']', ' '][..], "");
        let parsed = stripped.split(",").collect::<Vec<&str>>();
        let scale_float: f64 = self.scale.parse().unwrap();
        Domain::new([
            parsed[0].parse().unwrap(),
            parsed[1].parse().unwrap(),
            parsed[2].parse().unwrap(),
        ],
        scale_float
    )
    }
}
