use lidar_format_convert::{convert_path, ConvertOptions};

fn main() -> Result<(), lidar_format_convert::Error> {
    let options = ConvertOptions {
        allow_lossy: false,
        ..Default::default()
    };
    let report = convert_path("input.xyz", "output.ply", &options)?;
    println!("converted {} points", report.points_written);
    Ok(())
}
