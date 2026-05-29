use e57::{CartesianCoordinate, E57Reader};
use std::env;
use std::fs::File;
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: cargo run --example check_e57 <file.e57>");
        std::process::exit(1);
    }

    let path_str = &args[1];
    let path = Path::new(path_str);
    if !path.exists() {
        eprintln!("Error: File not found at '{}'", path_str);
        std::process::exit(1);
    }

    let file = File::open(path)?;
    let mut reader = E57Reader::new(file)?;
    println!("Successfully parsed E57 file: {}", path_str);

    if let Some(crs) = reader.coordinate_metadata() {
        println!("Coordinate Reference System (CRS):");
        println!("  {}", crs.trim());
    } else {
        println!("Coordinate Reference System (CRS): None specified");
    }

    let pcs = reader.pointclouds();
    println!("\nContains {} point cloud(s):", pcs.len());
    for (idx, pc) in pcs.iter().enumerate() {
        println!("------------------------------------------------------------");
        println!("Scan #{}:", idx);
        println!("  GUID:        {}", pc.guid.as_deref().unwrap_or("None"));
        if let Some(name) = &pc.name {
            println!("  Name:        {}", name);
        }
        if let Some(desc) = &pc.description {
            println!("  Description: {}", desc);
        }
        println!("  Points:      {}", pc.records);

        if let Some(bounds) = &pc.cartesian_bounds {
            println!("  Cartesian Bounds:");
            let format_bound =
                |val: Option<f64>| val.map_or("N/A".to_string(), |v| format!("{:.4}", v));
            println!(
                "    X: [{}, {}]",
                format_bound(bounds.x_min),
                format_bound(bounds.x_max)
            );
            println!(
                "    Y: [{}, {}]",
                format_bound(bounds.y_min),
                format_bound(bounds.y_max)
            );
            println!(
                "    Z: [{}, {}]",
                format_bound(bounds.z_min),
                format_bound(bounds.z_max)
            );
        }

        if let Some(transform) = &pc.transform {
            println!("  Transform:");
            println!(
                "    Translation: [x: {:.4}, y: {:.4}, z: {:.4}]",
                transform.translation.x, transform.translation.y, transform.translation.z
            );
            println!(
                "    Rotation (Quat): [w: {:.4}, x: {:.4}, y: {:.4}, z: {:.4}]",
                transform.rotation.w,
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z
            );
        }

        // Read and print a sample of the first 5 points
        let limit = 5.min(pc.records as usize);
        if limit > 0 {
            println!("  Sample Points (first {}):", limit);
            let mut pc_reader = reader.pointcloud_simple(pc)?;
            pc_reader.normalize_intensity(false);

            println!(
                "    {:^5} | {:^10} | {:^10} | {:^10} | {:^9} | {:^15}",
                "Index", "X", "Y", "Z", "Intensity", "Color (R,G,B)"
            );
            println!(
                "    ------|------------|------------|------------|-----------|----------------"
            );
            for (p_idx, p_res) in pc_reader.take(limit).enumerate() {
                let p = p_res?;
                let (x, y, z) = match p.cartesian {
                    CartesianCoordinate::Valid { x, y, z } => (x, y, z),
                    CartesianCoordinate::Direction { x, y, z } => (x, y, z),
                    CartesianCoordinate::Invalid => (0.0, 0.0, 0.0),
                };
                let intensity_str = p
                    .intensity
                    .map_or("N/A".to_string(), |i| format!("{:.4}", i));
                let color_str = p.color.map_or("N/A".to_string(), |c| {
                    format!("({:.2}, {:.2}, {:.2})", c.red, c.green, c.blue)
                });
                println!(
                    "    {:^5} | {:>10.4} | {:>10.4} | {:>10.4} | {:>9} | {:^15}",
                    p_idx, x, y, z, intensity_str, color_str
                );
            }
        }
    }

    Ok(())
}
