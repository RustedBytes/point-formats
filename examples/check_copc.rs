use copc_rs::{BoundsSelection, CopcReader, LodSelection};
use std::env;
use std::fs::File;
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: cargo run --example check_copc <file.copc.laz>");
        std::process::exit(1);
    }

    let path_str = &args[1];
    let path = Path::new(path_str);
    if !path.exists() {
        eprintln!("Error: File not found at '{}'", path_str);
        std::process::exit(1);
    }

    let file = File::open(path)?;
    let mut reader = CopcReader::new(file)?;
    println!("Successfully parsed COPC file: {}", path_str);
    println!("------------------------------------------------------------");

    // LAS Header
    let header = reader.header();
    println!("LAS Header (COPC Base):");
    println!(
        "  LAS Version:          {}.{}",
        header.version().major,
        header.version().minor
    );
    println!(
        "  System Identifier:    {}",
        header.system_identifier().trim()
    );
    println!(
        "  Generating Software:  {}",
        header.generating_software().trim()
    );
    println!("  Point Format ID:      {}", header.point_format().to_u8()?);
    println!("  Declared Point Count: {}", header.number_of_points());

    let bounds = header.bounds();
    println!("  COPC Extents/Bounds:");
    println!("    X: [{:.4}, {:.4}]", bounds.min.x, bounds.max.x);
    println!("    Y: [{:.4}, {:.4}]", bounds.min.y, bounds.max.y);
    println!("    Z: [{:.4}, {:.4}]", bounds.min.z, bounds.max.z);

    // COPC Info
    let info = reader.copc_info();
    println!("\nCOPC Info details:");
    println!("  {:?}", info);

    // Read and print a sample of the first 5 points
    let points_iter = reader.points(LodSelection::All, BoundsSelection::All)?;
    let mut sample_points = Vec::new();
    for p in points_iter {
        sample_points.push(p);
        if sample_points.len() >= 5 {
            break;
        }
    }

    if !sample_points.is_empty() {
        println!("\nSample Points from COPC (first {}):", sample_points.len());
        println!(
            "    {:^5} | {:^10} | {:^10} | {:^10} | {:^9} | {:^15}",
            "Index", "X", "Y", "Z", "Intensity", "Color (R,G,B)"
        );
        println!("    ------|------------|------------|------------|-----------|----------------");
        for (idx, p) in sample_points.iter().enumerate() {
            let color_str = p.color.as_ref().map_or("N/A".to_string(), |c| {
                format!("({}, {}, {})", c.red, c.green, c.blue)
            });
            println!(
                "    {:^5} | {:>10.4} | {:>10.4} | {:>10.4} | {:>9} | {:^15}",
                idx, p.x, p.y, p.z, p.intensity, color_str
            );
        }
    }

    Ok(())
}
