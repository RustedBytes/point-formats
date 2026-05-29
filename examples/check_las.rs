use std::env;
use std::path::Path;
use std::collections::BTreeMap;
use las::Reader;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: cargo run --example check_las <file.las>");
        std::process::exit(1);
    }
    
    let path_str = &args[1];
    let path = Path::new(path_str);
    if !path.exists() {
        eprintln!("Error: File not found at '{}'", path_str);
        std::process::exit(1);
    }

    let mut reader = Reader::from_path(path)?;
    let header = reader.header();
    
    println!("Successfully parsed LAS file: {}", path_str);
    println!("------------------------------------------------------------");
    println!("LAS Header Information:");
    println!("  Version:              {}.{}", header.version().major, header.version().minor);
    println!("  System Identifier:    {}", header.system_identifier().trim());
    println!("  Generating Software:  {}", header.generating_software().trim());
    if let Some(date) = header.date() {
        println!("  File Creation Date:   {}", date);
    } else {
        println!("  File Creation Date:   N/A");
    }
    println!("  Point Format ID:      {}", header.point_format().to_u8()?);
    println!("  Declared Point Count: {}", header.number_of_points());
    
    let bounds = header.bounds();
    println!("  Declared Extents/Bounds:");
    println!("    X: [{:.4}, {:.4}]", bounds.min.x, bounds.max.x);
    println!("    Y: [{:.4}, {:.4}]", bounds.min.y, bounds.max.y);
    println!("    Z: [{:.4}, {:.4}]", bounds.min.z, bounds.max.z);

    // Compute stats by reading points
    println!("\nScanning points for statistics...");
    let mut classification_stats = BTreeMap::new();
    let mut color_count = 0;
    let mut intensity_count = 0;
    let mut gps_time_count = 0;
    let mut actual_point_count = 0;

    // We store the first 5 points for displaying later
    let mut sample_points = Vec::new();

    for wrapped_point in reader.points() {
        let p = wrapped_point?;
        actual_point_count += 1;
        
        let class_code: u8 = p.classification.into();
        *classification_stats.entry(class_code).or_insert(0) += 1;

        if p.color.is_some() {
            color_count += 1;
        }
        if p.intensity > 0 {
            intensity_count += 1;
        }
        if p.gps_time.is_some() {
            gps_time_count += 1;
        }

        if sample_points.len() < 5 {
            sample_points.push(p);
        }
    }

    println!("Scan Complete! Stats from {} scanned points:", actual_point_count);
    println!("  Points with Intensity: {} ({:.1}%)", intensity_count, (intensity_count as f64 / actual_point_count as f64) * 100.0);
    println!("  Points with Color:     {} ({:.1}%)", color_count, (color_count as f64 / actual_point_count as f64) * 100.0);
    println!("  Points with GPS Time:  {} ({:.1}%)", gps_time_count, (gps_time_count as f64 / actual_point_count as f64) * 100.0);
    
    println!("  Classification Distribution:");
    for (class_code, count) in &classification_stats {
        let name = match class_code {
            0 => "Created, never classified",
            1 => "Unclassified",
            2 => "Ground",
            3 => "Low Vegetation",
            4 => "Medium Vegetation",
            5 => "High Vegetation",
            6 => "Building",
            7 => "Low Point (noise)",
            8 => "Model Key Point",
            9 => "Water",
            12 => "Overlap Points",
            _ => "Reserved / Custom",
        };
        println!("    Code {:>2} ({:<26}): {:>10} ({:.1}%)", class_code, name, count, (*count as f64 / actual_point_count as f64) * 100.0);
    }

    if !sample_points.is_empty() {
        println!("\nSample Points (first {}):", sample_points.len());
        println!("    {:^5} | {:^10} | {:^10} | {:^10} | {:^9} | {:^15}", "Index", "X", "Y", "Z", "Intensity", "Color (R,G,B)");
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
