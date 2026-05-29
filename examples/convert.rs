use lidar_format_convert::{convert_path, ConvertOptions};
use std::env;
use std::path::Path;
use std::time::Instant;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 3 {
        eprintln!("Usage: cargo run --example convert <input_file> <output_file> [--allow-lossy]");
        std::process::exit(1);
    }
    
    let input_path_str = &args[1];
    let output_path_str = &args[2];
    
    let input_path = Path::new(input_path_str);
    if !input_path.exists() {
        eprintln!("Error: Input file not found at '{}'", input_path_str);
        std::process::exit(1);
    }

    let mut allow_lossy = false;
    if args.len() >= 4 && args[3] == "--allow-lossy" {
        allow_lossy = true;
    }

    let options = ConvertOptions {
        allow_lossy,
        ..Default::default()
    };

    println!("Converting '{}' to '{}'...", input_path_str, output_path_str);
    let start_time = Instant::now();
    let report = match convert_path(input_path, Path::new(output_path_str), &options) {
        Ok(rep) => rep,
        Err(err) => {
            eprintln!("\nConversion failed: {}", err);
            std::process::exit(1);
        }
    };
    let duration = start_time.elapsed();

    println!("\n============================================================");
    println!("                  CONVERSION REPORT                         ");
    println!("============================================================");
    println!("  Input Format:         {:?}", report.input_format);
    println!("  Output Format:        {:?}", report.output_format);
    println!("------------------------------------------------------------");
    println!("  Points Read:          {}", report.points_read);
    println!("  Points Written:       {}", report.points_written);
    println!("  Faces Read:           {}", report.faces_read);
    println!("  Faces Written:        {}", report.faces_written);
    println!("------------------------------------------------------------");
    println!("  Time Elapsed:         {:.3}s", duration.as_secs_f64());
    if report.points_read > 0 {
        let pts_per_sec = report.points_read as f64 / duration.as_secs_f64();
        println!("  Throughput:           {:.0} points/sec", pts_per_sec);
    }
    println!("============================================================");

    if !report.warnings.is_empty() {
        println!("\nWarnings during conversion:");
        for warning in &report.warnings {
            println!("  ⚠️  {}", warning);
        }
    } else {
        println!("\nConversion completed cleanly with 0 warnings!");
    }

    Ok(())
}
