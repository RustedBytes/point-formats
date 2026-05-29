use lidar_format_convert::io::{PcdEncoding, PlyEncoding};
use lidar_format_convert::{convert_path, ConvertOptions, Format};
use std::env;
use std::str::FromStr;

fn main() {
    if let Err(error) = run() {
        eprintln!("error: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let mut args = env::args().skip(1).peekable();
    if args.peek().is_none() {
        print_help();
        return Ok(());
    }

    let mut options = ConvertOptions::default();
    let mut input = None;
    let mut output = None;

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                print_help();
                return Ok(());
            }
            "--list-formats" => {
                list_formats();
                return Ok(());
            }
            "--allow-lossy" => options.allow_lossy = true,
            "--input-format" => {
                let value = args
                    .next()
                    .ok_or_else(|| "--input-format requires a value".to_string())?;
                options.input_format = Some(Format::from_str(&value)?);
            }
            "--output-format" => {
                let value = args
                    .next()
                    .ok_or_else(|| "--output-format requires a value".to_string())?;
                options.output_format = Some(Format::from_str(&value)?);
            }
            "--binary-ply" => options.native.ply.encoding = PlyEncoding::BinaryLittleEndian,
            "--ascii-ply" => options.native.ply.encoding = PlyEncoding::Ascii,
            "--binary-pcd" => options.native.pcd.encoding = PcdEncoding::Binary,
            "--ascii-pcd" => options.native.pcd.encoding = PcdEncoding::Ascii,
            "--ascii-stl" => options.native.stl.binary = false,
            "--binary-stl" => options.native.stl.binary = true,
            value if value.starts_with('-') => return Err(format!("unknown option '{value}'")),
            value => {
                if input.is_none() {
                    input = Some(value.to_string());
                } else if output.is_none() {
                    output = Some(value.to_string());
                } else {
                    return Err(format!("unexpected positional argument '{value}'"));
                }
            }
        }
    }

    let input = input.ok_or_else(|| "missing input path".to_string())?;
    let output = output.ok_or_else(|| "missing output path".to_string())?;
    let report = convert_path(&input, &output, &options).map_err(|error| error.to_string())?;
    eprintln!(
        "converted {} -> {}: {} points, {} faces",
        report.input_format, report.output_format, report.points_written, report.faces_written
    );
    for warning in report.warnings {
        eprintln!("warning: {warning}");
    }
    Ok(())
}

fn print_help() {
    println!(
        "lidar-convert <input> <output> [options]\n\n\
Options:\n\
  --input-format <fmt>    Override input format detection\n\
  --output-format <fmt>   Override output format detection\n\
  --allow-lossy           Permit mesh->point conversion by dropping faces\n\
  --ascii-ply             Write ASCII PLY (default)\n\
  --binary-ply            Write binary little-endian PLY\n\
  --ascii-pcd             Write ASCII PCD (default)\n\
  --binary-pcd            Write binary PCD\n\
  --ascii-stl             Write ASCII STL\n\
  --binary-stl            Write binary STL (default)\n\
  --list-formats          Print represented formats and support status\n\
  -h, --help              Print this help\n"
    );
}

fn list_formats() {
    println!("{:<16} {:<20} support", "format", "family");
    for format in Format::ALL {
        let family = format!("{:?}", format.family());
        let support = format!("{:?}", format.support());
        println!("{:<16} {:<20} {}", format.name(), family, support);
    }
}
