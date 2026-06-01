use point_formats::io::{PcdEncoding, PlyEncoding};
use point_formats::{
    convert_path, quantize_path, ConvertOptions, Format, QuantizeDType, QuantizeMode,
    QuantizeOptions,
};
use std::env;
use std::str::FromStr;

fn main() {
    if let Err(error) = run_with_args(env::args().skip(1)) {
        eprintln!("error: {error}");
        std::process::exit(2);
    }
}

fn run_with_args<I>(args: I) -> Result<(), String>
where
    I: IntoIterator<Item = String>,
{
    let mut args = args.into_iter().peekable();
    if args.peek().is_none() {
        print_help();
        return Ok(());
    }

    let mut options = ConvertOptions::default();
    let mut quantize_mode = None;
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
            "--quantize-step" => {
                let value = args
                    .next()
                    .ok_or_else(|| "--quantize-step requires a value".to_string())?;
                let step = value.parse::<f64>().map_err(|_| {
                    format!("--quantize-step requires a numeric value, got '{value}'")
                })?;
                set_quantize_mode(&mut quantize_mode, QuantizeMode::Step(step))?;
            }
            "--quantize-dtype" => {
                let value = args
                    .next()
                    .ok_or_else(|| "--quantize-dtype requires a value".to_string())?;
                let dtype = QuantizeDType::from_str(&value).map_err(|error| error.to_string())?;
                set_quantize_mode(&mut quantize_mode, QuantizeMode::DType(dtype))?;
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
    if let Some(mode) = quantize_mode {
        let quantize_options = QuantizeOptions {
            mode,
            input_format: options.input_format,
            output_format: options.output_format,
            allow_lossy: options.allow_lossy,
            geometry_policy: options.geometry_policy,
            native: options.native,
        };
        let report =
            quantize_path(&input, &output, &quantize_options).map_err(|error| error.to_string())?;
        eprintln!(
            "quantized {} -> {} with {}: {} points, {} faces",
            report.input_format,
            report.output_format,
            report.mode,
            report.points_written,
            report.faces_written
        );
        for warning in report.warnings {
            eprintln!("warning: {warning}");
        }
    } else {
        let report = convert_path(&input, &output, &options).map_err(|error| error.to_string())?;
        eprintln!(
            "converted {} -> {}: {} points, {} faces",
            report.input_format, report.output_format, report.points_written, report.faces_written
        );
        for warning in report.warnings {
            eprintln!("warning: {warning}");
        }
    }
    Ok(())
}

fn set_quantize_mode(target: &mut Option<QuantizeMode>, mode: QuantizeMode) -> Result<(), String> {
    if target.is_some() {
        Err("--quantize-step and --quantize-dtype are mutually exclusive".to_string())
    } else {
        *target = Some(mode);
        Ok(())
    }
}

fn print_help() {
    println!(
        "points-convert <input> <output> [options]\n\n\
Options:\n\
  --input-format <fmt>    Override input format detection\n\
  --output-format <fmt>   Override output format detection\n\
  --allow-lossy           Permit mesh->point conversion by dropping faces\n\
  --quantize-step <step>  Snap output coordinates to a grid step\n\
  --quantize-dtype <dt>   Quantize output coordinates to f16/bf16/f32/f64/int8/uint8\n\
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_main_cli() {
        // Test no args
        assert!(run_with_args(vec![]).is_ok());

        // Test help
        assert!(run_with_args(vec!["--help".to_string()]).is_ok());
        assert!(run_with_args(vec!["-h".to_string()]).is_ok());

        // Test list formats
        assert!(run_with_args(vec!["--list-formats".to_string()]).is_ok());

        // Test invalid option
        assert!(run_with_args(vec!["--invalid-option".to_string()]).is_err());

        // Test unexpected positional argument
        assert!(run_with_args(vec![
            "in.xyz".to_string(),
            "out.ply".to_string(),
            "extra.ply".to_string(),
        ])
        .is_err());

        // Test missing output path
        assert!(run_with_args(vec!["in.xyz".to_string()]).is_err());

        // Test missing and invalid quantization step
        assert!(run_with_args(vec!["--quantize-step".to_string()]).is_err());
        assert!(run_with_args(vec![
            "--quantize-step".to_string(),
            "nope".to_string(),
            "in.xyz".to_string(),
            "out.ply".to_string(),
        ])
        .is_err());
        assert!(run_with_args(vec!["--quantize-dtype".to_string()]).is_err());
        assert!(run_with_args(vec![
            "--quantize-dtype".to_string(),
            "float8".to_string(),
            "in.xyz".to_string(),
            "out.ply".to_string(),
        ])
        .is_err());
        assert!(run_with_args(vec![
            "--quantize-step".to_string(),
            "0.5".to_string(),
            "--quantize-dtype".to_string(),
            "f16".to_string(),
            "in.xyz".to_string(),
            "out.ply".to_string(),
        ])
        .is_err());

        // Test valid options parsing, then fails on execution due to missing file (which is fine)
        let res = run_with_args(vec![
            "--allow-lossy".to_string(),
            "--input-format".to_string(),
            "xyz".to_string(),
            "--output-format".to_string(),
            "ply".to_string(),
            "--binary-ply".to_string(),
            "--ascii-ply".to_string(),
            "--binary-pcd".to_string(),
            "--ascii-pcd".to_string(),
            "--ascii-stl".to_string(),
            "--binary-stl".to_string(),
            "--quantize-step".to_string(),
            "0.01".to_string(),
            "nonexistent_file.xyz".to_string(),
            "output_file.ply".to_string(),
        ]);
        assert!(res.is_err());
        let err = res.unwrap_err();
        assert!(err.contains("No such file or directory") || err.contains("entity not found"));
    }

    #[test]
    fn test_main_cli_quantizes_file() {
        let dir =
            std::env::temp_dir().join(format!("points_cli_quantize_{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&dir).unwrap();
        let input = dir.join("scan.xyz");
        let output = dir.join("scan.xyz");
        std::fs::write(&input, "0.24 0.26 -0.26\n").unwrap();

        let result = run_with_args(vec![
            input.display().to_string(),
            output.display().to_string(),
            "--quantize-step".to_string(),
            "0.5".to_string(),
        ]);
        assert!(result.is_ok());

        let contents = std::fs::read_to_string(&output).unwrap();
        assert!(contents.contains("0.000000 0.500000 -0.500000"));

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn test_main_cli_quantizes_dtype_file() {
        let dir = std::env::temp_dir().join(format!("points_cli_dtype_{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&dir).unwrap();
        let input = dir.join("scan.xyz");
        let output = dir.join("scan.xyz");
        std::fs::write(&input, "1.0002 2.0002 3.0002\n").unwrap();

        let result = run_with_args(vec![
            input.display().to_string(),
            output.display().to_string(),
            "--quantize-dtype".to_string(),
            "f16".to_string(),
        ]);
        assert!(result.is_ok());

        let contents = std::fs::read_to_string(&output).unwrap();
        assert!(contents.contains("1.000000 2.000000 3.000000"));

        let _ = std::fs::remove_dir_all(&dir);
    }
}
