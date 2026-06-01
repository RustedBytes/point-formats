use point_formats::io::{PcdEncoding, PlyEncoding};
use point_formats::{
    convert_path, quantize_path, AttributeValue, ConvertOptions, Format, Geometry, Mesh, Metadata,
    PointCloud, QuantizeDType, QuantizeMode, QuantizeOptions, Vec3,
};
use std::env;
use std::path::Path;
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
    let mut inspect = false;
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
            "--inspect" => inspect = true,
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
    if inspect {
        if output.is_some() {
            return Err("--inspect accepts exactly one input path".to_string());
        }
        if options.output_format.is_some() {
            return Err("--inspect cannot be combined with --output-format".to_string());
        }
        if quantize_mode.is_some() {
            return Err("--inspect cannot be combined with quantization options".to_string());
        }
        let report = inspect_path(&input, &options).map_err(|error| error.to_string())?;
        print!("{report}");
        return Ok(());
    }

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

fn inspect_path(path: impl AsRef<Path>, options: &ConvertOptions) -> point_formats::Result<String> {
    let path = path.as_ref();
    let format = options
        .input_format
        .map(Ok)
        .unwrap_or_else(|| Format::from_path(path))?;
    let mut geometry = point_formats::io::read_path(path, format, &options.native)?;
    geometry.metadata_mut().source_format = Some(format);
    Ok(format_inspection(path, format, &geometry))
}

fn format_inspection(path: &Path, format: Format, geometry: &Geometry) -> String {
    let mut out = String::new();
    push_line(&mut out, "path", &path.display().to_string());
    push_line(&mut out, "format", format.name());
    push_line(&mut out, "family", &format!("{:?}", format.family()));
    push_line(&mut out, "support", &format!("{:?}", format.support()));
    push_line(
        &mut out,
        "geometry",
        match geometry {
            Geometry::PointCloud(_) => "point-cloud",
            Geometry::Mesh(_) => "mesh",
        },
    );
    push_line(&mut out, "points", &geometry.point_count().to_string());
    push_line(&mut out, "faces", &geometry.face_count().to_string());
    if let Some(bounds) = geometry_bounds(geometry) {
        push_line(&mut out, "bounds.min", &format_vec3(bounds.min));
        push_line(&mut out, "bounds.max", &format_vec3(bounds.max));
    } else {
        push_line(&mut out, "bounds", "none");
    }

    match geometry {
        Geometry::PointCloud(cloud) => append_cloud_fields(&mut out, cloud),
        Geometry::Mesh(mesh) => append_mesh_fields(&mut out, mesh),
    }
    append_metadata(&mut out, geometry.metadata());
    out
}

fn append_cloud_fields(out: &mut String, cloud: &PointCloud) {
    push_line(out, "has.intensity", bool_str(cloud.has_intensity()));
    push_line(out, "has.color", bool_str(cloud.has_color()));
    push_line(
        out,
        "has.classification",
        bool_str(cloud.has_classification()),
    );
    push_line(out, "has.gps_time", bool_str(cloud.has_gps_time()));
    push_line(out, "has.normals", bool_str(cloud.has_normals()));
}

fn append_mesh_fields(out: &mut String, mesh: &Mesh) {
    let has_color = mesh.vertices.iter().any(|vertex| vertex.color.is_some());
    let has_normals = mesh.vertices.iter().any(|vertex| vertex.normal.is_some());
    push_line(out, "has.color", bool_str(has_color));
    push_line(out, "has.normals", bool_str(has_normals));
}

fn append_metadata(out: &mut String, metadata: &Metadata) {
    if let Some(source_format) = metadata.source_format {
        push_line(out, "metadata.source_format", source_format.name());
    }
    if let Some(point_count_hint) = metadata.point_count_hint {
        push_line(
            out,
            "metadata.point_count_hint",
            &point_count_hint.to_string(),
        );
    }
    push_line(
        out,
        "metadata.crs_wkt",
        presence_str(metadata.crs_wkt.is_some()),
    );
    push_line(
        out,
        "metadata.scanner_transform",
        presence_str(metadata.scanner_transform.is_some()),
    );
    push_line(
        out,
        "metadata.comments",
        &metadata.comments.len().to_string(),
    );
    push_line(
        out,
        "metadata.warnings",
        &metadata.warnings.len().to_string(),
    );
    push_line(
        out,
        "metadata.attributes",
        &metadata.attributes.len().to_string(),
    );
    for (idx, comment) in metadata.comments.iter().take(5).enumerate() {
        push_line(out, &format!("metadata.comment.{idx}"), comment);
    }
    for (idx, warning) in metadata.warnings.iter().take(5).enumerate() {
        push_line(out, &format!("metadata.warning.{idx}"), warning);
    }
    for (key, value) in metadata.attributes.iter().take(10) {
        push_line(
            out,
            &format!("metadata.attribute.{key}"),
            &format_attribute(value),
        );
    }
}

fn geometry_bounds(geometry: &Geometry) -> Option<point_formats::Bounds3> {
    match geometry {
        Geometry::PointCloud(cloud) => cloud.bounds(),
        Geometry::Mesh(mesh) => mesh.bounds(),
    }
}

fn format_attribute(value: &AttributeValue) -> String {
    match value {
        AttributeValue::Int(value) => value.to_string(),
        AttributeValue::UInt(value) => value.to_string(),
        AttributeValue::Float(value) => value.to_string(),
        AttributeValue::Text(value) => value.clone(),
    }
}

fn format_vec3(value: Vec3) -> String {
    format!("{} {} {}", value.x, value.y, value.z)
}

fn bool_str(value: bool) -> &'static str {
    if value {
        "yes"
    } else {
        "no"
    }
}

fn presence_str(value: bool) -> &'static str {
    if value {
        "present"
    } else {
        "none"
    }
}

fn push_line(out: &mut String, key: &str, value: &str) {
    out.push_str(key);
    out.push_str(": ");
    out.push_str(value);
    out.push('\n');
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
Inspect:\n\
  points-convert --inspect <input> [--input-format <fmt>]\n\n\
Options:\n\
  --input-format <fmt>    Override input format detection\n\
  --output-format <fmt>   Override output format detection\n\
  --inspect               Print decoded file metadata and exit\n\
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

        // Test inspect validation
        assert!(run_with_args(vec!["--inspect".to_string()]).is_err());
        assert!(run_with_args(vec![
            "--inspect".to_string(),
            "in.xyz".to_string(),
            "out.ply".to_string(),
        ])
        .is_err());
        assert!(run_with_args(vec![
            "--inspect".to_string(),
            "--output-format".to_string(),
            "ply".to_string(),
            "in.xyz".to_string(),
        ])
        .is_err());
        assert!(run_with_args(vec![
            "--inspect".to_string(),
            "--quantize-step".to_string(),
            "0.5".to_string(),
            "in.xyz".to_string(),
        ])
        .is_err());

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

    #[test]
    fn test_main_cli_inspects_file() {
        let dir = std::env::temp_dir().join(format!("points_cli_inspect_{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&dir).unwrap();
        let input = dir.join("scan.xyz");
        std::fs::write(&input, "1 2 3\n4 5 6 7\n").unwrap();

        let result = run_with_args(vec!["--inspect".to_string(), input.display().to_string()]);
        assert!(result.is_ok());

        let report = inspect_path(&input, &ConvertOptions::default()).unwrap();
        assert!(report.contains("format: xyz\n"));
        assert!(report.contains("geometry: point-cloud\n"));
        assert!(report.contains("points: 2\n"));
        assert!(report.contains("faces: 0\n"));
        assert!(report.contains("bounds.min: 1 2 3\n"));
        assert!(report.contains("bounds.max: 4 5 6\n"));
        assert!(report.contains("has.intensity: yes\n"));
        assert!(report.contains("metadata.source_format: xyz\n"));

        let _ = std::fs::remove_dir_all(&dir);
    }
}
