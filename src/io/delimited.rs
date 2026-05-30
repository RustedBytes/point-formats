use crate::error::{Error, Result};
use crate::format::Format;
use crate::io::{parse_f32, parse_f64, parse_u16, ColumnMapping, DelimitedOptions, Delimiter};
use crate::types::{Color, Metadata, Point, PointCloud, Vec3};
use std::io::{BufRead, Write};

pub fn read<R: BufRead>(
    reader: &mut R,
    format: Format,
    options: &DelimitedOptions,
) -> Result<PointCloud> {
    let mut cloud = PointCloud::empty();
    cloud.metadata.source_format = Some(format);

    let mut mapping = options.columns.clone();
    let mut delimiter = options.delimiter;
    let mut first_data_seen = false;
    let mut header_decided = options.has_header;

    let mut line = String::new();
    let mut line_no = 0;

    loop {
        line.clear();
        let bytes_read = reader.read_line(&mut line)?;
        if bytes_read == 0 {
            break;
        }
        line_no += 1;
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') || trimmed.starts_with("//") {
            continue;
        }

        if delimiter == Delimiter::Auto {
            delimiter = Delimiter::detect(trimmed);
        }
        let mut fields_buf = [""; 64];
        let fields_len = delimiter.split_into_slice(trimmed, &mut fields_buf);
        let fields = &fields_buf[..fields_len];
        if fields.is_empty() {
            continue;
        }

        if !first_data_seen {
            let line_is_header = match header_decided {
                Some(value) => value,
                None => !looks_like_point_line(format, line_no, &mapping, fields),
            };
            header_decided = Some(line_is_header);
            if line_is_header {
                if let Some(header_mapping) = ColumnMapping::from_header(fields) {
                    mapping = header_mapping;
                } else {
                    return Err(Error::parse(
                        format,
                        line_no,
                        "header must include x/y/z columns (accepted aliases: x/easting/lon, y/northing/lat, z/elevation/height)",
                    ));
                }
                first_data_seen = true;
                continue;
            }
            first_data_seen = true;
        }

        let point = parse_point_fields(format, line_no, &mapping, fields)?;
        cloud.points.push(point);
    }

    cloud.metadata.point_count_hint = Some(cloud.points.len());
    Ok(cloud)
}

pub fn write<W: Write>(
    writer: &mut W,
    format: Format,
    cloud: &PointCloud,
    options: &DelimitedOptions,
) -> Result<()> {
    let delimiter = match options.delimiter {
        Delimiter::Auto => {
            if matches!(format, Format::Csv) {
                Delimiter::Comma
            } else {
                Delimiter::Whitespace
            }
        }
        other => other,
    };
    let sep = delimiter.as_str();
    let has_intensity = cloud.has_intensity();
    let has_color = cloud.has_color();
    let has_classification = cloud.has_classification();
    let has_gps_time = cloud.has_gps_time();
    let has_normals = cloud.has_normals();

    if options.write_header || matches!(format, Format::Csv) {
        let mut header = vec!["x", "y", "z"];
        if has_intensity {
            header.push("intensity");
        }
        if has_color {
            header.extend(["red", "green", "blue"]);
        }
        if has_classification {
            header.push("classification");
        }
        if has_gps_time {
            header.push("gps_time");
        }
        if has_normals {
            header.extend(["normal_x", "normal_y", "normal_z"]);
        }
        writeln!(writer, "{}", header.join(sep))?;
    }

    for point in &cloud.points {
        crate::io::write_fmt_f64(writer, point.position.x, options.precision)?;
        write!(writer, "{}", sep)?;
        crate::io::write_fmt_f64(writer, point.position.y, options.precision)?;
        write!(writer, "{}", sep)?;
        crate::io::write_fmt_f64(writer, point.position.z, options.precision)?;

        if has_intensity {
            write!(writer, "{}", sep)?;
            if let Some(v) = point.intensity {
                write!(writer, "{:.*}", options.precision, v)?;
            }
        }
        if has_color {
            write!(writer, "{}", sep)?;
            if let Some(color) = point.color {
                write!(
                    writer,
                    "{}{}{}{}{}",
                    color.red, sep, color.green, sep, color.blue
                )?;
            } else {
                write!(writer, "{}{}", sep, sep)?;
            }
        }
        if has_classification {
            write!(writer, "{}", sep)?;
            if let Some(v) = point.classification {
                write!(writer, "{}", v)?;
            }
        }
        if has_gps_time {
            write!(writer, "{}", sep)?;
            if let Some(v) = point.gps_time {
                crate::io::write_fmt_f64(writer, v, options.precision)?;
            }
        }
        if has_normals {
            write!(writer, "{}", sep)?;
            if let Some(normal) = point.normal {
                crate::io::write_fmt_f64(writer, normal.x, options.precision)?;
                write!(writer, "{}", sep)?;
                crate::io::write_fmt_f64(writer, normal.y, options.precision)?;
                write!(writer, "{}", sep)?;
                crate::io::write_fmt_f64(writer, normal.z, options.precision)?;
            } else {
                write!(writer, "{}{}", sep, sep)?;
            }
        }
        writeln!(writer)?;
    }
    Ok(())
}

pub(crate) fn parse_point_fields(
    format: Format,
    line_no: usize,
    mapping: &ColumnMapping,
    fields: &[&str],
) -> Result<Point> {
    fn get<'a>(
        format: Format,
        line_no: usize,
        fields: &'a [&str],
        idx: usize,
        name: &str,
    ) -> Result<&'a str> {
        fields
            .get(idx)
            .copied()
            .filter(|s| !s.trim().is_empty())
            .ok_or_else(|| {
                Error::parse(
                    format,
                    line_no,
                    format!("missing required column {name} at index {idx}"),
                )
            })
    }

    fn opt<'a>(fields: &'a [&str], idx: Option<usize>) -> Option<&'a str> {
        idx.and_then(|i| fields.get(i).copied())
            .map(str::trim)
            .filter(|s| !s.is_empty())
    }

    let x = parse_f64(
        format,
        line_no,
        "x",
        get(format, line_no, fields, mapping.x, "x")?,
    )?;
    let y = parse_f64(
        format,
        line_no,
        "y",
        get(format, line_no, fields, mapping.y, "y")?,
    )?;
    let z = parse_f64(
        format,
        line_no,
        "z",
        get(format, line_no, fields, mapping.z, "z")?,
    )?;
    let mut point = Point::new(x, y, z);

    if let Some(value) = opt(fields, mapping.intensity) {
        point.intensity = Some(parse_f32(format, line_no, "intensity", value)?);
    }

    if let (Some(r), Some(g), Some(b)) = (
        opt(fields, mapping.red),
        opt(fields, mapping.green),
        opt(fields, mapping.blue),
    ) {
        point.color = Some(Color::new(
            parse_u16(format, line_no, "red", r)?,
            parse_u16(format, line_no, "green", g)?,
            parse_u16(format, line_no, "blue", b)?,
        ));
    }

    if let Some(value) = opt(fields, mapping.classification) {
        point.classification = Some(crate::io::parse_u8(
            format,
            line_no,
            "classification",
            value,
        )?);
    }

    if let Some(value) = opt(fields, mapping.gps_time) {
        point.gps_time = Some(parse_f64(format, line_no, "gps_time", value)?);
    }

    if let (Some(nx), Some(ny), Some(nz)) = (
        opt(fields, mapping.normal_x),
        opt(fields, mapping.normal_y),
        opt(fields, mapping.normal_z),
    ) {
        point.normal = Some(Vec3::new(
            parse_f64(format, line_no, "normal_x", nx)?,
            parse_f64(format, line_no, "normal_y", ny)?,
            parse_f64(format, line_no, "normal_z", nz)?,
        ));
    }

    if !point.position.is_finite() {
        return Err(Error::parse(
            format,
            line_no,
            "point coordinates must be finite",
        ));
    }

    Ok(point)
}

fn looks_like_point_line(
    format: Format,
    line_no: usize,
    mapping: &ColumnMapping,
    fields: &[&str],
) -> bool {
    let needed = [mapping.x, mapping.y, mapping.z];
    needed.iter().all(|&idx| {
        fields
            .get(idx)
            .copied()
            .map(|value| parse_f64(format, line_no, "coordinate", value).is_ok())
            .unwrap_or(false)
    })
}

#[allow(dead_code)]
pub(crate) fn cloud_with_format(mut cloud: PointCloud, format: Format) -> PointCloud {
    if cloud.metadata == Metadata::default() {
        cloud.metadata.source_format = Some(format);
    }
    cloud
}
