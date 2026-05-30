use crate::error::{Error, Result};
use crate::format::Format;
use crate::io::{parse_f32, parse_f64, parse_u16};
use crate::types::{Color, Point, PointCloud};
use std::io::{BufRead, Write};

pub fn read<R: BufRead>(reader: &mut R) -> Result<PointCloud> {
    let mut header_lines: Vec<(usize, String)> = Vec::with_capacity(10);
    let mut line = String::new();
    let mut line_no = 0;

    while header_lines.len() < 10 {
        line.clear();
        let bytes_read = reader.read_line(&mut line)?;
        if bytes_read == 0 {
            return Err(Error::parse(
                Format::Ptx,
                None,
                "PTX requires columns, rows, four scanner basis lines, four transform lines, and point records",
            ));
        }
        line_no += 1;
        let trimmed = line.trim();
        if !trimmed.is_empty() && !trimmed.starts_with('#') && !trimmed.starts_with("//") {
            header_lines.push((line_no, trimmed.to_string()));
        }
    }

    let columns = parse_usize(header_lines[0].0, &header_lines[0].1, "columns")?;
    let rows = parse_usize(header_lines[1].0, &header_lines[1].1, "rows")?;
    let expected_points = columns.checked_mul(rows).ok_or_else(|| {
        Error::parse(
            Format::Ptx,
            header_lines[1].0,
            "columns * rows overflowed usize",
        )
    })?;

    let mut transform = [[0.0_f64; 4]; 4];
    for row in 0..4 {
        let (l_no, text) = &header_lines[6 + row];
        let mut parts_buf = [""; 16];
        let mut count = 0;
        for part in text.split_whitespace() {
            if count < 16 {
                parts_buf[count] = part;
                count += 1;
            } else {
                break;
            }
        }
        let parts = &parts_buf[..count];
        if parts.len() != 4 {
            return Err(Error::parse(
                Format::Ptx,
                *l_no,
                "expected four values in PTX transform row",
            ));
        }
        for (col, value) in parts.iter().enumerate() {
            transform[row][col] = parse_f64(Format::Ptx, *l_no, "transform", value)?;
        }
    }

    let mut cloud = PointCloud::empty();
    cloud.metadata.source_format = Some(Format::Ptx);
    cloud.metadata.point_count_hint = Some(expected_points);
    cloud.metadata.scanner_transform = Some(transform);

    while cloud.points.len() < expected_points {
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
        let mut parts_buf = [""; 16];
        let mut count = 0;
        for part in trimmed.split_whitespace() {
            if count < 16 {
                parts_buf[count] = part;
                count += 1;
            } else {
                break;
            }
        }
        let parts = &parts_buf[..count];
        if parts.len() < 3 {
            return Err(Error::parse(
                Format::Ptx,
                line_no,
                "expected point record x y z ...",
            ));
        }
        let mut point = Point::new(
            parse_f64(Format::Ptx, line_no, "x", parts[0])?,
            parse_f64(Format::Ptx, line_no, "y", parts[1])?,
            parse_f64(Format::Ptx, line_no, "z", parts[2])?,
        );
        if parts.len() >= 4 {
            point.intensity = Some(parse_f32(Format::Ptx, line_no, "intensity", parts[3])?);
        }
        if parts.len() >= 7 {
            point.color = Some(Color::new(
                parse_u16(Format::Ptx, line_no, "red", parts[4])?,
                parse_u16(Format::Ptx, line_no, "green", parts[5])?,
                parse_u16(Format::Ptx, line_no, "blue", parts[6])?,
            ));
        }
        cloud.points.push(point);
    }

    if cloud.points.len() != expected_points {
        cloud.metadata.warnings.push(format!(
            "PTX header declared {expected_points} records but only {} point lines were available",
            cloud.points.len()
        ));
    }

    Ok(cloud)
}

pub fn write<W: Write>(writer: &mut W, cloud: &PointCloud) -> Result<()> {
    writeln!(writer, "{}", cloud.points.len())?;
    writeln!(writer, "1")?;
    // Scanner origin and axes. PTX requires these lines even for unstructured exports.
    writeln!(writer, "0 0 0")?;
    writeln!(writer, "1 0 0")?;
    writeln!(writer, "0 1 0")?;
    writeln!(writer, "0 0 1")?;

    let transform = cloud.metadata.scanner_transform.unwrap_or([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    for row in transform {
        crate::io::write_fmt_f64(writer, row[0], 12)?;
        write!(writer, " ")?;
        crate::io::write_fmt_f64(writer, row[1], 12)?;
        write!(writer, " ")?;
        crate::io::write_fmt_f64(writer, row[2], 12)?;
        write!(writer, " ")?;
        crate::io::write_fmt_f64(writer, row[3], 12)?;
        writeln!(writer)?;
    }

    let has_intensity = cloud.has_intensity();
    let has_color = cloud.has_color();
    for point in &cloud.points {
        crate::io::write_fmt_f64(writer, point.position.x, 6)?;
        write!(writer, " ")?;
        crate::io::write_fmt_f64(writer, point.position.y, 6)?;
        write!(writer, " ")?;
        crate::io::write_fmt_f64(writer, point.position.z, 6)?;
        if has_intensity {
            write!(writer, " {}", point.intensity.unwrap_or(0.0))?;
        }
        if has_color {
            let color = point.color.unwrap_or(Color::new(0, 0, 0));
            write!(writer, " {} {} {}", color.red, color.green, color.blue)?;
        }
        writeln!(writer)?;
    }
    Ok(())
}

fn parse_usize(line_no: usize, text: &str, name: &str) -> Result<usize> {
    text.parse::<usize>().map_err(|_| {
        Error::parse(
            Format::Ptx,
            line_no,
            format!("expected integer {name}, got '{text}'"),
        )
    })
}
