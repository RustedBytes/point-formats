use crate::error::{Error, Result};
use crate::format::Format;
use crate::io::{fmt_f64, parse_f32, parse_f64, parse_u16};
use crate::types::{Color, Point, PointCloud};
use std::io::{BufRead, Write};

pub fn read<R: BufRead>(reader: &mut R) -> Result<PointCloud> {
    let mut lines: Vec<(usize, String)> = Vec::new();
    for (idx, line) in reader.lines().enumerate() {
        let line = line?;
        let trimmed = line.trim();
        if !trimmed.is_empty() && !trimmed.starts_with('#') && !trimmed.starts_with("//") {
            lines.push((idx + 1, trimmed.to_string()));
        }
    }

    if lines.len() < 10 {
        return Err(Error::parse(
            Format::Ptx,
            None,
            "PTX requires columns, rows, four scanner basis lines, four transform lines, and point records",
        ));
    }

    let columns = parse_usize(lines[0].0, &lines[0].1, "columns")?;
    let rows = parse_usize(lines[1].0, &lines[1].1, "rows")?;
    let expected_points = columns
        .checked_mul(rows)
        .ok_or_else(|| Error::parse(Format::Ptx, lines[1].0, "columns * rows overflowed usize"))?;

    let mut transform = [[0.0_f64; 4]; 4];
    for row in 0..4 {
        let (line_no, text) = &lines[6 + row];
        let parts: Vec<&str> = text.split_whitespace().collect();
        if parts.len() != 4 {
            return Err(Error::parse(
                Format::Ptx,
                *line_no,
                "expected four values in PTX transform row",
            ));
        }
        for (col, value) in parts.iter().enumerate() {
            transform[row][col] = parse_f64(Format::Ptx, *line_no, "transform", value)?;
        }
    }

    let mut cloud = PointCloud::empty();
    cloud.metadata.source_format = Some(Format::Ptx);
    cloud.metadata.point_count_hint = Some(expected_points);
    cloud.metadata.scanner_transform = Some(transform);

    for (line_no, text) in lines.iter().skip(10).take(expected_points) {
        let parts: Vec<&str> = text.split_whitespace().collect();
        if parts.len() < 3 {
            return Err(Error::parse(
                Format::Ptx,
                *line_no,
                "expected point record x y z ...",
            ));
        }
        let mut point = Point::new(
            parse_f64(Format::Ptx, *line_no, "x", parts[0])?,
            parse_f64(Format::Ptx, *line_no, "y", parts[1])?,
            parse_f64(Format::Ptx, *line_no, "z", parts[2])?,
        );
        if parts.len() >= 4 {
            point.intensity = Some(parse_f32(Format::Ptx, *line_no, "intensity", parts[3])?);
        }
        if parts.len() >= 7 {
            point.color = Some(Color::new(
                parse_u16(Format::Ptx, *line_no, "red", parts[4])?,
                parse_u16(Format::Ptx, *line_no, "green", parts[5])?,
                parse_u16(Format::Ptx, *line_no, "blue", parts[6])?,
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
        writeln!(
            writer,
            "{} {} {} {}",
            fmt_f64(row[0], 12),
            fmt_f64(row[1], 12),
            fmt_f64(row[2], 12),
            fmt_f64(row[3], 12)
        )?;
    }

    let has_intensity = cloud.has_intensity();
    let has_color = cloud.has_color();
    for point in &cloud.points {
        write!(
            writer,
            "{} {} {}",
            fmt_f64(point.position.x, 6),
            fmt_f64(point.position.y, 6),
            fmt_f64(point.position.z, 6)
        )?;
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
