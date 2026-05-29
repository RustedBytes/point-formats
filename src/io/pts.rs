use crate::error::{Error, Result};
use crate::format::Format;
use crate::io::{fmt_f64, parse_f32, parse_f64, parse_u16};
use crate::types::{Color, Point, PointCloud};
use std::io::{BufRead, Write};

pub fn read<R: BufRead>(reader: &mut R) -> Result<PointCloud> {
    let mut cloud = PointCloud::empty();
    cloud.metadata.source_format = Some(Format::Pts);

    let mut expected_count: Option<usize> = None;
    let mut first_payload_line = true;

    for (line_idx, line) in reader.lines().enumerate() {
        let line_no = line_idx + 1;
        let line = line?;
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') || trimmed.starts_with("//") {
            continue;
        }

        let parts: Vec<&str> = trimmed.split_whitespace().collect();
        if first_payload_line && parts.len() == 1 {
            if let Ok(count) = parts[0].parse::<usize>() {
                expected_count = Some(count);
                first_payload_line = false;
                continue;
            }
        }
        first_payload_line = false;
        cloud.points.push(parse_pts_point(line_no, &parts)?);
    }

    if let Some(expected) = expected_count {
        if expected != cloud.points.len() {
            cloud.metadata.warnings.push(format!(
                "PTS header declared {expected} points but file contained {} point records",
                cloud.points.len()
            ));
        }
    }
    cloud.metadata.point_count_hint = Some(cloud.points.len());
    Ok(cloud)
}

pub fn write<W: Write>(writer: &mut W, cloud: &PointCloud) -> Result<()> {
    writeln!(writer, "{}", cloud.points.len())?;
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

fn parse_pts_point(line_no: usize, parts: &[&str]) -> Result<Point> {
    if parts.len() < 3 {
        return Err(Error::parse(
            Format::Pts,
            line_no,
            "expected at least x y z",
        ));
    }
    let mut point = Point::new(
        parse_f64(Format::Pts, line_no, "x", parts[0])?,
        parse_f64(Format::Pts, line_no, "y", parts[1])?,
        parse_f64(Format::Pts, line_no, "z", parts[2])?,
    );

    match parts.len() {
        3 => {}
        4 => point.intensity = Some(parse_f32(Format::Pts, line_no, "intensity", parts[3])?),
        6 => {
            point.color = Some(Color::new(
                parse_u16(Format::Pts, line_no, "red", parts[3])?,
                parse_u16(Format::Pts, line_no, "green", parts[4])?,
                parse_u16(Format::Pts, line_no, "blue", parts[5])?,
            ));
        }
        _ => {
            point.intensity = Some(parse_f32(Format::Pts, line_no, "intensity", parts[3])?);
            if parts.len() >= 7 {
                point.color = Some(Color::new(
                    parse_u16(Format::Pts, line_no, "red", parts[4])?,
                    parse_u16(Format::Pts, line_no, "green", parts[5])?,
                    parse_u16(Format::Pts, line_no, "blue", parts[6])?,
                ));
            }
        }
    }

    Ok(point)
}
