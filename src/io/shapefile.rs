use crate::error::{Error, Result};
use crate::types::{Color, Geometry, Point, PointCloud};
use std::path::Path;

/// Reads a point cloud from an Esri Shapefile.
pub fn read(path: impl AsRef<Path>) -> Result<Geometry> {
    let mut reader = shapefile::Reader::from_path(path)
        .map_err(|e| Error::invalid(format!("Failed to open Shapefile: {}", e)))?;

    let mut points = Vec::new();
    for result in reader.iter_shapes_and_records() {
        let (shape, record) = result
            .map_err(|e| Error::invalid(format!("Failed to read shape/record: {}", e)))?;

        let mut pt = match shape {
            shapefile::Shape::Point(p) => Point::new(p.x, p.y, 0.0),
            shapefile::Shape::PointZ(pz) => Point::new(pz.x, pz.y, pz.z),
            _ => continue,
        };

        for (name, value) in record {
            let name_lower = name.to_lowercase();
            match name_lower.as_str() {
                "intensity" => {
                    if let Some(i) = to_f32(value) {
                        pt.intensity = Some(i);
                    }
                }
                "class" | "classific" | "classification" => {
                    if let Some(c) = to_u8(value) {
                        pt.classification = Some(c);
                    }
                }
                "color" | "hex_color" => {
                    if let Some(hex_str) = to_string(value) {
                        if let Some(c) = parse_hex_color(&hex_str) {
                            pt.color = Some(c);
                        }
                    }
                }
                "gps_time" | "gpstime" | "time" => {
                    if let Some(t) = to_f64(value) {
                        pt.gps_time = Some(t);
                    }
                }
                "scan_angle" | "scanangle" => {
                    if let Some(a) = to_f32(value) {
                        pt.scan_angle = Some(a);
                    }
                }
                _ => {}
            }
        }
        points.push(pt);
    }

    Ok(Geometry::PointCloud(PointCloud::new(points)))
}

/// Writes a point cloud to an Esri Shapefile.
pub fn write(path: impl AsRef<Path>, cloud: &PointCloud) -> Result<()> {
    if cloud.points.is_empty() {
        return Err(Error::invalid("Cannot write empty point cloud to Shapefile"));
    }

    let mut builder = dbase::TableWriterBuilder::new();

    let has_intensity = cloud.has_intensity();
    let has_classification = cloud.has_classification();
    let has_color = cloud.has_color();
    let has_gps_time = cloud.has_gps_time();
    let has_scan_angle = cloud.points.iter().any(|p| p.scan_angle.is_some());
    let has_any = has_intensity || has_classification || has_color || has_gps_time || has_scan_angle;

    if has_intensity {
        builder = builder.add_float_field(
            dbase::FieldName::try_from("intensity")
                .map_err(|e| Error::invalid(format!("FieldName error: {}", e)))?,
            12,
            6,
        );
    }
    if has_classification {
        builder = builder.add_integer_field(
            dbase::FieldName::try_from("class")
                .map_err(|e| Error::invalid(format!("FieldName error: {}", e)))?,
        );
    }
    if has_color {
        builder = builder.add_character_field(
            dbase::FieldName::try_from("color")
                .map_err(|e| Error::invalid(format!("FieldName error: {}", e)))?,
            7,
        );
    }
    if has_gps_time {
        builder = builder.add_double_field(
            dbase::FieldName::try_from("gps_time")
                .map_err(|e| Error::invalid(format!("FieldName error: {}", e)))?,
        );
    }
    if has_scan_angle {
        builder = builder.add_float_field(
            dbase::FieldName::try_from("scan_angle")
                .map_err(|e| Error::invalid(format!("FieldName error: {}", e)))?,
            12,
            6,
        );
    }
    if !has_any {
        builder = builder.add_integer_field(
            dbase::FieldName::try_from("id")
                .map_err(|e| Error::invalid(format!("FieldName error: {}", e)))?,
        );
    }

    let mut writer = shapefile::Writer::from_path(path, builder)
        .map_err(|e| Error::invalid(format!("Failed to create Shapefile writer: {}", e)))?;

    for (idx, p) in cloud.points.iter().enumerate() {
        let pt_z = shapefile::PointZ::new(p.position.x, p.position.y, p.position.z, shapefile::NO_DATA);

        let mut record = dbase::Record::default();
        if has_intensity {
            record.insert("intensity".to_string(), dbase::FieldValue::Float(p.intensity));
        }
        if has_classification {
            let val = p.classification.unwrap_or(0) as i32;
            record.insert("class".to_string(), dbase::FieldValue::Integer(val));
        }
        if has_color {
            let val = p.color.map(|c| {
                format!(
                    "#{:02x}{:02x}{:02x}",
                    c.red >> 8,
                    c.green >> 8,
                    c.blue >> 8
                )
            });
            record.insert("color".to_string(), dbase::FieldValue::Character(val));
        }
        if has_gps_time {
            let val = p.gps_time.unwrap_or(0.0);
            record.insert("gps_time".to_string(), dbase::FieldValue::Double(val));
        }
        if has_scan_angle {
            record.insert("scan_angle".to_string(), dbase::FieldValue::Float(p.scan_angle));
        }
        if !has_any {
            record.insert("id".to_string(), dbase::FieldValue::Integer(idx as i32));
        }

        writer
            .write_shape_and_record(&pt_z, &record)
            .map_err(|e| Error::invalid(format!("Failed to write shape/record: {}", e)))?;
    }

    Ok(())
}

fn to_f64(value: dbase::FieldValue) -> Option<f64> {
    match value {
        dbase::FieldValue::Numeric(Some(v)) => Some(v),
        dbase::FieldValue::Float(Some(v)) => Some(v as f64),
        dbase::FieldValue::Integer(v) => Some(v as f64),
        dbase::FieldValue::Double(v) => Some(v),
        dbase::FieldValue::Character(Some(s)) => s.trim().parse::<f64>().ok(),
        _ => None,
    }
}

fn to_f32(value: dbase::FieldValue) -> Option<f32> {
    to_f64(value).map(|v| v as f32)
}

fn to_u8(value: dbase::FieldValue) -> Option<u8> {
    match value {
        dbase::FieldValue::Integer(v) => Some(v as u8),
        dbase::FieldValue::Numeric(Some(v)) => Some(v as u8),
        dbase::FieldValue::Character(Some(s)) => s.trim().parse::<u8>().ok(),
        _ => None,
    }
}

fn to_string(value: dbase::FieldValue) -> Option<String> {
    match value {
        dbase::FieldValue::Character(Some(s)) => Some(s.trim().to_string()),
        _ => None,
    }
}

fn parse_hex_color(s: &str) -> Option<Color> {
    let s = s.trim_start_matches('#');
    if s.len() == 6 || s.len() == 8 {
        let r = u8::from_str_radix(&s[0..2], 16).ok()? as u16;
        let g = u8::from_str_radix(&s[2..4], 16).ok()? as u16;
        let b = u8::from_str_radix(&s[4..6], 16).ok()? as u16;
        Some(Color::new(r * 257, g * 257, b * 257))
    } else {
        None
    }
}
