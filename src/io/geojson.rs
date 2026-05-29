use crate::error::{Error, Result};
use crate::types::{Color, Geometry, Point, PointCloud, Vec3};
use std::io::{Read, Write};

/// Reads a point cloud from a GeoJSON stream.
pub fn read<R: Read>(reader: R) -> Result<Geometry> {
    let geojson: geojson::GeoJson = serde_json::from_reader(reader)
        .map_err(|e| Error::invalid(format!("Failed to parse GeoJSON: {}", e)))?;

    let mut points = Vec::new();
    match geojson {
        geojson::GeoJson::FeatureCollection(collection) => {
            for feature in collection.features {
                if let Some(geom) = &feature.geometry {
                    extract_points_from_geometry(geom, &mut points, feature.properties.as_ref());
                }
            }
        }
        geojson::GeoJson::Feature(feature) => {
            if let Some(geom) = &feature.geometry {
                extract_points_from_geometry(geom, &mut points, feature.properties.as_ref());
            }
        }
        geojson::GeoJson::Geometry(geom) => {
            extract_points_from_geometry(&geom, &mut points, None);
        }
    }

    Ok(Geometry::PointCloud(PointCloud::new(points)))
}

/// Writes a point cloud to a GeoJSON stream.
pub fn write<W: Write>(writer: W, cloud: &PointCloud) -> Result<()> {
    let mut features = Vec::with_capacity(cloud.points.len());
    for p in &cloud.points {
        let geometry = geojson::Geometry::new_point(vec![p.position.x, p.position.y, p.position.z]);
        let mut properties = geojson::JsonObject::new();
        if let Some(i) = p.intensity {
            properties.insert("intensity".to_string(), serde_json::Value::from(i));
        }
        if let Some(c) = p.classification {
            properties.insert("classification".to_string(), serde_json::Value::from(c));
        }
        if let Some(rn) = p.return_number {
            properties.insert("return_number".to_string(), serde_json::Value::from(rn));
        }
        if let Some(nr) = p.number_of_returns {
            properties.insert("number_of_returns".to_string(), serde_json::Value::from(nr));
        }
        if let Some(t) = p.gps_time {
            properties.insert("gps_time".to_string(), serde_json::Value::from(t));
        }
        if let Some(a) = p.scan_angle {
            properties.insert("scan_angle".to_string(), serde_json::Value::from(a));
        }
        if let Some(n) = p.normal {
            properties.insert("normal_x".to_string(), serde_json::Value::from(n.x));
            properties.insert("normal_y".to_string(), serde_json::Value::from(n.y));
            properties.insert("normal_z".to_string(), serde_json::Value::from(n.z));
        }
        if let Some(color) = p.color {
            let hex = format!(
                "#{:02x}{:02x}{:02x}",
                color.red >> 8,
                color.green >> 8,
                color.blue >> 8
            );
            properties.insert("color".to_string(), serde_json::Value::from(hex));
        }

        let feature = geojson::Feature {
            bbox: None,
            geometry: Some(geometry),
            id: None,
            properties: Some(properties),
            foreign_members: None,
        };
        features.push(feature);
    }

    let collection = geojson::FeatureCollection {
        bbox: None,
        features,
        foreign_members: None,
    };

    let geojson = geojson::GeoJson::FeatureCollection(collection);
    serde_json::to_writer(writer, &geojson)
        .map_err(|e| Error::invalid(format!("Failed to write GeoJSON: {}", e)))?;
    Ok(())
}

fn extract_points_from_geometry(
    geom: &geojson::Geometry,
    points: &mut Vec<Point>,
    properties: Option<&geojson::JsonObject>,
) {
    match &geom.value {
        geojson::GeometryValue::Point { coordinates } => {
            if let Some(mut p) = parse_geojson_coords(coordinates.as_slice()) {
                apply_geojson_properties(&mut p, properties);
                points.push(p);
            }
        }
        geojson::GeometryValue::MultiPoint { coordinates } => {
            for coords in coordinates {
                if let Some(mut p) = parse_geojson_coords(coords.as_slice()) {
                    apply_geojson_properties(&mut p, properties);
                    points.push(p);
                }
            }
        }
        geojson::GeometryValue::LineString { coordinates } => {
            for coords in coordinates {
                if let Some(mut p) = parse_geojson_coords(coords.as_slice()) {
                    apply_geojson_properties(&mut p, properties);
                    points.push(p);
                }
            }
        }
        geojson::GeometryValue::MultiLineString { coordinates } => {
            for coords_list in coordinates {
                for coords in coords_list {
                    if let Some(mut p) = parse_geojson_coords(coords.as_slice()) {
                        apply_geojson_properties(&mut p, properties);
                        points.push(p);
                    }
                }
            }
        }
        geojson::GeometryValue::Polygon { coordinates } => {
            for coords_list in coordinates {
                for coords in coords_list {
                    if let Some(mut p) = parse_geojson_coords(coords.as_slice()) {
                        apply_geojson_properties(&mut p, properties);
                        points.push(p);
                    }
                }
            }
        }
        geojson::GeometryValue::MultiPolygon { coordinates } => {
            for coords_list_list in coordinates {
                for coords_list in coords_list_list {
                    for coords in coords_list {
                        if let Some(mut p) = parse_geojson_coords(coords.as_slice()) {
                            apply_geojson_properties(&mut p, properties);
                            points.push(p);
                        }
                    }
                }
            }
        }
        geojson::GeometryValue::GeometryCollection { geometries } => {
            for geom in geometries {
                extract_points_from_geometry(geom, points, properties);
            }
        }
    }
}

fn parse_geojson_coords(coords: &[f64]) -> Option<Point> {
    if coords.len() >= 2 {
        let x = coords[0];
        let y = coords[1];
        let z = if coords.len() >= 3 { coords[2] } else { 0.0 };
        Some(Point::new(x, y, z))
    } else {
        None
    }
}

fn apply_geojson_properties(point: &mut Point, properties: Option<&geojson::JsonObject>) {
    let Some(props) = properties else {
        return;
    };

    // Intensity
    if let Some(val) = props.get("intensity") {
        if let Some(i) = val.as_f64() {
            point.intensity = Some(i as f32);
        }
    }

    // Classification
    if let Some(val) = props.get("classification") {
        if let Some(c) = val.as_u64() {
            point.classification = Some(c as u8);
        }
    }

    // Return number & number of returns
    if let Some(val) = props.get("return_number") {
        if let Some(r) = val.as_u64() {
            point.return_number = Some(r as u8);
        }
    }
    if let Some(val) = props.get("number_of_returns") {
        if let Some(r) = val.as_u64() {
            point.number_of_returns = Some(r as u8);
        }
    }

    // GPS time
    if let Some(val) = props.get("gps_time") {
        if let Some(t) = val.as_f64() {
            point.gps_time = Some(t);
        }
    }

    // Scan angle
    if let Some(val) = props.get("scan_angle") {
        if let Some(a) = val.as_f64() {
            point.scan_angle = Some(a as f32);
        }
    }

    // Normal
    let nx = props.get("normal_x").and_then(|v| v.as_f64());
    let ny = props.get("normal_y").and_then(|v| v.as_f64());
    let nz = props.get("normal_z").and_then(|v| v.as_f64());
    if let (Some(x), Some(y), Some(z)) = (nx, ny, nz) {
        point.normal = Some(Vec3::new(x, y, z));
    }

    // Color
    if let Some(val) = props.get("color") {
        if let Some(hex_str) = val.as_str() {
            if let Some(c) = parse_hex_color(hex_str) {
                point.color = Some(c);
            }
        } else if let Some(arr) = val.as_array() {
            if arr.len() >= 3 {
                let r = arr[0].as_u64().unwrap_or(0) as u16;
                let g = arr[1].as_u64().unwrap_or(0) as u16;
                let b = arr[2].as_u64().unwrap_or(0) as u16;
                let scale = if r <= 255 && g <= 255 && b <= 255 {
                    257
                } else {
                    1
                };
                point.color = Some(Color::new(r * scale, g * scale, b * scale));
            }
        }
    } else {
        let r_val = props.get("red").or_else(|| props.get("r"));
        let g_val = props.get("green").or_else(|| props.get("g"));
        let b_val = props.get("blue").or_else(|| props.get("b"));
        if let (Some(r), Some(g), Some(b)) = (r_val, g_val, b_val) {
            let r = r.as_u64().unwrap_or(0) as u16;
            let g = g.as_u64().unwrap_or(0) as u16;
            let b = b.as_u64().unwrap_or(0) as u16;
            let scale = if r <= 255 && g <= 255 && b <= 255 {
                257
            } else {
                1
            };
            point.color = Some(Color::new(r * scale, g * scale, b * scale));
        }
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
