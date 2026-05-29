use crate::error::{Error, Result};
use crate::types::{Color, Geometry, Point, PointCloud};
use copc_rs::{BoundsSelection, CopcReader, LodSelection};
use std::io::{Read, Seek};

/// Reads a point cloud from a COPC stream.
pub fn read<R: Read + Seek>(reader: &mut R) -> Result<Geometry> {
    let mut copc_reader = CopcReader::new(reader).map_err(|e| Error::invalid(e.to_string()))?;

    let iter = copc_reader
        .points(LodSelection::All, BoundsSelection::All)
        .map_err(|e| Error::invalid(e.to_string()))?;

    let mut points = Vec::new();
    for p in iter {
        let mut point = Point::new(p.x, p.y, p.z);
        if p.intensity > 0 {
            point.intensity = Some(p.intensity as f32);
        }
        if let Some(c) = p.color {
            point.color = Some(Color::new(c.red, c.green, c.blue));
        }
        let class_code: u8 = p.classification.into();
        if class_code > 0 {
            point.classification = Some(class_code);
        }
        if p.return_number > 0 {
            point.return_number = Some(p.return_number);
        }
        if p.number_of_returns > 0 {
            point.number_of_returns = Some(p.number_of_returns);
        }
        if let Some(t) = p.gps_time {
            point.gps_time = Some(t);
        }
        if p.scan_angle != 0.0 {
            point.scan_angle = Some(p.scan_angle);
        }
        points.push(point);
    }

    Ok(Geometry::PointCloud(PointCloud::new(points)))
}
