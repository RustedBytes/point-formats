use crate::error::{Error, Result};
use crate::types::{Color, Geometry, Point, PointCloud};
use las::{point::Format, Builder, Reader, Writer};
use std::io::{Read, Seek, Write};

/// Reads a point cloud from a LAS/LAZ stream.
pub fn read<R: Read + Seek + Send + 'static>(reader: R) -> Result<Geometry> {
    let mut reader = Reader::new(reader).map_err(|e| Error::invalid(e.to_string()))?;
    let mut points = Vec::new();
    for wrapped_point in reader.points() {
        let p = wrapped_point.map_err(|e| Error::invalid(e.to_string()))?;
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

/// Writes a point cloud to a LAS/LAZ stream.
pub fn write<W: Write + Seek + Send + 'static>(writer: W, cloud: &PointCloud) -> Result<()> {
    let has_color = cloud.has_color();
    let has_gps_time = cloud.has_gps_time();

    let format_val = match (has_gps_time, has_color) {
        (true, true) => 3,
        (false, true) => 2,
        (true, false) => 1,
        (false, false) => 0,
    };

    let mut builder = Builder::from((1, 2)); // LAS 1.2
    builder.point_format = Format::new(format_val).map_err(|e| Error::invalid(e.to_string()))?;

    let mut header = builder
        .into_header()
        .map_err(|e| Error::invalid(e.to_string()))?;

    let mut las_points = Vec::with_capacity(cloud.points.len());
    for p in &cloud.points {
        let mut las_point = las::Point {
            x: p.position.x,
            y: p.position.y,
            z: p.position.z,
            ..Default::default()
        };
        if let Some(i) = p.intensity {
            las_point.intensity = i as u16;
        }
        if let Some(c) = p.color {
            las_point.color = Some(las::Color::new(c.red, c.green, c.blue));
        }
        if let Some(cls) = p.classification {
            las_point.classification = las::point::Classification::new(cls)
                .unwrap_or(las::point::Classification::Unclassified);
        }
        if let Some(rn) = p.return_number {
            las_point.return_number = rn;
        }
        if let Some(nr) = p.number_of_returns {
            las_point.number_of_returns = nr;
        }
        if let Some(t) = p.gps_time {
            las_point.gps_time = Some(t);
        }
        if let Some(sa) = p.scan_angle {
            las_point.scan_angle = sa;
        }

        header.add_point(&las_point);
        las_points.push(las_point);
    }

    let mut writer = Writer::new(writer, header).map_err(|e| Error::invalid(e.to_string()))?;
    for las_point in las_points {
        writer
            .write_point(las_point)
            .map_err(|e| Error::invalid(e.to_string()))?;
    }
    writer.close().map_err(|e| Error::invalid(e.to_string()))?;

    Ok(())
}
