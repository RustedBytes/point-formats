use crate::error::{Error, Result};
use crate::types::{Color as NativeColor, Geometry, Metadata, Point as NativePoint, PointCloud};
use e57::{CartesianCoordinate, E57Reader, E57Writer, Record, RecordDataType, RecordName, RecordValue};
use std::io::{Read, Seek};
use std::path::Path;

/// Reads a point cloud from an E57 stream.
pub fn read<R: Read + Seek>(reader: &mut R) -> Result<Geometry> {
    let mut e57_reader = E57Reader::new(reader).map_err(|e| Error::invalid(e.to_string()))?;
    let pointclouds = e57_reader.pointclouds();

    let mut points = Vec::new();
    let mut metadata = Metadata::default();

    if let Some(meta) = e57_reader.coordinate_metadata() {
        metadata.crs_wkt = Some(meta.to_string());
    }

    for pc in &pointclouds {
        let mut pc_reader = e57_reader
            .pointcloud_simple(pc)
            .map_err(|e| Error::invalid(e.to_string()))?;
        pc_reader.normalize_intensity(false);
        for p in pc_reader {
            let p = p.map_err(|e| Error::invalid(e.to_string()))?;
            let mut native_point = match p.cartesian {
                CartesianCoordinate::Valid { x, y, z } => NativePoint::new(x, y, z),
                CartesianCoordinate::Direction { x, y, z } => NativePoint::new(x, y, z),
                CartesianCoordinate::Invalid => NativePoint::new(0.0, 0.0, 0.0),
            };
            if let Some(intensity) = p.intensity {
                native_point.intensity = Some(intensity);
            }
            if let Some(c) = p.color {
                let red = (c.red.clamp(0.0, 1.0) * 65535.0).round() as u16;
                let green = (c.green.clamp(0.0, 1.0) * 65535.0).round() as u16;
                let blue = (c.blue.clamp(0.0, 1.0) * 65535.0).round() as u16;
                native_point.color = Some(NativeColor::new(red, green, blue));
            }
            points.push(native_point);
        }
    }

    let mut cloud = PointCloud::new(points);
    cloud.metadata = metadata;
    Ok(Geometry::PointCloud(cloud))
}

/// Writes a point cloud to an E57 file path.
pub fn write_to_path(path: &Path, cloud: &PointCloud) -> Result<()> {
    let file_guid = uuid::Uuid::new_v4().to_string();
    let mut e57_writer = E57Writer::from_file(path, &file_guid).map_err(|e| Error::invalid(e.to_string()))?;

    if let Some(crs) = &cloud.metadata.crs_wkt {
        e57_writer.set_coordinate_metadata(Some(crs.clone()));
    }

    let pc_guid = uuid::Uuid::new_v4().to_string();
    let has_color = cloud.has_color();
    let has_intensity = cloud.has_intensity();

    let mut prototype = vec![
        Record::CARTESIAN_X_F64,
        Record::CARTESIAN_Y_F64,
        Record::CARTESIAN_Z_F64,
    ];
    if has_color {
        prototype.push(Record::COLOR_RED_U8);
        prototype.push(Record::COLOR_GREEN_U8);
        prototype.push(Record::COLOR_BLUE_U8);
    }
    if has_intensity {
        prototype.push(Record {
            name: RecordName::Intensity,
            data_type: RecordDataType::Single { min: None, max: None },
        });
    }

    let mut pc_writer = e57_writer
        .add_pointcloud(&pc_guid, prototype)
        .map_err(|e| Error::invalid(e.to_string()))?;

    for p in &cloud.points {
        let mut values = vec![
            RecordValue::Double(p.position.x),
            RecordValue::Double(p.position.y),
            RecordValue::Double(p.position.z),
        ];
        if has_color {
            if let Some(c) = p.color {
                values.push(RecordValue::Integer((c.red >> 8) as i64));
                values.push(RecordValue::Integer((c.green >> 8) as i64));
                values.push(RecordValue::Integer((c.blue >> 8) as i64));
            } else {
                values.push(RecordValue::Integer(0));
                values.push(RecordValue::Integer(0));
                values.push(RecordValue::Integer(0));
            }
        }
        if has_intensity {
            let intensity = p.intensity.unwrap_or(0.0);
            values.push(RecordValue::Single(intensity));
        }
        pc_writer
            .add_point(values)
            .map_err(|e| Error::invalid(e.to_string()))?;
    }

    pc_writer.finalize().map_err(|e| Error::invalid(e.to_string()))?;
    e57_writer.finalize().map_err(|e| Error::invalid(e.to_string()))?;

    Ok(())
}
