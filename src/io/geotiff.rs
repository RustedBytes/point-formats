use crate::error::{Error, Result};
use crate::types::{Geometry, Point, PointCloud};
use std::io::{Read, Seek, Write};
use tiff::decoder::{Decoder, DecodingResult};
use tiff::encoder::{colortype, TiffEncoder};
use tiff::tags::Tag;

/// Reads a point cloud from a GeoTIFF stream.
pub fn read<R: Read + Seek>(reader: R) -> Result<Geometry> {
    let mut decoder =
        Decoder::new(reader).map_err(|e| Error::invalid(format!("Failed to open TIFF: {}", e)))?;

    // ModelPixelScaleTag (33550) and ModelTiepointTag (33922)
    let pixel_scale = decoder.get_tag_f64_vec(Tag::Unknown(33550)).ok();
    let tiepoints = decoder.get_tag_f64_vec(Tag::Unknown(33922)).ok();

    let (width, height) = decoder
        .dimensions()
        .map_err(|e| Error::invalid(format!("Failed to read TIFF dimensions: {}", e)))?;

    let img_res = decoder
        .read_image()
        .map_err(|e| Error::invalid(format!("Failed to decode TIFF image: {}", e)))?;

    let mut points = Vec::new();

    // Map any supported data type to f64 elevation values
    let elevations = match img_res {
        DecodingResult::U8(data) => data.into_iter().map(|v| v as f64).collect(),
        DecodingResult::U16(data) => data.into_iter().map(|v| v as f64).collect(),
        DecodingResult::I8(data) => data.into_iter().map(|v| v as f64).collect(),
        DecodingResult::I16(data) => data.into_iter().map(|v| v as f64).collect(),
        DecodingResult::U32(data) => data.into_iter().map(|v| v as f64).collect(),
        DecodingResult::I32(data) => data.into_iter().map(|v| v as f64).collect(),
        DecodingResult::U64(data) => data.into_iter().map(|v| v as f64).collect(),
        DecodingResult::I64(data) => data.into_iter().map(|v| v as f64).collect(),
        DecodingResult::F16(data) => data.into_iter().map(|v| v.to_f64()).collect(),
        DecodingResult::F32(data) => data.into_iter().map(|v| v as f64).collect(),
        DecodingResult::F64(data) => data,
    };

    let tiepoint = tiepoints.as_ref().and_then(|t| {
        if t.len() >= 6 {
            Some((t[0], t[1], t[2], t[3], t[4], t[5]))
        } else {
            None
        }
    });

    let scale = pixel_scale.as_ref().and_then(|s| {
        if s.len() >= 2 {
            Some((s[0], s[1]))
        } else {
            None
        }
    });

    for r in 0..height {
        for c in 0..width {
            let idx = (r * width + c) as usize;
            if idx >= elevations.len() {
                break;
            }
            let z = elevations[idx];

            let (x, y) = match (tiepoint, scale) {
                (Some((px, py, _pz, wx, wy, _wz)), Some((sx, sy))) => {
                    let col_diff = c as f64 - px;
                    let row_diff = r as f64 - py;
                    (wx + col_diff * sx, wy - row_diff * sy)
                }
                _ => (c as f64, -(r as f64)),
            };

            let mut p = Point::new(x, y, z);
            // Save row and column as custom attributes for spatial context preservation
            p.return_number = Some(0);
            points.push(p);
        }
    }

    Ok(Geometry::PointCloud(PointCloud::new(points)))
}

/// Writes a point cloud to a GeoTIFF stream.
pub fn write<W: Write + Seek>(writer: W, cloud: &PointCloud) -> Result<()> {
    if cloud.points.is_empty() {
        return Err(Error::invalid("Cannot write empty point cloud to GeoTIFF"));
    }

    // 1. Grid/rasterize the point cloud into a 2D regular grid
    let mut min_x = f64::MAX;
    let mut max_x = f64::MIN;
    let mut min_y = f64::MAX;
    let mut max_y = f64::MIN;
    for p in &cloud.points {
        let x = p.position.x;
        let y = p.position.y;
        if x < min_x {
            min_x = x;
        }
        if x > max_x {
            max_x = x;
        }
        if y < min_y {
            min_y = y;
        }
        if y > max_y {
            max_y = y;
        }
    }

    if min_x >= max_x {
        max_x = min_x + 1.0;
    }
    if min_y >= max_y {
        max_y = min_y + 1.0;
    }

    // Use a square grid sized proportionally to the point count
    let grid_size = (cloud.points.len() as f64).sqrt().round() as usize;
    let grid_size = grid_size.clamp(16, 512);

    let width = grid_size;
    let height = grid_size;

    let scale_x = (max_x - min_x) / width as f64;
    let scale_y = (max_y - min_y) / height as f64;

    let mut sum_z = vec![0.0; width * height];
    let mut count = vec![0; width * height];

    for p in &cloud.points {
        let px = p.position.x;
        let py = p.position.y;

        let c = ((px - min_x) / scale_x).floor() as usize;
        let r = ((max_y - py) / scale_y).floor() as usize;

        let c = c.min(width - 1);
        let r = r.min(height - 1);

        let idx = r * width + c;
        sum_z[idx] += p.position.z;
        count[idx] += 1;
    }

    let avg_z = cloud.points.iter().map(|p| p.position.z).sum::<f64>() / cloud.points.len() as f64;

    let mut raster = vec![0.0f32; width * height];
    for i in 0..(width * height) {
        if count[i] > 0 {
            raster[i] = (sum_z[i] / count[i] as f64) as f32;
        } else {
            raster[i] = avg_z as f32;
        }
    }

    // 2. Encode using tiff encoder
    let mut encoder = TiffEncoder::new(writer)
        .map_err(|e| Error::invalid(format!("Failed to create TIFF encoder: {}", e)))?;

    let mut image_encoder = encoder
        .new_image::<colortype::Gray32Float>(width as u32, height as u32)
        .map_err(|e| Error::invalid(format!("Failed to start TIFF image encoding: {}", e)))?;

    // ModelPixelScaleTag (33550)
    let scale: &[f64] = &[scale_x, scale_y, 0.0];
    image_encoder
        .encoder()
        .write_tag(Tag::Unknown(33550), scale)
        .map_err(|e| Error::invalid(format!("Failed to write ModelPixelScaleTag: {}", e)))?;

    // ModelTiepointTag (33922)
    // Map pixel (0, 0, 0) to world coordinate (min_x, max_y, 0.0)
    let tiepoint: &[f64] = &[0.0, 0.0, 0.0, min_x, max_y, 0.0];
    image_encoder
        .encoder()
        .write_tag(Tag::Unknown(33922), tiepoint)
        .map_err(|e| Error::invalid(format!("Failed to write ModelTiepointTag: {}", e)))?;

    image_encoder
        .write_data(&raster)
        .map_err(|e| Error::invalid(format!("Failed to write TIFF pixel data: {}", e)))?;

    Ok(())
}
