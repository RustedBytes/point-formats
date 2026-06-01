//! Coordinate quantization helpers.
//!
//! Quantization snaps coordinates to a uniform grid anchored at zero while
//! preserving point/vertex count and non-position attributes.

use crate::convert::{self, GeometryPolicy};
use crate::error::{Error, Result};
use crate::format::Format;
use crate::io::NativeOptions;
use crate::types::{Geometry, Mesh, Point, PointCloud, Vec3};
use std::path::Path;

/// Options for saving a quantized copy of a geometry file.
#[derive(Debug, Clone, PartialEq)]
pub struct QuantizeOptions {
    pub step: f64,
    pub input_format: Option<Format>,
    pub output_format: Option<Format>,
    pub allow_lossy: bool,
    pub geometry_policy: GeometryPolicy,
    pub native: NativeOptions,
}

/// Summary of a completed quantization.
#[derive(Debug, Clone, PartialEq)]
pub struct QuantizationReport {
    pub input_format: Format,
    pub output_format: Format,
    pub step: f64,
    pub points_read: usize,
    pub points_written: usize,
    pub faces_read: usize,
    pub faces_written: usize,
    pub warnings: Vec<String>,
}

/// Reads a file, snaps its coordinates to the configured grid, and writes it.
pub fn quantize_path(
    input: impl AsRef<Path>,
    output: impl AsRef<Path>,
    options: &QuantizeOptions,
) -> Result<QuantizationReport> {
    validate_step(options.step)?;

    let input = input.as_ref();
    let output = output.as_ref();
    let input_format = options
        .input_format
        .map(Ok)
        .unwrap_or_else(|| Format::from_path(input))?;
    let output_format = options
        .output_format
        .map(Ok)
        .unwrap_or_else(|| Format::from_path(output))?;

    let mut geometry = crate::io::read_path(input, input_format, &options.native)?;
    geometry.metadata_mut().source_format = Some(input_format);
    let points_read = geometry.point_count();
    let faces_read = geometry.face_count();

    let convert_options = crate::ConvertOptions {
        input_format: options.input_format,
        output_format: options.output_format,
        allow_lossy: options.allow_lossy,
        geometry_policy: options.geometry_policy,
        native: options.native.clone(),
    };
    geometry = convert::apply_geometry_policy(geometry, output_format, &convert_options)?;
    quantize_geometry(&mut geometry, options.step)?;

    let points_written = geometry.point_count();
    let faces_written = geometry.face_count();
    let warnings = geometry.metadata().warnings.clone();

    crate::io::write_path(output, output_format, &geometry, &options.native)?;

    Ok(QuantizationReport {
        input_format,
        output_format,
        step: options.step,
        points_read,
        points_written,
        faces_read,
        faces_written,
        warnings,
    })
}

/// Snaps all point or vertex positions in a geometry to the given grid step.
pub fn quantize_geometry(geometry: &mut Geometry, step: f64) -> Result<()> {
    validate_step(step)?;
    match geometry {
        Geometry::PointCloud(cloud) => quantize_point_cloud(cloud, step),
        Geometry::Mesh(mesh) => quantize_mesh(mesh, step),
    }
}

/// Snaps all point positions in a point cloud to the given grid step.
pub fn quantize_point_cloud(cloud: &mut PointCloud, step: f64) -> Result<()> {
    validate_step(step)?;
    for point in &mut cloud.points {
        quantize_point(point, step)?;
    }
    Ok(())
}

/// Snaps all vertex positions in a mesh to the given grid step.
pub fn quantize_mesh(mesh: &mut Mesh, step: f64) -> Result<()> {
    validate_step(step)?;
    for vertex in &mut mesh.vertices {
        vertex.position = quantize_vec3(vertex.position, step)?;
    }
    Ok(())
}

/// Snaps a point position to the given grid step.
pub fn quantize_point(point: &mut Point, step: f64) -> Result<()> {
    point.position = quantize_vec3(point.position, step)?;
    Ok(())
}

/// Snaps a vector to the given grid step.
pub fn quantize_vec3(value: Vec3, step: f64) -> Result<Vec3> {
    validate_step(step)?;
    Ok(Vec3::new(
        snap_value(value.x, step),
        snap_value(value.y, step),
        snap_value(value.z, step),
    ))
}

/// Snaps a scalar coordinate to the given grid step.
pub fn quantize_value(value: f64, step: f64) -> Result<f64> {
    validate_step(step)?;
    Ok(snap_value(value, step))
}

fn snap_value(value: f64, step: f64) -> f64 {
    let snapped = (value / step).round() * step;
    if snapped == 0.0 {
        0.0
    } else {
        snapped
    }
}

fn validate_step(step: f64) -> Result<()> {
    if step.is_finite() && step > 0.0 {
        Ok(())
    } else {
        Err(Error::invalid(
            "quantization step must be a finite positive number",
        ))
    }
}
