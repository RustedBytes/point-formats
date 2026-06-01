//! Coordinate quantization helpers.
//!
//! Quantization snaps coordinates to a uniform grid or to values representable
//! by a selected numeric dtype while preserving point/vertex count and
//! non-position attributes.

use crate::convert::{self, GeometryPolicy};
use crate::error::{Error, Result};
use crate::format::Format;
use crate::io::NativeOptions;
use crate::types::{Geometry, Mesh, Point, PointCloud, Vec3};
use std::fmt;
use std::path::Path;
use std::str::FromStr;

/// Quantization mode for coordinate values.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum QuantizeMode {
    /// Snap coordinates to a uniform grid step anchored at zero.
    Step(f64),
    /// Round coordinates to the representable values/range of a dtype.
    DType(QuantizeDType),
}

impl fmt::Display for QuantizeMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Step(step) => write!(f, "step {step}"),
            Self::DType(dtype) => write!(f, "{dtype}"),
        }
    }
}

/// Numeric dtype used for value quantization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuantizeDType {
    F16,
    Bf16,
    F32,
    F64,
    Int8,
    UInt8,
}

impl fmt::Display for QuantizeDType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::F16 => "f16",
            Self::Bf16 => "bf16",
            Self::F32 => "f32",
            Self::F64 => "f64",
            Self::Int8 => "int8",
            Self::UInt8 => "uint8",
        })
    }
}

impl FromStr for QuantizeDType {
    type Err = Error;

    fn from_str(value: &str) -> Result<Self> {
        match value.trim().to_ascii_lowercase().as_str() {
            "f16" | "float16" => Ok(Self::F16),
            "bf16" | "bfloat16" => Ok(Self::Bf16),
            "f32" | "float32" => Ok(Self::F32),
            "f64" | "float64" => Ok(Self::F64),
            "int8" | "i8" => Ok(Self::Int8),
            "uint8" | "u8" => Ok(Self::UInt8),
            _ => Err(Error::invalid(format!(
                "unknown quantization dtype '{value}'"
            ))),
        }
    }
}

/// Options for saving a quantized copy of a geometry file.
#[derive(Debug, Clone, PartialEq)]
pub struct QuantizeOptions {
    pub mode: QuantizeMode,
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
    pub mode: QuantizeMode,
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
    validate_mode(options.mode)?;

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
    quantize_geometry(&mut geometry, options.mode)?;

    let points_written = geometry.point_count();
    let faces_written = geometry.face_count();
    let warnings = geometry.metadata().warnings.clone();

    crate::io::write_path(output, output_format, &geometry, &options.native)?;

    Ok(QuantizationReport {
        input_format,
        output_format,
        mode: options.mode,
        points_read,
        points_written,
        faces_read,
        faces_written,
        warnings,
    })
}

/// Quantizes all point or vertex positions in a geometry.
pub fn quantize_geometry(geometry: &mut Geometry, mode: QuantizeMode) -> Result<()> {
    validate_mode(mode)?;
    match geometry {
        Geometry::PointCloud(cloud) => quantize_point_cloud(cloud, mode),
        Geometry::Mesh(mesh) => quantize_mesh(mesh, mode),
    }
}

/// Quantizes all point positions in a point cloud.
pub fn quantize_point_cloud(cloud: &mut PointCloud, mode: QuantizeMode) -> Result<()> {
    validate_mode(mode)?;
    for point in &mut cloud.points {
        quantize_point(point, mode)?;
    }
    Ok(())
}

/// Quantizes all vertex positions in a mesh.
pub fn quantize_mesh(mesh: &mut Mesh, mode: QuantizeMode) -> Result<()> {
    validate_mode(mode)?;
    for vertex in &mut mesh.vertices {
        vertex.position = quantize_vec3(vertex.position, mode)?;
    }
    Ok(())
}

/// Quantizes a point position.
pub fn quantize_point(point: &mut Point, mode: QuantizeMode) -> Result<()> {
    point.position = quantize_vec3(point.position, mode)?;
    Ok(())
}

/// Quantizes a vector.
pub fn quantize_vec3(value: Vec3, mode: QuantizeMode) -> Result<Vec3> {
    validate_mode(mode)?;
    Ok(Vec3::new(
        quantize_value(value.x, mode)?,
        quantize_value(value.y, mode)?,
        quantize_value(value.z, mode)?,
    ))
}

/// Quantizes a scalar coordinate.
pub fn quantize_value(value: f64, mode: QuantizeMode) -> Result<f64> {
    validate_mode(mode)?;
    match mode {
        QuantizeMode::Step(step) => Ok(snap_step_value(value, step)),
        QuantizeMode::DType(dtype) => quantize_dtype_value(value, dtype),
    }
}

/// Snaps a scalar coordinate to the given grid step.
pub fn quantize_step_value(value: f64, step: f64) -> Result<f64> {
    validate_step(step)?;
    Ok(snap_step_value(value, step))
}

/// Quantizes a scalar coordinate to the given dtype.
pub fn quantize_dtype_value(value: f64, dtype: QuantizeDType) -> Result<f64> {
    if !value.is_finite() {
        return Err(Error::invalid(
            "dtype quantization requires finite coordinate values",
        ));
    }
    let quantized = match dtype {
        QuantizeDType::F16 => half::f16::from_f32(value as f32).to_f32() as f64,
        QuantizeDType::Bf16 => half::bf16::from_f32(value as f32).to_f32() as f64,
        QuantizeDType::F32 => (value as f32) as f64,
        QuantizeDType::F64 => value,
        QuantizeDType::Int8 => quantize_integer_value(value, i8::MIN as f64, i8::MAX as f64)?,
        QuantizeDType::UInt8 => quantize_integer_value(value, u8::MIN as f64, u8::MAX as f64)?,
    };
    Ok(normalize_zero(quantized))
}

fn snap_step_value(value: f64, step: f64) -> f64 {
    let snapped = (value / step).round() * step;
    normalize_zero(snapped)
}

fn quantize_integer_value(value: f64, min: f64, max: f64) -> Result<f64> {
    let rounded = value.round();
    if (min..=max).contains(&rounded) {
        Ok(rounded)
    } else {
        Err(Error::invalid(format!(
            "coordinate {value} is outside dtype range {min}..{max}"
        )))
    }
}

fn normalize_zero(value: f64) -> f64 {
    if value == 0.0 {
        0.0
    } else {
        value
    }
}

fn validate_mode(mode: QuantizeMode) -> Result<()> {
    match mode {
        QuantizeMode::Step(step) => validate_step(step),
        QuantizeMode::DType(_) => Ok(()),
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
