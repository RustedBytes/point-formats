use crate::error::{Error, Result};
use crate::format::Format;
use crate::io::NativeOptions;
use crate::types::{Geometry, PointCloud};
use std::path::Path;

/// How the conversion pipeline should treat formats that can contain either
/// vertices-only point data or triangle meshes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GeometryPolicy {
    /// Preserve mesh faces when the destination supports them; otherwise require
    /// `allow_lossy` before dropping faces.
    #[default]
    Auto,
    /// Force point-cloud output. Mesh inputs drop faces only when `allow_lossy` is true.
    PointsOnly,
    /// Require mesh geometry.
    MeshOnly,
}

/// Conversion options. Defaults prioritize preservation and explicit errors over
/// silent lossy behavior.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConvertOptions {
    pub input_format: Option<Format>,
    pub output_format: Option<Format>,
    pub allow_lossy: bool,
    pub geometry_policy: GeometryPolicy,
    pub native: NativeOptions,
}

impl Default for ConvertOptions {
    fn default() -> Self {
        Self {
            input_format: None,
            output_format: None,
            allow_lossy: false,
            geometry_policy: GeometryPolicy::Auto,
            native: NativeOptions::default(),
        }
    }
}

/// Summary of a completed conversion.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConversionReport {
    pub input_format: Format,
    pub output_format: Format,
    pub points_read: usize,
    pub points_written: usize,
    pub faces_read: usize,
    pub faces_written: usize,
    pub warnings: Vec<String>,
}

/// Converts an input file to an output file using built-in native codecs.
///
/// Heavyweight formats such as LAS/LAZ/COPC/E57 are represented by [`Format`]
/// but require adapter codecs. The built-in path returns explicit
/// [`Error::UnsupportedFormat`] for those formats.
pub fn convert_path(
    input: impl AsRef<Path>,
    output: impl AsRef<Path>,
    options: &ConvertOptions,
) -> Result<ConversionReport> {
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

    if let Some(report) = crate::streaming::try_convert_path_streaming(input, output, options)? {
        return Ok(report);
    }

    let mut geometry = crate::io::read_path(input, input_format, &options.native)?;
    geometry.metadata_mut().source_format = Some(input_format);
    let points_read = geometry.point_count();
    let faces_read = geometry.face_count();

    geometry = apply_geometry_policy(geometry, output_format, options)?;

    let points_written = geometry.point_count();
    let faces_written = geometry.face_count();
    let warnings = geometry.metadata().warnings.clone();

    crate::io::write_path(output, output_format, &geometry, &options.native)?;

    Ok(ConversionReport {
        input_format,
        output_format,
        points_read,
        points_written,
        faces_read,
        faces_written,
        warnings,
    })
}

pub(crate) fn apply_geometry_policy(
    geometry: Geometry,
    output_format: Format,
    options: &ConvertOptions,
) -> Result<Geometry> {
    match options.geometry_policy {
        GeometryPolicy::Auto => coerce_for_output(geometry, output_format, options.allow_lossy),
        GeometryPolicy::PointsOnly => force_points(geometry, output_format, options.allow_lossy),
        GeometryPolicy::MeshOnly => match geometry {
            Geometry::Mesh(mesh) => Ok(Geometry::Mesh(mesh)),
            Geometry::PointCloud(_) => Err(Error::LossyConversionBlocked {
                from: "point cloud",
                to: output_format,
                reason: "mesh output was requested, but no meshing algorithm is configured"
                    .to_string(),
            }),
        },
    }
}

fn coerce_for_output(
    geometry: Geometry,
    output_format: Format,
    allow_lossy: bool,
) -> Result<Geometry> {
    match (&geometry, output_format) {
        (
            Geometry::Mesh(_),
            Format::Xyz | Format::Txt | Format::Csv | Format::Pts | Format::Ptx | Format::Pcd,
        ) => force_points(geometry, output_format, allow_lossy),
        _ => Ok(geometry),
    }
}

fn force_points(geometry: Geometry, output_format: Format, allow_lossy: bool) -> Result<Geometry> {
    match geometry {
        Geometry::PointCloud(cloud) => Ok(Geometry::PointCloud(cloud)),
        Geometry::Mesh(mesh) => {
            if !allow_lossy {
                return Err(Error::LossyConversionBlocked {
                    from: "mesh",
                    to: output_format,
                    reason: "faces would be discarded".to_string(),
                });
            }
            Ok(Geometry::PointCloud(mesh.vertex_cloud()))
        }
    }
}

/// Converts geometry already in memory to a point cloud, dropping faces only when
/// `allow_lossy` is true.
pub fn geometry_to_point_cloud(
    geometry: Geometry,
    destination: Format,
    allow_lossy: bool,
) -> Result<PointCloud> {
    match force_points(geometry, destination, allow_lossy)? {
        Geometry::PointCloud(cloud) => Ok(cloud),
        Geometry::Mesh(_) => Err(Error::invalid(
            "internal conversion error: mesh remained after force_points",
        )),
    }
}
