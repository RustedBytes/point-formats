//! Native readers and writers.

pub mod delimited;
pub mod obj;
pub mod pcd;
pub mod ply;
pub mod pts;
pub mod ptx;
pub mod stl;

#[cfg(feature = "copc")]
pub mod copc;
#[cfg(feature = "e57")]
pub mod e57;
#[cfg(feature = "geospatial")]
pub mod geojson;
#[cfg(feature = "geospatial")]
pub mod geotiff;
#[cfg(feature = "las")]
pub mod las;

pub mod asciigrid;
#[cfg(feature = "dxf")]
pub mod dxf;
#[cfg(feature = "gltf")]
pub mod gltf;
#[cfg(feature = "gpkg")]
pub mod gpkg;
#[cfg(feature = "robotics")]
pub mod robotics;
#[cfg(feature = "sensor")]
pub mod sensor;
#[cfg(feature = "shapefile")]
pub mod shapefile;

use crate::error::{Error, Result};
use crate::format::Format;
use crate::types::Geometry;
use std::fs::File;
use std::io::{BufReader, BufWriter};
use std::path::Path;

/// Delimiter used by delimited text point files.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Delimiter {
    Auto,
    Whitespace,
    Comma,
    Tab,
    Semicolon,
}

impl Delimiter {
    #[inline]
    pub(crate) fn split_into_slice<'a>(self, line: &'a str, fields: &mut [&'a str]) -> usize {
        let mut count = 0;
        let limit = fields.len();
        match self {
            Self::Auto => Self::detect(line).split_into_slice(line, fields),
            Self::Whitespace => {
                for part in line.split_whitespace() {
                    if count < limit {
                        fields[count] = part;
                        count += 1;
                    } else {
                        break;
                    }
                }
                count
            }
            Self::Comma => {
                for part in line.split(',') {
                    if count < limit {
                        fields[count] = part.trim();
                        count += 1;
                    } else {
                        break;
                    }
                }
                count
            }
            Self::Tab => {
                for part in line.split('\t') {
                    if count < limit {
                        fields[count] = part.trim();
                        count += 1;
                    } else {
                        break;
                    }
                }
                count
            }
            Self::Semicolon => {
                for part in line.split(';') {
                    if count < limit {
                        fields[count] = part.trim();
                        count += 1;
                    } else {
                        break;
                    }
                }
                count
            }
        }
    }

    #[inline]
    pub(crate) fn detect(line: &str) -> Self {
        if line.contains(',') {
            Self::Comma
        } else if line.contains(';') {
            Self::Semicolon
        } else if line.contains('\t') {
            Self::Tab
        } else {
            Self::Whitespace
        }
    }

    #[inline]
    pub(crate) const fn as_str(self) -> &'static str {
        match self {
            Self::Auto | Self::Whitespace => " ",
            Self::Comma => ",",
            Self::Tab => "\t",
            Self::Semicolon => ";",
        }
    }
}

/// Column positions for delimited text files. Missing optional columns are `None`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ColumnMapping {
    pub x: usize,
    pub y: usize,
    pub z: usize,
    pub intensity: Option<usize>,
    pub red: Option<usize>,
    pub green: Option<usize>,
    pub blue: Option<usize>,
    pub classification: Option<usize>,
    pub gps_time: Option<usize>,
    pub normal_x: Option<usize>,
    pub normal_y: Option<usize>,
    pub normal_z: Option<usize>,
}

impl Default for ColumnMapping {
    fn default() -> Self {
        Self {
            x: 0,
            y: 1,
            z: 2,
            intensity: Some(3),
            red: Some(4),
            green: Some(5),
            blue: Some(6),
            classification: Some(7),
            gps_time: Some(8),
            normal_x: Some(9),
            normal_y: Some(10),
            normal_z: Some(11),
        }
    }
}

impl ColumnMapping {
    pub(crate) fn from_header(header: &[&str]) -> Option<Self> {
        fn find(header: &[&str], names: &[&str]) -> Option<usize> {
            header.iter().position(|value| {
                let normalized = value
                    .trim()
                    .trim_matches('"')
                    .trim_matches('\'')
                    .to_ascii_lowercase()
                    .replace([' ', '-', '.'], "_");
                names.iter().any(|candidate| normalized == *candidate)
            })
        }

        let x = find(header, &["x", "easting", "east", "lon", "longitude"])?;
        let y = find(header, &["y", "northing", "north", "lat", "latitude"])?;
        let z = find(header, &["z", "elevation", "height", "altitude"])?;
        Some(Self {
            x,
            y,
            z,
            intensity: find(header, &["intensity", "i"]),
            red: find(header, &["red", "r"]),
            green: find(header, &["green", "g"]),
            blue: find(header, &["blue", "b"]),
            classification: find(header, &["classification", "class", "label"]),
            gps_time: find(header, &["gps_time", "gpstime", "time", "timestamp"]),
            normal_x: find(header, &["normal_x", "nx", "n_x"]),
            normal_y: find(header, &["normal_y", "ny", "n_y"]),
            normal_z: find(header, &["normal_z", "nz", "n_z"]),
        })
    }
}

/// Options for XYZ/TXT/CSV-style formats.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DelimitedOptions {
    pub delimiter: Delimiter,
    /// `None` means autodetect by trying to parse the first non-comment line.
    pub has_header: Option<bool>,
    pub columns: ColumnMapping,
    pub write_header: bool,
    pub precision: usize,
}

impl Default for DelimitedOptions {
    fn default() -> Self {
        Self {
            delimiter: Delimiter::Auto,
            has_header: None,
            columns: ColumnMapping::default(),
            write_header: false,
            precision: 6,
        }
    }
}

/// PLY encoding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlyEncoding {
    Ascii,
    BinaryLittleEndian,
}

/// PLY writer options.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlyOptions {
    pub encoding: PlyEncoding,
    pub precision: usize,
}

impl Default for PlyOptions {
    fn default() -> Self {
        Self {
            encoding: PlyEncoding::Ascii,
            precision: 6,
        }
    }
}

/// PCD encoding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PcdEncoding {
    Ascii,
    Binary,
}

/// PCD writer options.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PcdOptions {
    pub encoding: PcdEncoding,
    pub precision: usize,
}

impl Default for PcdOptions {
    fn default() -> Self {
        Self {
            encoding: PcdEncoding::Ascii,
            precision: 6,
        }
    }
}

/// STL writer options.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StlOptions {
    pub binary: bool,
}

impl Default for StlOptions {
    fn default() -> Self {
        Self { binary: true }
    }
}

/// Native reader/writer options.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct NativeOptions {
    pub delimited: DelimitedOptions,
    pub ply: PlyOptions,
    pub pcd: PcdOptions,
    pub stl: StlOptions,
}

pub fn read_path(
    path: impl AsRef<Path>,
    format: Format,
    options: &NativeOptions,
) -> Result<Geometry> {
    let path = path.as_ref();

    #[cfg(feature = "shapefile")]
    if matches!(format, Format::Shapefile) {
        return shapefile::read(path);
    }

    #[cfg(feature = "gltf")]
    if matches!(format, Format::Gltf | Format::Glb) {
        return gltf::read(path);
    }

    #[cfg(feature = "gpkg")]
    if matches!(format, Format::Gpkg) {
        return gpkg::read(path);
    }

    #[cfg(feature = "robotics")]
    if matches!(format, Format::RosBag) {
        return robotics::read_rosbag(path);
    }

    #[cfg(feature = "robotics")]
    if matches!(format, Format::Ros2Bag) {
        return robotics::read_ros2bag(path);
    }

    #[cfg(feature = "robotics")]
    if matches!(format, Format::PointCloud2) {
        return robotics::read_pc2(path);
    }

    #[cfg(feature = "sensor")]
    if matches!(format, Format::Pcap) {
        return sensor::read_pcap(path);
    }

    #[cfg(feature = "sensor")]
    if matches!(format, Format::UdpPackets) {
        return sensor::read_udppackets(path);
    }

    #[cfg(feature = "sensor")]
    if matches!(format, Format::VendorRaw) {
        return sensor::read_vendorraw(path);
    }

    let file = File::open(path)?;
    let mut reader = BufReader::new(file);
    match format {
        Format::Xyz | Format::Txt | Format::Csv => {
            let mut opts = options.delimited.clone();
            if matches!(format, Format::Csv) && matches!(opts.delimiter, Delimiter::Auto) {
                opts.delimiter = Delimiter::Comma;
                opts.write_header = true;
            }
            delimited::read(&mut reader, format, &opts).map(Geometry::PointCloud)
        }
        Format::Pts => pts::read(&mut reader).map(Geometry::PointCloud),
        Format::Ptx => ptx::read(&mut reader).map(Geometry::PointCloud),
        Format::Ply => ply::read(&mut reader),
        Format::Pcd => pcd::read(&mut reader),
        Format::Obj => obj::read(&mut reader),
        Format::Stl => stl::read(&mut reader),
        Format::AsciiGrid => asciigrid::read(&mut reader),

        #[cfg(feature = "dxf")]
        Format::Dxf => dxf::read(&mut reader),

        #[cfg(feature = "las")]
        Format::Las | Format::Laz => las::read(reader),

        #[cfg(feature = "copc")]
        Format::Copc => copc::read(&mut reader),

        #[cfg(feature = "e57")]
        Format::E57 => e57::read(&mut reader),

        #[cfg(feature = "geospatial")]
        Format::GeoTiff | Format::Cog => geotiff::read(reader),

        #[cfg(feature = "geospatial")]
        Format::GeoJson => geojson::read(reader),

        _ => Err(Error::unsupported(format, "read", format.adapter_hint())),
    }
}

pub fn write_path(
    path: impl AsRef<Path>,
    format: Format,
    geometry: &Geometry,
    options: &NativeOptions,
) -> Result<()> {
    let path = path.as_ref();

    #[cfg(feature = "e57")]
    if matches!(format, Format::E57) {
        let cloud = as_cloud_for_point_format(geometry, format)?;
        return e57::write_to_path(path, cloud);
    }

    #[cfg(feature = "shapefile")]
    if matches!(format, Format::Shapefile) {
        let cloud = as_cloud_for_point_format(geometry, format)?;
        return shapefile::write(path, cloud);
    }

    #[cfg(feature = "gltf")]
    if matches!(format, Format::Gltf) {
        return gltf::write_gltf(path, geometry);
    }

    #[cfg(feature = "gltf")]
    if matches!(format, Format::Glb) {
        return gltf::write_glb(path, geometry);
    }

    #[cfg(feature = "gpkg")]
    if matches!(format, Format::Gpkg) {
        let cloud = as_cloud_for_point_format(geometry, format)?;
        return gpkg::write(path, cloud);
    }

    #[cfg(feature = "robotics")]
    if matches!(format, Format::RosBag) {
        let cloud = as_cloud_for_point_format(geometry, format)?;
        return robotics::write_rosbag(path, cloud);
    }

    #[cfg(feature = "robotics")]
    if matches!(format, Format::Ros2Bag) {
        let cloud = as_cloud_for_point_format(geometry, format)?;
        return robotics::write_ros2bag(path, cloud);
    }

    #[cfg(feature = "robotics")]
    if matches!(format, Format::PointCloud2) {
        let cloud = as_cloud_for_point_format(geometry, format)?;
        return robotics::write_pc2(path, cloud);
    }

    #[cfg(feature = "sensor")]
    if matches!(format, Format::Pcap) {
        let cloud = as_cloud_for_point_format(geometry, format)?;
        return sensor::write_pcap(path, cloud);
    }

    #[cfg(feature = "sensor")]
    if matches!(format, Format::UdpPackets) {
        let cloud = as_cloud_for_point_format(geometry, format)?;
        return sensor::write_udppackets(path, cloud);
    }

    #[cfg(feature = "sensor")]
    if matches!(format, Format::VendorRaw) {
        let cloud = as_cloud_for_point_format(geometry, format)?;
        return sensor::write_vendorraw(path, cloud);
    }

    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);
    match format {
        Format::Xyz | Format::Txt | Format::Csv => {
            let cloud = as_cloud_for_point_format(geometry, format)?;
            let mut opts = options.delimited.clone();
            if matches!(format, Format::Csv) {
                if matches!(opts.delimiter, Delimiter::Auto) {
                    opts.delimiter = Delimiter::Comma;
                }
                opts.write_header = true;
            }
            delimited::write(&mut writer, format, cloud, &opts)
        }
        Format::Pts => pts::write(&mut writer, as_cloud_for_point_format(geometry, format)?),
        Format::Ptx => ptx::write(&mut writer, as_cloud_for_point_format(geometry, format)?),
        Format::Ply => ply::write(&mut writer, geometry, &options.ply),
        Format::Pcd => pcd::write(
            &mut writer,
            as_cloud_for_point_format(geometry, format)?,
            &options.pcd,
        ),
        Format::Obj => obj::write(&mut writer, geometry),
        Format::Stl => stl::write(&mut writer, geometry, &options.stl),
        Format::AsciiGrid => {
            let cloud = as_cloud_for_point_format(geometry, format)?;
            asciigrid::write(&mut writer, cloud)
        }

        #[cfg(feature = "dxf")]
        Format::Dxf => dxf::write(&mut writer, geometry),

        #[cfg(feature = "las")]
        Format::Las | Format::Laz => {
            let cloud = as_cloud_for_point_format(geometry, format)?;
            las::write(writer, cloud)
        }

        #[cfg(feature = "geospatial")]
        Format::GeoTiff | Format::Cog => {
            let cloud = as_cloud_for_point_format(geometry, format)?;
            geotiff::write(writer, cloud)
        }

        #[cfg(feature = "geospatial")]
        Format::GeoJson => {
            let cloud = as_cloud_for_point_format(geometry, format)?;
            geojson::write(writer, cloud)
        }

        _ => Err(Error::unsupported(format, "write", format.adapter_hint())),
    }
}

fn as_cloud_for_point_format(
    geometry: &Geometry,
    format: Format,
) -> Result<&crate::types::PointCloud> {
    match geometry {
        Geometry::PointCloud(cloud) => Ok(cloud),
        Geometry::Mesh(_) => Err(Error::LossyConversionBlocked {
            from: "mesh",
            to: format,
            reason: "the destination is a point-cloud format and cannot preserve faces".to_string(),
        }),
    }
}

#[cold]
#[inline(never)]
fn numeric_parse_error(format: Format, line: usize, name: &str, value: &str) -> Error {
    Error::parse(
        format,
        line,
        format!("expected numeric {name}, got '{value}'"),
    )
}

#[inline]
pub(crate) fn parse_f64(format: Format, line: usize, name: &str, value: &str) -> Result<f64> {
    value
        .parse::<f64>()
        .map_err(|_| numeric_parse_error(format, line, name, value))
}

#[inline]
pub(crate) fn parse_f32(format: Format, line: usize, name: &str, value: &str) -> Result<f32> {
    value
        .parse::<f32>()
        .map_err(|_| numeric_parse_error(format, line, name, value))
}

#[cold]
#[inline(never)]
fn range_parse_error(format: Format, line: usize, name: &str, value: &str, limit: &str) -> Error {
    Error::parse(
        format,
        line,
        format!("expected {name} in range {limit}, got '{value}'"),
    )
}

#[inline]
pub(crate) fn parse_u8(format: Format, line: usize, name: &str, value: &str) -> Result<u8> {
    if let Ok(v) = value.parse::<u8>() {
        return Ok(v);
    }
    let as_float = parse_f64(format, line, name, value)?;
    if as_float.fract() == 0.0 && (0.0..=u8::MAX as f64).contains(&as_float) {
        Ok(as_float as u8)
    } else {
        Err(range_parse_error(format, line, name, value, "0..255"))
    }
}

#[inline]
pub(crate) fn parse_u16(format: Format, line: usize, name: &str, value: &str) -> Result<u16> {
    if let Ok(v) = value.parse::<u16>() {
        return Ok(v);
    }
    let as_float = parse_f64(format, line, name, value)?;
    if as_float.fract() == 0.0 && (0.0..=u16::MAX as f64).contains(&as_float) {
        Ok(as_float as u16)
    } else {
        Err(range_parse_error(format, line, name, value, "0..65535"))
    }
}

#[inline]
pub(crate) fn write_fmt_f64<W: std::io::Write>(
    writer: &mut W,
    value: f64,
    precision: usize,
) -> std::io::Result<()> {
    if value == 0.0 {
        write!(writer, "{:.*}", precision, 0.0)
    } else {
        write!(writer, "{:.*}", precision, value)
    }
}

#[inline]
pub(crate) fn fmt_f64(value: f64, precision: usize) -> String {
    if value == 0.0 {
        // Avoid writing -0.000000 after transforms/triangulation.
        return format!("{:.*}", precision, 0.0);
    }
    format!("{:.*}", precision, value)
}

#[inline]
pub(crate) fn write_f32_le<W: std::io::Write>(writer: &mut W, value: f32) -> Result<()> {
    writer.write_all(&value.to_le_bytes())?;
    Ok(())
}

#[inline]
pub(crate) fn write_f64_le<W: std::io::Write>(writer: &mut W, value: f64) -> Result<()> {
    writer.write_all(&value.to_le_bytes())?;
    Ok(())
}

#[inline]
pub(crate) fn write_u16_le<W: std::io::Write>(writer: &mut W, value: u16) -> Result<()> {
    writer.write_all(&value.to_le_bytes())?;
    Ok(())
}

#[inline]
pub(crate) fn write_u32_le<W: std::io::Write>(writer: &mut W, value: u32) -> Result<()> {
    writer.write_all(&value.to_le_bytes())?;
    Ok(())
}

#[inline]
pub(crate) fn read_exact<const N: usize, R: std::io::Read>(reader: &mut R) -> Result<[u8; N]> {
    let mut bytes = [0_u8; N];
    reader.read_exact(&mut bytes)?;
    Ok(bytes)
}

#[inline]
pub(crate) fn read_f32_le<R: std::io::Read>(reader: &mut R) -> Result<f32> {
    Ok(f32::from_le_bytes(read_exact(reader)?))
}

#[inline]
pub(crate) fn read_f64_le<R: std::io::Read>(reader: &mut R) -> Result<f64> {
    Ok(f64::from_le_bytes(read_exact(reader)?))
}

#[inline]
pub(crate) fn read_u16_le<R: std::io::Read>(reader: &mut R) -> Result<u16> {
    Ok(u16::from_le_bytes(read_exact(reader)?))
}

#[inline]
pub(crate) fn read_u32_le<R: std::io::Read>(reader: &mut R) -> Result<u32> {
    Ok(u32::from_le_bytes(read_exact(reader)?))
}
