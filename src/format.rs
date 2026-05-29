use crate::error::{Error, Result};
use std::fmt;
use std::path::{Path, PathBuf};
use std::str::FromStr;

/// High-level family of a format. This is used to decide when conversion
/// requires semantic work such as meshing, rasterization, or decoding packets.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FormatFamily {
    PointCloud,
    Mesh,
    Raster,
    Vector,
    Database,
    RoboticsStream,
    SensorRaw,
    WebTiles,
    VendorProject,
}

/// Declares whether a format is implemented natively in this crate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FormatSupport {
    NativeReadWrite,
    NativeReadOnly,
    NativeWriteOnly,
    AdapterRequired,
    MetadataOnly,
}

/// Formats from the supplied LiDAR / point-cloud list.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[allow(missing_docs)]
pub enum Format {
    Las,
    Laz,
    Copc,
    E57,
    Ply,
    Pcd,
    Xyz,
    Txt,
    Csv,
    Pts,
    Ptx,
    Rcp,
    Rcs,
    Potree,
    Ept,
    GeoTiff,
    Cog,
    AsciiGrid,
    NetCdf,
    Hdf5,
    Shapefile,
    GeoJson,
    Gpkg,
    Obj,
    Fbx,
    Gltf,
    Glb,
    Stl,
    Dxf,
    Dwg,
    Pcap,
    UdpPackets,
    VendorRaw,
    RosBag,
    Ros2Bag,
    PointCloud2,
}

impl Format {
    /// Infers a format from a path's extension and known compound extensions.
    pub fn from_path(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        Self::from_path_opt(path).ok_or_else(|| Error::UnknownFormat {
            path: PathBuf::from(path),
        })
    }

    /// Same as [`Format::from_path`], but returns `None` instead of an error.
    pub fn from_path_opt(path: impl AsRef<Path>) -> Option<Self> {
        let path = path.as_ref();
        let name = path.file_name()?.to_string_lossy().to_ascii_lowercase();
        if name.ends_with(".copc.laz") {
            return Some(Self::Copc);
        }
        if name.ends_with(".cog.tif") || name.ends_with(".cog.tiff") {
            return Some(Self::Cog);
        }
        if name.ends_with(".tar.gz") {
            return None;
        }

        let ext = path.extension()?.to_string_lossy().to_ascii_lowercase();
        match ext.as_str() {
            "las" => Some(Self::Las),
            "laz" => Some(Self::Laz),
            "copc" => Some(Self::Copc),
            "e57" => Some(Self::E57),
            "ply" => Some(Self::Ply),
            "pcd" => Some(Self::Pcd),
            "xyz" => Some(Self::Xyz),
            "txt" => Some(Self::Txt),
            "csv" => Some(Self::Csv),
            "pts" => Some(Self::Pts),
            "ptx" => Some(Self::Ptx),
            "rcp" => Some(Self::Rcp),
            "rcs" => Some(Self::Rcs),
            "potree" => Some(Self::Potree),
            "ept" => Some(Self::Ept),
            "tif" | "tiff" => Some(Self::GeoTiff),
            "asc" => Some(Self::AsciiGrid),
            "nc" | "cdf" | "netcdf" => Some(Self::NetCdf),
            "h5" | "hdf5" => Some(Self::Hdf5),
            "shp" => Some(Self::Shapefile),
            "geojson" | "json" => Some(Self::GeoJson),
            "gpkg" => Some(Self::Gpkg),
            "obj" => Some(Self::Obj),
            "fbx" => Some(Self::Fbx),
            "gltf" => Some(Self::Gltf),
            "glb" => Some(Self::Glb),
            "stl" => Some(Self::Stl),
            "dxf" => Some(Self::Dxf),
            "dwg" => Some(Self::Dwg),
            "pcap" | "pcapng" => Some(Self::Pcap),
            "bag" => Some(Self::RosBag),
            "db3" => Some(Self::Ros2Bag),
            _ => None,
        }
    }

    /// Stable lowercase name used by the CLI and diagnostics.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Las => "las",
            Self::Laz => "laz",
            Self::Copc => "copc",
            Self::E57 => "e57",
            Self::Ply => "ply",
            Self::Pcd => "pcd",
            Self::Xyz => "xyz",
            Self::Txt => "txt",
            Self::Csv => "csv",
            Self::Pts => "pts",
            Self::Ptx => "ptx",
            Self::Rcp => "rcp",
            Self::Rcs => "rcs",
            Self::Potree => "potree",
            Self::Ept => "ept",
            Self::GeoTiff => "geotiff",
            Self::Cog => "cog",
            Self::AsciiGrid => "ascii-grid",
            Self::NetCdf => "netcdf",
            Self::Hdf5 => "hdf5",
            Self::Shapefile => "shapefile",
            Self::GeoJson => "geojson",
            Self::Gpkg => "gpkg",
            Self::Obj => "obj",
            Self::Fbx => "fbx",
            Self::Gltf => "gltf",
            Self::Glb => "glb",
            Self::Stl => "stl",
            Self::Dxf => "dxf",
            Self::Dwg => "dwg",
            Self::Pcap => "pcap",
            Self::UdpPackets => "udp-packets",
            Self::VendorRaw => "vendor-raw",
            Self::RosBag => "ros-bag",
            Self::Ros2Bag => "ros2-bag",
            Self::PointCloud2 => "pointcloud2",
        }
    }

    /// Format family.
    pub const fn family(self) -> FormatFamily {
        match self {
            Self::Las
            | Self::Laz
            | Self::Copc
            | Self::E57
            | Self::Ply
            | Self::Pcd
            | Self::Xyz
            | Self::Txt
            | Self::Csv
            | Self::Pts
            | Self::Ptx => FormatFamily::PointCloud,
            Self::Obj | Self::Fbx | Self::Gltf | Self::Glb | Self::Stl | Self::Dxf | Self::Dwg => {
                FormatFamily::Mesh
            }
            Self::GeoTiff | Self::Cog | Self::AsciiGrid | Self::NetCdf | Self::Hdf5 => {
                FormatFamily::Raster
            }
            Self::Shapefile | Self::GeoJson => FormatFamily::Vector,
            Self::Gpkg => FormatFamily::Database,
            Self::RosBag | Self::Ros2Bag | Self::PointCloud2 => FormatFamily::RoboticsStream,
            Self::Pcap | Self::UdpPackets | Self::VendorRaw => FormatFamily::SensorRaw,
            Self::Potree | Self::Ept => FormatFamily::WebTiles,
            Self::Rcp | Self::Rcs => FormatFamily::VendorProject,
        }
    }

    /// Native support level in this crate.
    pub const fn support(self) -> FormatSupport {
        match self {
            Self::Ply
            | Self::Pcd
            | Self::Xyz
            | Self::Txt
            | Self::Csv
            | Self::Pts
            | Self::Ptx
            | Self::Obj
            | Self::Stl => FormatSupport::NativeReadWrite,
            Self::Las
            | Self::Laz
            | Self::Copc
            | Self::E57
            | Self::GeoTiff
            | Self::Cog
            | Self::AsciiGrid
            | Self::NetCdf
            | Self::Hdf5
            | Self::Shapefile
            | Self::GeoJson
            | Self::Gpkg
            | Self::Fbx
            | Self::Gltf
            | Self::Glb
            | Self::Dxf
            | Self::Dwg
            | Self::Potree
            | Self::Ept
            | Self::Pcap
            | Self::UdpPackets
            | Self::VendorRaw
            | Self::RosBag
            | Self::Ros2Bag
            | Self::PointCloud2
            | Self::Rcp
            | Self::Rcs => FormatSupport::AdapterRequired,
        }
    }

    /// Returns true when the format can be read by the built-in codecs.
    pub const fn is_native_read(self) -> bool {
        matches!(
            self.support(),
            FormatSupport::NativeReadWrite | FormatSupport::NativeReadOnly
        )
    }

    /// Returns true when the format can be written by the built-in codecs.
    pub const fn is_native_write(self) -> bool {
        matches!(
            self.support(),
            FormatSupport::NativeReadWrite | FormatSupport::NativeWriteOnly
        )
    }

    /// Human-readable reason when this format needs an adapter.
    pub const fn adapter_hint(self) -> &'static str {
        match self {
            Self::Las | Self::Laz => "use an adapter built on the `las` crate; enable LAZ through its laz/laz-parallel features when writing compressed files",
            Self::Copc => "use an adapter built on `copc-rs` or a PDAL pipeline; COPC requires LAZ hierarchy/index handling",
            Self::E57 => "use an adapter built on the `e57` crate; E57 can contain multiple scans, poses, images, and vendor extensions",
            Self::GeoTiff | Self::Cog | Self::AsciiGrid => "raster products need an explicit gridding/rasterization policy and a GDAL/tiff adapter",
            Self::NetCdf | Self::Hdf5 => "scientific containers need dataset/schema selection and a netcdf/hdf5 adapter",
            Self::Shapefile | Self::GeoJson | Self::Gpkg => "vector/database products need an explicit feature extraction schema and GIS adapter",
            Self::Fbx | Self::Gltf | Self::Glb | Self::Dxf | Self::Dwg => "DCC/CAD formats need a mesh/CAD adapter and may not preserve point attributes",
            Self::Potree | Self::Ept => "web tile formats need tiling, indexing, and hierarchy generation",
            Self::Pcap | Self::UdpPackets | Self::VendorRaw => "raw sensor data must be decoded using vendor packet calibration before point-cloud export",
            Self::RosBag | Self::Ros2Bag | Self::PointCloud2 => "robotics streams need ROS message schemas, topic selection, frame transforms, and timestamp policy",
            Self::Rcp | Self::Rcs => "Autodesk project formats are proprietary/vendor-specific; use vendor/export tooling or an adapter",
            _ => "format is supported natively",
        }
    }

    /// All formats represented by this crate.
    pub const ALL: &'static [Self] = &[
        Self::Las,
        Self::Laz,
        Self::Copc,
        Self::E57,
        Self::Ply,
        Self::Pcd,
        Self::Xyz,
        Self::Txt,
        Self::Csv,
        Self::Pts,
        Self::Ptx,
        Self::Rcp,
        Self::Rcs,
        Self::Potree,
        Self::Ept,
        Self::GeoTiff,
        Self::Cog,
        Self::AsciiGrid,
        Self::NetCdf,
        Self::Hdf5,
        Self::Shapefile,
        Self::GeoJson,
        Self::Gpkg,
        Self::Obj,
        Self::Fbx,
        Self::Gltf,
        Self::Glb,
        Self::Stl,
        Self::Dxf,
        Self::Dwg,
        Self::Pcap,
        Self::UdpPackets,
        Self::VendorRaw,
        Self::RosBag,
        Self::Ros2Bag,
        Self::PointCloud2,
    ];
}

impl fmt::Display for Format {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

impl FromStr for Format {
    type Err = String;

    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        let normalized = s
            .trim()
            .to_ascii_lowercase()
            .replace('_', "-")
            .replace('.', "");
        match normalized.as_str() {
            "las" => Ok(Self::Las),
            "laz" => Ok(Self::Laz),
            "copc" | "copclaz" => Ok(Self::Copc),
            "e57" => Ok(Self::E57),
            "ply" => Ok(Self::Ply),
            "pcd" => Ok(Self::Pcd),
            "xyz" => Ok(Self::Xyz),
            "txt" => Ok(Self::Txt),
            "csv" => Ok(Self::Csv),
            "pts" => Ok(Self::Pts),
            "ptx" => Ok(Self::Ptx),
            "rcp" => Ok(Self::Rcp),
            "rcs" => Ok(Self::Rcs),
            "potree" => Ok(Self::Potree),
            "ept" => Ok(Self::Ept),
            "geotiff" | "tif" | "tiff" => Ok(Self::GeoTiff),
            "cog" => Ok(Self::Cog),
            "ascii-grid" | "asc" | "asciigrid" => Ok(Self::AsciiGrid),
            "netcdf" | "nc" => Ok(Self::NetCdf),
            "hdf5" | "h5" => Ok(Self::Hdf5),
            "shapefile" | "shp" => Ok(Self::Shapefile),
            "geojson" => Ok(Self::GeoJson),
            "gpkg" | "geopackage" => Ok(Self::Gpkg),
            "obj" => Ok(Self::Obj),
            "fbx" => Ok(Self::Fbx),
            "gltf" => Ok(Self::Gltf),
            "glb" => Ok(Self::Glb),
            "stl" => Ok(Self::Stl),
            "dxf" => Ok(Self::Dxf),
            "dwg" => Ok(Self::Dwg),
            "pcap" | "pcapng" => Ok(Self::Pcap),
            "udp" | "udp-packets" | "udppackets" => Ok(Self::UdpPackets),
            "vendor-raw" | "vendorraw" | "raw" => Ok(Self::VendorRaw),
            "ros-bag" | "rosbag" | "bag" => Ok(Self::RosBag),
            "ros2-bag" | "ros2bag" | "db3" => Ok(Self::Ros2Bag),
            "pointcloud2" | "point-cloud2" | "sensor-msgs-pointcloud2" => Ok(Self::PointCloud2),
            _ => Err(format!("unknown format '{s}'")),
        }
    }
}
