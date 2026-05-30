//! `point-formats` is a dependency-light crate for converting among
//! common LiDAR, point-cloud, and simple mesh interchange formats.
//!
//! The crate deliberately separates two concerns:
//!
//! * **Native codecs** for formats that can be implemented portably in safe Rust
//!   without large native libraries: XYZ/TXT/CSV, PTS/PTX, PLY, PCD, OBJ, and STL.
//! * **Adapter-ready formats** such as LAS/LAZ/COPC/E57/GeoTIFF/ROS bags/vendor
//!   packets, which are represented in the API and return explicit errors until
//!   a downstream adapter registers a codec.
//!
//! # Example
//!
//! ```no_run
//! use point_formats::{convert_path, ConvertOptions};
//!
//! let report = convert_path(
//!     "scan.xyz",
//!     "scan.ply",
//!     &ConvertOptions::default(),
//! )?;
//! println!("wrote {} points", report.points_written);
//! # Ok::<(), point_formats::Error>(())
//! ```

pub mod adapters;
pub mod convert;
pub mod error;
pub mod format;
pub mod io;
pub mod types;

pub use convert::{convert_path, ConversionReport, ConvertOptions, GeometryPolicy};
pub use error::{Error, Result};
pub use format::{Format, FormatFamily, FormatSupport};
pub use types::{
    AttributeValue, Bounds3, Color, Face, Geometry, Mesh, Metadata, Point, PointCloud, Vec3, Vertex,
};
