//! Adapter traits for heavyweight or vendor formats.
//!
//! The native crate keeps a small dependency surface. LAS/LAZ, COPC, E57,
//! GeoTIFF/COG, ROS bags, and vendor packet formats can be integrated by
//! implementing [`Codec`] and dispatching through your own registry or wrapper.

use crate::{ConvertOptions, Format, Geometry, Result};
use std::path::Path;

/// Read/write plugin for one or more formats.
pub trait Codec: Send + Sync {
    /// Returns true when this codec can read `format`.
    fn can_read(&self, format: Format) -> bool;

    /// Returns true when this codec can write `format`.
    fn can_write(&self, format: Format) -> bool;

    /// Reads geometry from `path`.
    fn read_path(&self, path: &Path, format: Format, options: &ConvertOptions) -> Result<Geometry>;

    /// Writes geometry to `path`.
    fn write_path(
        &self,
        path: &Path,
        format: Format,
        geometry: &Geometry,
        options: &ConvertOptions,
    ) -> Result<()>;
}

/// Static registry for custom codecs. It is intentionally not global so users
/// can avoid hidden state and configure codecs per pipeline.
#[derive(Default)]
pub struct CodecRegistry {
    codecs: Vec<Box<dyn Codec>>,
}

impl CodecRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_codec(mut self, codec: Box<dyn Codec>) -> Self {
        self.codecs.push(codec);
        self
    }

    pub fn register(&mut self, codec: Box<dyn Codec>) {
        self.codecs.push(codec);
    }

    pub fn reader(&self, format: Format) -> Option<&dyn Codec> {
        self.codecs
            .iter()
            .find(|codec| codec.can_read(format))
            .map(|codec| codec.as_ref())
    }

    pub fn writer(&self, format: Format) -> Option<&dyn Codec> {
        self.codecs
            .iter()
            .find(|codec| codec.can_write(format))
            .map(|codec| codec.as_ref())
    }
}
