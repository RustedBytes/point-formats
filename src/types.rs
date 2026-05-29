use crate::format::Format;
use std::collections::BTreeMap;

/// Three-dimensional vector or coordinate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    #[inline]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    #[inline]
    #[allow(clippy::should_implement_trait)]
    pub fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }

    #[inline]
    pub fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    #[inline]
    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    #[inline]
    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    #[inline]
    pub fn normalized(self) -> Option<Self> {
        let norm = self.norm();
        if norm == 0.0 || !norm.is_finite() {
            None
        } else {
            Some(Self::new(self.x / norm, self.y / norm, self.z / norm))
        }
    }
}

/// RGB color stored as 16-bit components so LAS/E57 style precision is not
/// discarded when passing through richer formats.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Color {
    pub red: u16,
    pub green: u16,
    pub blue: u16,
}

impl Color {
    #[inline]
    pub const fn new(red: u16, green: u16, blue: u16) -> Self {
        Self { red, green, blue }
    }

    #[inline]
    pub const fn from_u8(red: u8, green: u8, blue: u8) -> Self {
        Self {
            red: red as u16,
            green: green as u16,
            blue: blue as u16,
        }
    }

    #[inline]
    pub fn to_u8_lossy(self) -> [u8; 3] {
        [
            self.red.min(255) as u8,
            self.green.min(255) as u8,
            self.blue.min(255) as u8,
        ]
    }

    #[inline]
    pub fn to_unit_rgb(self) -> [f64; 3] {
        [
            self.red as f64 / u16::MAX as f64,
            self.green as f64 / u16::MAX as f64,
            self.blue as f64 / u16::MAX as f64,
        ]
    }

    #[inline]
    pub fn from_unit_rgb(red: f64, green: f64, blue: f64) -> Option<Self> {
        fn component(v: f64) -> Option<u16> {
            if !v.is_finite() || !(0.0..=1.0).contains(&v) {
                return None;
            }
            Some((v * u16::MAX as f64).round() as u16)
        }
        Some(Self::new(
            component(red)?,
            component(green)?,
            component(blue)?,
        ))
    }
}

/// Dynamic per-point attribute retained by adapters that need fields outside
/// the normalized point model.
#[derive(Debug, Clone, PartialEq)]
pub enum AttributeValue {
    Int(i64),
    UInt(u64),
    Float(f64),
    Text(String),
}

/// Normalized LiDAR/point-cloud point.
#[derive(Debug, Clone, PartialEq)]
pub struct Point {
    pub position: Vec3,
    pub intensity: Option<f32>,
    pub color: Option<Color>,
    pub classification: Option<u8>,
    pub return_number: Option<u8>,
    pub number_of_returns: Option<u8>,
    pub gps_time: Option<f64>,
    pub scan_angle: Option<f32>,
    pub normal: Option<Vec3>,
    pub attributes: BTreeMap<String, AttributeValue>,
}

impl Point {
    #[inline]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            position: Vec3::new(x, y, z),
            intensity: None,
            color: None,
            classification: None,
            return_number: None,
            number_of_returns: None,
            gps_time: None,
            scan_angle: None,
            normal: None,
            attributes: BTreeMap::new(),
        }
    }

    #[inline]
    pub fn with_intensity(mut self, intensity: f32) -> Self {
        self.intensity = Some(intensity);
        self
    }

    #[inline]
    pub fn with_color(mut self, color: Color) -> Self {
        self.color = Some(color);
        self
    }

    #[inline]
    pub fn with_classification(mut self, classification: u8) -> Self {
        self.classification = Some(classification);
        self
    }

    #[inline]
    pub fn with_normal(mut self, normal: Vec3) -> Self {
        self.normal = Some(normal);
        self
    }
}

/// Axis-aligned bounds for a point cloud or mesh.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds3 {
    pub min: Vec3,
    pub max: Vec3,
}

impl Bounds3 {
    #[inline]
    pub fn empty() -> Self {
        Self {
            min: Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            max: Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        }
    }

    pub fn from_points<'a>(points: impl IntoIterator<Item = &'a Point>) -> Option<Self> {
        let mut bounds = Self::empty();
        let mut any = false;
        for point in points {
            bounds.include(point.position);
            any = true;
        }
        any.then_some(bounds)
    }

    pub fn from_vertices<'a>(vertices: impl IntoIterator<Item = &'a Vertex>) -> Option<Self> {
        let mut bounds = Self::empty();
        let mut any = false;
        for vertex in vertices {
            bounds.include(vertex.position);
            any = true;
        }
        any.then_some(bounds)
    }

    #[inline]
    pub fn include(&mut self, p: Vec3) {
        self.min.x = self.min.x.min(p.x);
        self.min.y = self.min.y.min(p.y);
        self.min.z = self.min.z.min(p.z);
        self.max.x = self.max.x.max(p.x);
        self.max.y = self.max.y.max(p.y);
        self.max.z = self.max.z.max(p.z);
    }
}

/// Metadata shared by point clouds and meshes. The crate stores CRS and scanner
/// transforms but does not invent them for formats that do not carry them.
#[derive(Debug, Clone, PartialEq)]
#[derive(Default)]
pub struct Metadata {
    pub source_format: Option<Format>,
    pub point_count_hint: Option<usize>,
    pub crs_wkt: Option<String>,
    pub scanner_transform: Option<[[f64; 4]; 4]>,
    pub comments: Vec<String>,
    pub warnings: Vec<String>,
    pub attributes: BTreeMap<String, AttributeValue>,
}


/// Owned point cloud. Suitable for moderate-size conversion and tests. Large
/// production LAS/COPC/E57 adapters should implement streaming codecs using the
/// adapter traits in [`crate::adapters`].
#[derive(Debug, Clone, PartialEq)]
pub struct PointCloud {
    pub points: Vec<Point>,
    pub metadata: Metadata,
}

impl PointCloud {
    pub fn new(points: Vec<Point>) -> Self {
        Self {
            points,
            metadata: Metadata::default(),
        }
    }

    pub fn empty() -> Self {
        Self::new(Vec::new())
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn bounds(&self) -> Option<Bounds3> {
        Bounds3::from_points(&self.points)
    }

    pub fn has_color(&self) -> bool {
        self.points.iter().any(|p| p.color.is_some())
    }

    pub fn has_intensity(&self) -> bool {
        self.points.iter().any(|p| p.intensity.is_some())
    }

    pub fn has_classification(&self) -> bool {
        self.points.iter().any(|p| p.classification.is_some())
    }

    pub fn has_gps_time(&self) -> bool {
        self.points.iter().any(|p| p.gps_time.is_some())
    }

    pub fn has_normals(&self) -> bool {
        self.points.iter().any(|p| p.normal.is_some())
    }
}

/// Mesh vertex. The crate keeps optional color/normal because PLY/OBJ can carry
/// them, while STL only stores per-facet normals.
#[derive(Debug, Clone, PartialEq)]
pub struct Vertex {
    pub position: Vec3,
    pub normal: Option<Vec3>,
    pub color: Option<Color>,
}

impl Vertex {
    #[inline]
    pub fn new(position: Vec3) -> Self {
        Self {
            position,
            normal: None,
            color: None,
        }
    }
}

/// Triangle face using zero-based vertex indices.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Face {
    pub indices: [usize; 3],
}

impl Face {
    #[inline]
    pub const fn new(a: usize, b: usize, c: usize) -> Self {
        Self { indices: [a, b, c] }
    }
}

/// Triangle mesh.
#[derive(Debug, Clone, PartialEq)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub faces: Vec<Face>,
    pub metadata: Metadata,
}

impl Mesh {
    #[inline]
    pub fn new(vertices: Vec<Vertex>, faces: Vec<Face>) -> Self {
        Self {
            vertices,
            faces,
            metadata: Metadata::default(),
        }
    }

    #[inline]
    pub fn bounds(&self) -> Option<Bounds3> {
        Bounds3::from_vertices(&self.vertices)
    }

    pub fn vertex_cloud(&self) -> PointCloud {
        let points = self
            .vertices
            .iter()
            .map(|vertex| {
                let mut point = Point::new(vertex.position.x, vertex.position.y, vertex.position.z);
                point.normal = vertex.normal;
                point.color = vertex.color;
                point
            })
            .collect();
        let mut metadata = self.metadata.clone();
        metadata.warnings.push(
            "mesh faces were discarded while converting vertices to a point cloud".to_string(),
        );
        PointCloud { points, metadata }
    }
}

/// Geometry returned by readers. Formats like PLY/OBJ can be either a point
/// cloud or mesh depending on whether face data is present.
#[derive(Debug, Clone, PartialEq)]
pub enum Geometry {
    PointCloud(PointCloud),
    Mesh(Mesh),
}

impl Geometry {
    pub fn point_count(&self) -> usize {
        match self {
            Self::PointCloud(cloud) => cloud.points.len(),
            Self::Mesh(mesh) => mesh.vertices.len(),
        }
    }

    pub fn face_count(&self) -> usize {
        match self {
            Self::PointCloud(_) => 0,
            Self::Mesh(mesh) => mesh.faces.len(),
        }
    }

    pub fn metadata(&self) -> &Metadata {
        match self {
            Self::PointCloud(cloud) => &cloud.metadata,
            Self::Mesh(mesh) => &mesh.metadata,
        }
    }

    pub fn metadata_mut(&mut self) -> &mut Metadata {
        match self {
            Self::PointCloud(cloud) => &mut cloud.metadata,
            Self::Mesh(mesh) => &mut mesh.metadata,
        }
    }
}
