# lidar-format-convert

`lidar-format-convert` is a Rust crate and CLI for converting among LiDAR,
point-cloud, and simple mesh formats with a preservation-first data model.

The crate is intentionally split into:

1. **Native codecs** implemented in dependency-light Rust for portable
   interchange formats.
2. **Adapter-ready formats** represented in the public API but requiring
   external libraries, vendor SDKs, ROS tooling, or an explicit rasterization /
   meshing policy.

This avoids pretending that LAS/COPC/E57/GeoTIFF/ROS bags/vendor packets are all
simple syntax translations. Those formats often carry CRS, scanner poses,
packet calibration, image attachments, tiling hierarchies, compression, or raster
semantics that must be handled deliberately.

## Native support in this crate

| Format | Read | Write | Notes |
|---|---:|---:|---|
| XYZ / TXT | yes | yes | Whitespace text points; optional columns through `DelimitedOptions`. |
| CSV | yes | yes | Header autodetection; configurable column mapping. |
| PTS | yes | yes | Handles common count-first terrestrial scanner exports. |
| PTX | yes | yes | Preserves PTX scanner transform metadata when present. |
| PLY | yes | yes | ASCII and binary little-endian; point clouds and triangle meshes; 16-bit RGB preservation. |
| PCD | yes | yes | PCD 0.7 ASCII and binary; arbitrary fields parsed enough to recover XYZ/intensity/color/class/normals. |
| OBJ | yes | yes | Vertex-only point clouds and triangle meshes; polygon faces triangulated by fan. |
| STL | yes | yes | ASCII and binary triangle meshes; point clouds require meshing before export. |

## Represented but adapter-required

The `Format` enum also represents LAS, LAZ, COPC, E57, RCP/RCS,
Potree/EPT, GeoTIFF/COG, ASCII Grid, NetCDF/HDF5, Shapefile/GeoJSON/GPKG,
FBX/glTF/GLB, DXF/DWG, PCAP/UDP/vendor raw, ROS bag/ROS 2 bag, and
PointCloud2. Built-in conversion returns `Error::UnsupportedFormat` with a
format-specific hint instead of silently dropping semantics.

Recommended adapter directions:

- LAS/LAZ: implement `adapters::Codec` using `las` with its LAZ feature flags.
- COPC: implement `adapters::Codec` using `copc-rs` or a PDAL-backed pipeline.
- E57: implement `adapters::Codec` using `e57`, preserving scan grouping and poses.
- GeoTIFF/COG/ASCII Grid: add a rasterization policy before writing rasters.
- ROS/PCAP/vendor raw: decode packets/messages first, including calibration,
  timestamps, frames, and topic selection.
- CAD/DCC formats: add mesh/CAD-specific adapters and document unsupported point
  attributes.

## CLI

```bash
cargo run --bin lidar-convert -- scan.xyz scan.ply
cargo run --bin lidar-convert -- scan.pcd scan.ply --binary-ply
cargo run --bin lidar-convert -- mesh.obj vertices.csv --allow-lossy
cargo run --bin lidar-convert -- --list-formats
```

## Library usage

```rust
use lidar_format_convert::{convert_path, ConvertOptions};

let report = convert_path("scan.xyz", "scan.ply", &ConvertOptions::default())?;
println!("wrote {} points", report.points_written);
# Ok::<(), lidar_format_convert::Error>(())
```

For in-memory use:

```rust
use lidar_format_convert::{Color, Geometry, Point, PointCloud};
use lidar_format_convert::io::{self, PlyEncoding};

let cloud = PointCloud::new(vec![
    Point::new(1.0, 2.0, 3.0).with_color(Color::new(255, 128, 0)),
]);
let geometry = Geometry::PointCloud(cloud);
let mut bytes = Vec::new();
let mut options = io::PlyOptions::default();
options.encoding = PlyEncoding::Ascii;
io::ply::write(&mut bytes, &geometry, &options)?;
# Ok::<(), lidar_format_convert::Error>(())
```

## Semantic design choices

- Coordinates are stored as `f64` to avoid losing precision when converting
  between text, LAS-style coordinates, PLY double properties, and PCD double
  fields.
- Colors are stored as `u16` RGB. This preserves LAS/E57-style 16-bit colors;
  formats that conventionally use `u8` can downscale explicitly.
- Faces use zero-based indices internally. OBJ converts to/from one-based and
  supports negative indices on read.
- PLY/OBJ polygon faces are triangulated by fan. This is deterministic but can
  alter non-convex polygons; use a geometry library if exact polygon semantics
  matter.
- STL stores only triangles and per-facet normals. Point cloud to STL conversion
  is blocked unless a separate meshing stage is performed.
- Mesh-to-point conversion is lossy because faces are discarded. The high-level
  converter refuses it unless `allow_lossy` is set.
- CRS, scanner transforms, comments, and warnings are retained in `Metadata` when
  a native format can carry or expose them. Formats without CRS do not invent one.

## Module structure

```text
src/
  lib.rs
  error.rs          # Error and Result
  format.rs         # Format enum, families, support metadata, detection
  types.rs          # Point, PointCloud, Mesh, Metadata, Geometry
  convert.rs        # High-level conversion API
  adapters/mod.rs   # Codec trait and registry for LAS/E57/COPC/etc.
  io/
    mod.rs          # Native options and dispatch
    delimited.rs    # XYZ/TXT/CSV
    pts.rs          # PTS
    ptx.rs          # PTX
    ply.rs          # PLY ASCII/binary little-endian
    pcd.rs          # PCD ASCII/binary
    obj.rs          # OBJ
    stl.rs          # STL ASCII/binary
  main.rs           # CLI
```

## Testing and validation

```bash
cargo fmt
cargo test
cargo clippy --all-targets --all-features -- -D warnings
```

Suggested validation against source systems:

1. Export a small known cloud to XYZ/CSV/PLY/PCD from the C++ or vendor tool.
2. Convert with this crate.
3. Compare point count, bounds, max coordinate error, intensity/color/class
   preservation, and face count where applicable.
4. Use exact comparisons for integer attributes and tolerances for floating
   coordinates.

Example tolerance strategy:

```rust
const EPS: f64 = 1e-9;
assert!((a.position.x - b.position.x).abs() <= EPS);
assert_eq!(a.color, b.color);
```

## Known limitations

- This is an in-memory native converter. Very large point clouds should use
  streaming adapters that implement `adapters::Codec`.
- Binary big-endian PLY and PCD `binary_compressed` are not implemented natively.
- Raster outputs need an explicit gridding/interpolation policy and are therefore
  adapter-required.
- Raw packets, ROS messages, and vendor project formats cannot be converted
  safely without calibration, schemas, frame transforms, and topic/stream choice.
- STL cannot represent point attributes, colors, classification, CRS, or point
  clouds.

## License

Dual-licensed under MIT or Apache-2.0.
