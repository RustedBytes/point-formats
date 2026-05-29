# point-formats

[![Crates.io Version](https://img.shields.io/crates/v/point-formats)](https://crates.io/crates/point-formats)

`point-formats` is a Rust crate and CLI for converting among LiDAR,
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

Natively supported formats (compiled in by default or enabled via Cargo features):

| Format | Extension | Feature Flag | Read | Write | Notes |
|---|---|---|---:|---:|---|
| XYZ / TXT | `.xyz`, `.txt` | *(default)* | yes | yes | Whitespace text points; optional columns through `DelimitedOptions`. |
| CSV | `.csv` | *(default)* | yes | yes | Header autodetection; configurable column mapping. |
| PTS | `.pts` | *(default)* | yes | yes | Handles common count-first terrestrial scanner exports. |
| PTX | `.ptx` | *(default)* | yes | yes | Preserves PTX scanner transform metadata when present. |
| PLY | `.ply` | *(default)* | yes | yes | ASCII and binary little-endian; point clouds and triangle meshes. |
| PCD | `.pcd` | *(default)* | yes | yes | PCD 0.7 ASCII and binary; recovers XYZ, intensity, colors, normals. |
| OBJ | `.obj` | *(default)* | yes | yes | Vertex-only point clouds and triangle meshes; fan triangulation. |
| STL | `.stl` | *(default)* | yes | yes | ASCII and binary triangle meshes; points require meshing first. |
| ASCII Grid | `.asc` | *(default)* | yes | yes | Native grid raster format; converts elevation grids. |
| LAS / LAZ | `.las`, `.laz` | `las` / `laz` | yes | yes | Industry standard point clouds; utilizes `las` crate. |
| COPC | `.copc.laz` | `copc` | yes | no | Cloud Optimized Point Cloud; utilizes `copc-rs`. |
| E57 | `.e57` | `e57` | yes | yes | ASTM E57 format; utilizes `e57` crate. |
| GeoTIFF / COG | `.tif`, `.tiff`, `.cog` | `geospatial` | yes | yes | Geospatial elevation raster grids. |
| GeoJSON | `.geojson`, `.json` | `geospatial` | yes | yes | Geospatial JSON vector files mapping properties/geometries. |
| Shapefile | `.shp` | `shapefile` | yes | yes | Esri Shapefile vector and DBF attributes. |
| GeoPackage | `.gpkg` | `gpkg` | yes | yes | SQLite database spatial point features. |
| glTF / GLB | `.gltf`, `.glb` | `gltf` | yes | yes | 3D graphics transmission format; maps points and meshes. |
| DXF | `.dxf` | `dxf` | yes | yes | AutoCAD DXF format; maps points and Face3D meshes. |
| ROS 1 Bag | `.bag` | `robotics` | yes | yes | ROS 1 serialization/deserialization for PointCloud2 streams. |
| ROS 2 Bag | `.db3` | `robotics` | yes | yes | ROS 2 SQLite database CDR alignment-based serialization. |
| PointCloud2 | `.pc2`, `.pointcloud2` | `robotics` | yes | yes | Direct ROS/DDS PointCloud2 message format. |
| PCAP | `.pcap`, `.pcapng` | `sensor` | yes | yes | Raw network capture containing UDP point packets. |
| UdpPackets | `.udp`, `.udppackets` | `sensor` | yes | yes | Stream format containing length-prefixed raw UDP payloads. |
| VendorRaw | `.raw`, `.vendorraw` | `sensor` | yes | yes | High-performance flat binary point streams. |

## Represented but adapter-required

The `Format` enum also represents formats that require heavy external SDKs, specialized desktop applications, or complex user-defined pipelines. Built-in conversions for these return `Error::UnsupportedFormat` with a format-specific hint instead of silently dropping data:

- **RCP / RCS**: Autodesk proprietary scan project formats.
- **Potree / EPT**: Large-scale web-tiled point cloud octree systems.
- **NetCDF / HDF5**: Scientific dataset containers that need database/mesh mapping policies.
- **FBX**: Filmbox proprietary 3D asset interchange format.
- **DWG**: AutoCAD proprietary design drawing format.

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
    asciigrid.rs    # ASCII Grid
    geojson.rs      # GeoJSON
    geotiff.rs      # GeoTIFF / COG
    shapefile.rs    # Shapefile
    gpkg.rs         # GeoPackage
    gltf.rs         # glTF / GLB
    dxf.rs          # DXF
    robotics.rs     # ROS bags, ROS 2 SQLite bags, PointCloud2 messages
    sensor.rs       # PCAP, UdpPackets, VendorRaw
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

MIT.
