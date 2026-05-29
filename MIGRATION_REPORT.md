# Migration / crate design report

## Source requirement summary

The requested crate targets a broad LiDAR ecosystem:

- Main point-cloud formats: LAS, LAZ, COPC, E57, PLY, PCD, XYZ/TXT/CSV, PTS/PTX,
  Autodesk RCP/RCS, Potree, Entwine/EPT, and database containers.
- Raw sensor / robotics formats: UDP packets, PCAP, vendor raw/project formats,
  ROS bag/ROS 2 bag, and runtime PointCloud2.
- Derived products: GeoTIFF, COG, ASCII Grid, NetCDF/HDF5, vector features,
  GeoJSON/Shapefile/GPKG.
- LiDAR-derived meshes/CAD/DCC formats: OBJ, FBX, glTF/GLB, STL, DXF/DWG.

This is not one homogeneous data model: the list spans raw packets, point clouds,
raster products, vector features, web tile hierarchies, triangle meshes, CAD
projects, and robotics message streams. The crate therefore represents all listed
formats but implements only safe, well-scoped native conversions where semantics
can be preserved without external SDKs or policy decisions.

## Rust design decisions

### Core data model

- `Point` stores `Vec3 { x, y, z }` as `f64` plus optional intensity, 16-bit RGB,
  classification, return metadata, GPS time, scan angle, normal, and dynamic
  attributes.
- `PointCloud` owns a `Vec<Point>` plus metadata.
- `Mesh` owns vertices and triangle faces. Faces are zero-based internally.
- `Geometry` is an enum over `PointCloud` and `Mesh` so PLY/OBJ can preserve faces.
- `Metadata` carries CRS WKT, scanner transform, comments, warnings, point-count
  hints, source format, and adapter-specific attributes.

### Error and loss policy

- All fallible APIs return `Result<T, Error>`.
- Unsupported heavyweight formats return `Error::UnsupportedFormat` with a
  format-specific adapter hint.
- Mesh-to-point conversion is blocked by default because it discards faces.
- Point-cloud-to-STL is blocked because STL requires triangles and meshing is an
  algorithmic stage, not a format conversion.

### Native codec choices

Native codecs are dependency-light and implemented in safe Rust:

- XYZ/TXT/CSV: flexible delimited reader/writer with header autodetection.
- PTS/PTX: common terrestrial scanner text exports, including PTX transform.
- PLY: ASCII and binary little-endian point clouds/meshes.
- PCD: PCD 0.7 ASCII and binary; `binary_compressed` intentionally not built in.
- OBJ: vertices and faces; polygons fan-triangulated; vertex colors supported via
  common non-standard `v x y z r g b` extension.
- STL: ASCII and binary triangle mesh I/O.

### Adapter-ready formats

Adapters should implement `adapters::Codec` for:

- LAS/LAZ with the `las` crate.
- COPC with `copc-rs` or PDAL.
- E57 with the `e57` crate.
- GeoTIFF/COG/ASCII Grid through a rasterization policy plus GDAL/tiff adapter.
- ROS/PCAP/vendor raw through message/packet decoders, calibration, and frame
  transforms.
- CAD/DCC formats through specialized mesh/CAD libraries.

## Important semantic mappings

| Concept | Rust representation | Notes |
|---|---|---|
| XYZ coordinates | `Vec3` / `f64` | Avoids silent precision loss. |
| LAS/E57-style RGB | `Color { u16, u16, u16 }` | PLY/PCD writers preserve 16-bit where possible. |
| Point classification | `Option<u8>` | Matches common LAS classification range. |
| Normals | `Option<Vec3>` | Preserved by PLY/PCD/OBJ when available. |
| Mesh faces | `Face { indices: [usize; 3] }` | Internal zero-based, converted for OBJ. |
| Scanner pose | `Metadata::scanner_transform` | PTX read/write support. |
| CRS | `Metadata::crs_wkt` | Native text/PLY/PCD generally do not invent CRS. |
| Unknown fields | `Point::attributes` and `Metadata::attributes` | Adapter extension point. |

## Behavior that could not be exactly matched natively

- LAS/LAZ/COPC require LAS headers, VLRs/EVLRs, scales/offsets, CRS records,
  compression, COPC hierarchy, and potentially waveform/extras handling.
- E57 can contain multiple scans, scanner poses, images, and extensions. A single
  flat `PointCloud` may be insufficient without an adapter policy.
- Raster formats require interpolation/gridding/aggregation choices.
- ROS/PCAP/vendor packets require calibration, timing, topic/frame selection, and
  device-specific decoding.
- FBX/glTF/DXF/DWG/RCP/RCS have broad CAD/DCC/vendor semantics beyond a portable
  core crate.

## Unsafe code

No `unsafe` blocks are used.

## Testing strategy

Included tests cover:

- Format detection, including compound extensions.
- CSV header and attribute roundtrip.
- PLY point-cloud roundtrip.
- PCD point-cloud roundtrip.
- OBJ polygon triangulation.
- STL binary mesh roundtrip.
- High-level XYZ-to-PLY file conversion.

Recommended additional project validation:

1. Run golden conversions against small known outputs from existing C++/vendor
   tools.
2. Compare point count, bounds, exact integer attributes, color ranges, and face
   counts.
3. Use tolerance-based comparisons for floating-point coordinates.
4. Validate large data through streaming adapters before production use.

## Performance considerations

- The core native API is in-memory and best suited for small to medium data,
  tests, and adapter boundary conversions.
- Writers avoid repeated per-point schema computation.
- Open formats with complex compression or tiling are intentionally delegated to
  dedicated crates/adapters.
- Large LAS/COPC/E57 pipelines should stream records and reuse buffers rather than
  materializing all points.

## Assumptions

- Coordinate origin and CRS are input-defined. The crate does not transform CRS.
- PLY binary big-endian is uncommon and not implemented.
- PCD binary is interpreted as little-endian, matching common PCL workflows.
- PTS/PTX variants differ across scanners; the native reader supports common
  export patterns and surfaces mismatched counts as warnings.
- OBJ vertex color is a common extension, not part of the strict original OBJ
  material model.
