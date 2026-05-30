use point_formats::{
    adapters::{Codec, CodecRegistry},
    AttributeValue, Bounds3, Color, Error, Face, Format, FormatSupport, Geometry, Mesh, Point,
    PointCloud, Vec3, Vertex,
};
use std::collections::BTreeMap;
use std::str::FromStr;

#[test]
fn test_vec3() {
    let z = Vec3::ZERO;
    assert_eq!(z.x, 0.0);
    assert_eq!(z.y, 0.0);
    assert_eq!(z.z, 0.0);

    let v1 = Vec3::new(1.0, 2.0, 3.0);
    assert!(v1.is_finite());
    assert!(!Vec3::new(f64::NAN, 2.0, 3.0).is_finite());

    let v2 = Vec3::new(4.0, 5.0, 6.0);
    let diff = v2.sub(v1);
    assert_eq!(diff, Vec3::new(3.0, 3.0, 3.0));

    let crossed = v1.cross(v2);
    assert_eq!(crossed, Vec3::new(-3.0, 6.0, -3.0));

    let dotted = v1.dot(v2);
    assert_eq!(dotted, 4.0 + 10.0 + 18.0);

    let v3 = Vec3::new(3.0, 4.0, 0.0);
    assert_eq!(v3.norm(), 5.0);

    let norm = v3.normalized().unwrap();
    assert_eq!(norm.x, 3.0 / 5.0);
    assert_eq!(norm.y, 4.0 / 5.0);
    assert_eq!(norm.z, 0.0);

    assert!(Vec3::ZERO.normalized().is_none());
    assert!(Vec3::new(f64::INFINITY, 0.0, 0.0).normalized().is_none());
}

#[test]
fn test_color() {
    let c = Color::new(100, 200, 300);
    assert_eq!(c.red, 100);
    assert_eq!(c.green, 200);
    assert_eq!(c.blue, 300);

    let c2 = Color::from_u8(10, 20, 30);
    assert_eq!(c2.red, 10);
    assert_eq!(c2.green, 20);
    assert_eq!(c2.blue, 30);

    assert_eq!(c.to_u8_lossy(), [100, 200, 255]);

    let unit = Color::new(0, u16::MAX / 2, u16::MAX).to_unit_rgb();
    assert_eq!(unit[0], 0.0);
    assert!((unit[1] - 0.5).abs() < 1e-4);
    assert_eq!(unit[2], 1.0);

    let from_unit = Color::from_unit_rgb(0.0, 0.5, 1.0).unwrap();
    assert_eq!(from_unit.red, 0);
    assert_eq!(from_unit.blue, u16::MAX);

    assert!(Color::from_unit_rgb(-0.1, 0.5, 1.0).is_none());
    assert!(Color::from_unit_rgb(1.1, 0.5, 1.0).is_none());
    assert!(Color::from_unit_rgb(f64::NAN, 0.5, 1.0).is_none());
}

#[test]
fn test_point_attributes() {
    let mut attrs = point_formats::types::PointAttributes::default();
    assert!(attrs.is_empty());

    // Test DerefMut
    attrs.insert("test".to_string(), AttributeValue::Int(42));
    assert_eq!(attrs.len(), 1);
    assert_eq!(attrs.get("test"), Some(&AttributeValue::Int(42)));

    // Test Deref
    let readonly = attrs.clone();
    assert_eq!(readonly.get("test"), Some(&AttributeValue::Int(42)));

    // Test From/Into
    let mut map = BTreeMap::new();
    map.insert("a".to_string(), AttributeValue::UInt(100));
    map.insert("b".to_string(), AttributeValue::Float(3.14));
    map.insert("c".to_string(), AttributeValue::Text("hello".to_string()));

    let point_attrs = point_formats::types::PointAttributes::from(map.clone());
    assert_eq!(point_attrs.len(), 3);

    let back_to_map: BTreeMap<String, AttributeValue> = point_attrs.into();
    assert_eq!(back_to_map, map);

    // Empty map conversion
    let empty_attrs = point_formats::types::PointAttributes::from(BTreeMap::new());
    assert!(empty_attrs.0.is_none());
    let empty_map: BTreeMap<String, AttributeValue> = empty_attrs.into();
    assert!(empty_map.is_empty());
}

#[test]
fn test_bounds3() {
    let empty = Bounds3::empty();
    assert_eq!(
        empty.min,
        Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY)
    );
    assert_eq!(
        empty.max,
        Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY)
    );

    let p1 = Point::new(1.0, 2.0, 3.0);
    let p2 = Point::new(-1.0, 5.0, 2.0);

    let bounds = Bounds3::from_points(&[p1, p2]).unwrap();
    assert_eq!(bounds.min, Vec3::new(-1.0, 2.0, 2.0));
    assert_eq!(bounds.max, Vec3::new(1.0, 5.0, 3.0));

    assert!(Bounds3::from_points(&[] as &[Point]).is_none());

    let v1 = Vertex::new(Vec3::new(0.0, 0.0, 0.0));
    let v2 = Vertex::new(Vec3::new(10.0, -10.0, 5.0));
    let bounds_v = Bounds3::from_vertices(&[v1, v2]).unwrap();
    assert_eq!(bounds_v.min, Vec3::new(0.0, -10.0, 0.0));
    assert_eq!(bounds_v.max, Vec3::new(10.0, 0.0, 5.0));

    assert!(Bounds3::from_vertices(&[] as &[Vertex]).is_none());
}

#[test]
fn test_point_cloud_and_geometry() {
    let mut cloud = PointCloud::empty();
    assert!(cloud.is_empty());
    assert_eq!(cloud.len(), 0);
    assert!(cloud.bounds().is_none());

    let p1 = Point::new(1.0, 2.0, 3.0)
        .with_intensity(10.0)
        .with_color(Color::from_u8(255, 0, 0))
        .with_classification(3)
        .with_normal(Vec3::new(0.0, 0.0, 1.0));

    let mut p2 = Point::new(4.0, 5.0, 6.0);
    p2.gps_time = Some(100.0);

    cloud.points.push(p1);
    cloud.points.push(p2);

    assert!(!cloud.is_empty());
    assert_eq!(cloud.len(), 2);
    assert!(cloud.bounds().is_some());
    assert!(cloud.has_intensity());
    assert!(cloud.has_color());
    assert!(cloud.has_classification());
    assert!(cloud.has_gps_time());
    assert!(cloud.has_normals());

    let mut geom = Geometry::PointCloud(cloud);
    assert_eq!(geom.point_count(), 2);
    assert_eq!(geom.face_count(), 0);
    assert_eq!(geom.metadata().comments.len(), 0);

    geom.metadata_mut().comments.push("hello".to_string());
    assert_eq!(geom.metadata().comments[0], "hello");

    let mesh = Mesh::new(
        vec![Vertex::new(Vec3::new(0.0, 0.0, 0.0))],
        vec![Face::new(0, 0, 0)],
    );
    let mut geom_mesh = Geometry::Mesh(mesh);
    assert_eq!(geom_mesh.point_count(), 1);
    assert_eq!(geom_mesh.face_count(), 1);
    assert!(geom_mesh.metadata_mut().warnings.is_empty());

    let m_bounds = geom_mesh.metadata().clone();
    assert!(m_bounds.warnings.is_empty());
}

#[test]
fn test_format() {
    // from_path, from_path_opt
    assert_eq!(Format::from_path_opt("a.xyz"), Some(Format::Xyz));
    assert_eq!(Format::from_path_opt("a.tar.gz"), None);
    assert_eq!(Format::from_path_opt("a.unknown"), None);
    assert!(Format::from_path("a.unknown").is_err());

    // display
    assert_eq!(format!("{}", Format::Xyz), "xyz");

    // FromStr
    assert_eq!(Format::from_str("xyz").unwrap(), Format::Xyz);
    assert_eq!(Format::from_str("copclaz").unwrap(), Format::Copc);
    assert_eq!(Format::from_str("ascii-grid").unwrap(), Format::AsciiGrid);
    assert!(Format::from_str("invalid").is_err());

    // ALL and details
    for &f in Format::ALL {
        let name = f.name();
        let _family = f.family();
        let support = f.support();
        let hint = f.adapter_hint();
        assert!(!name.is_empty());
        assert!(!hint.is_empty());

        let parsed = Format::from_str(name).unwrap();
        assert_eq!(parsed, f);

        // Native read/write check
        let is_r = f.is_native_read();
        let is_w = f.is_native_write();
        match support {
            FormatSupport::NativeReadWrite => {
                assert!(is_r);
                assert!(is_w);
            }
            FormatSupport::NativeReadOnly => {
                assert!(is_r);
                assert!(!is_w);
            }
            FormatSupport::NativeWriteOnly => {
                assert!(!is_r);
                assert!(is_w);
            }
            _ => {
                assert!(!is_r);
                assert!(!is_w);
            }
        }
    }
}

// Dummy codec implementation to test CodecRegistry
struct DummyCodec;
impl Codec for DummyCodec {
    fn can_read(&self, format: Format) -> bool {
        matches!(format, Format::Copc)
    }
    fn can_write(&self, format: Format) -> bool {
        matches!(format, Format::Copc)
    }
    fn read_path(
        &self,
        _path: &std::path::Path,
        _format: Format,
        _options: &point_formats::ConvertOptions,
    ) -> Result<Geometry, Error> {
        Ok(Geometry::PointCloud(PointCloud::empty()))
    }
    fn write_path(
        &self,
        _path: &std::path::Path,
        _format: Format,
        _geometry: &Geometry,
        _options: &point_formats::ConvertOptions,
    ) -> Result<(), Error> {
        Ok(())
    }
}

#[test]
fn test_codec_registry() {
    let mut registry = CodecRegistry::new();
    assert!(registry.reader(Format::Copc).is_none());

    registry.register(Box::new(DummyCodec));
    assert!(registry.reader(Format::Copc).is_some());
    assert!(registry.writer(Format::Copc).is_some());
    assert!(registry.reader(Format::Xyz).is_none());

    let registry2 = CodecRegistry::new().with_codec(Box::new(DummyCodec));
    assert!(registry2.reader(Format::Copc).is_some());
}

#[test]
fn test_pts_codec() {
    // 1. Valid roundtrip with XYZ, intensity, and color
    let mut cloud = PointCloud::empty();
    cloud.points.push(
        Point::new(1.0, 2.0, 3.0)
            .with_intensity(0.5)
            .with_color(Color::new(100, 200, 300)),
    );
    cloud.points.push(
        Point::new(4.0, 5.0, 6.0)
            .with_intensity(0.8)
            .with_color(Color::new(400, 500, 600)),
    );

    let mut buf = Vec::new();
    point_formats::io::pts::write(&mut buf, &cloud).unwrap();

    let mut cursor = std::io::Cursor::new(buf);
    let decoded = point_formats::io::pts::read(&mut cursor).unwrap();
    assert_eq!(decoded.points.len(), 2);
    assert_eq!(decoded.points[0].position.x, 1.0);
    assert_eq!(decoded.points[0].intensity, Some(0.5));
    assert_eq!(decoded.points[0].color, Some(Color::new(100, 200, 300)));

    // 2. Variations in fields
    // XYZ only
    let pts_xyz = b"2\n1 2 3\n4 5 6\n";
    let decoded_xyz = point_formats::io::pts::read(&mut std::io::Cursor::new(pts_xyz)).unwrap();
    assert_eq!(decoded_xyz.points.len(), 2);
    assert!(decoded_xyz.points[0].intensity.is_none());
    assert!(decoded_xyz.points[0].color.is_none());

    // XYZ + intensity (4 fields)
    let pts_intensity = b"1\n1 2 3 0.75\n";
    let decoded_int =
        point_formats::io::pts::read(&mut std::io::Cursor::new(pts_intensity)).unwrap();
    assert_eq!(decoded_int.points[0].intensity, Some(0.75));
    assert!(decoded_int.points[0].color.is_none());

    // XYZ + color (6 fields)
    let pts_color = b"1\n1 2 3 100 200 300\n";
    let decoded_col = point_formats::io::pts::read(&mut std::io::Cursor::new(pts_color)).unwrap();
    assert_eq!(decoded_col.points[0].color, Some(Color::new(100, 200, 300)));
    assert!(decoded_col.points[0].intensity.is_none());

    // XYZ + intensity + color (7 fields)
    let pts_all = b"1\n1 2 3 0.25 100 200 300\n";
    let decoded_all = point_formats::io::pts::read(&mut std::io::Cursor::new(pts_all)).unwrap();
    assert_eq!(decoded_all.points[0].intensity, Some(0.25));
    assert_eq!(decoded_all.points[0].color, Some(Color::new(100, 200, 300)));

    // 3. Comments and empty lines
    let pts_comments = b"// comment\n# comment\n\n1\n1 2 3\n";
    let decoded_comm =
        point_formats::io::pts::read(&mut std::io::Cursor::new(pts_comments)).unwrap();
    assert_eq!(decoded_comm.points.len(), 1);

    // 4. Mismatch warning
    let pts_mismatch = b"10\n1 2 3\n";
    let decoded_mismatch =
        point_formats::io::pts::read(&mut std::io::Cursor::new(pts_mismatch)).unwrap();
    assert_eq!(decoded_mismatch.points.len(), 1);
    assert!(!decoded_mismatch.metadata.warnings.is_empty());

    // 5. Parse errors
    // less than 3 fields
    assert!(point_formats::io::pts::read(&mut std::io::Cursor::new(b"1 2\n")).is_err());
    // invalid floats
    assert!(point_formats::io::pts::read(&mut std::io::Cursor::new(b"1 2 abc\n")).is_err());
    assert!(point_formats::io::pts::read(&mut std::io::Cursor::new(b"1 2 3 abc\n")).is_err());
    // invalid colors
    assert!(
        point_formats::io::pts::read(&mut std::io::Cursor::new(b"1 2 3 100 abc 300\n")).is_err()
    );
}

#[test]
fn test_ptx_codec() {
    // 1. Valid roundtrip with transform, intensity, and color
    let mut cloud = PointCloud::empty();
    cloud.points.push(
        Point::new(1.0, 2.0, 3.0)
            .with_intensity(0.5)
            .with_color(Color::new(100, 200, 300)),
    );
    let transform = [
        [1.0, 0.0, 0.0, 10.0],
        [0.0, 1.0, 0.0, 20.0],
        [0.0, 0.0, 1.0, 30.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    cloud.metadata.scanner_transform = Some(transform);

    let mut buf = Vec::new();
    point_formats::io::ptx::write(&mut buf, &cloud).unwrap();

    let mut cursor = std::io::Cursor::new(buf);
    let decoded = point_formats::io::ptx::read(&mut cursor).unwrap();
    assert_eq!(decoded.points.len(), 1);
    assert_eq!(decoded.metadata.scanner_transform, Some(transform));
    assert_eq!(decoded.points[0].position.x, 1.0);
    assert_eq!(decoded.points[0].intensity, Some(0.5));
    assert_eq!(decoded.points[0].color, Some(Color::new(100, 200, 300)));

    // 2. Parse errors
    // Truncated header
    assert!(point_formats::io::ptx::read(&mut std::io::Cursor::new(b"1\n2\n")).is_err());
    // Invalid rows/columns
    assert!(point_formats::io::ptx::read(&mut std::io::Cursor::new(
        b"abc\n1\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n"
    ))
    .is_err());
    // Column/row overflow
    assert!(point_formats::io::ptx::read(&mut std::io::Cursor::new(
        format!(
            "{}\n{}\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n",
            usize::MAX,
            usize::MAX
        )
        .as_bytes()
    ))
    .is_err());
    // Transform row with invalid values or not 4 values
    assert!(point_formats::io::ptx::read(&mut std::io::Cursor::new(
        b"1\n1\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n1 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n"
    ))
    .is_err());
    // Truncated points (warning)
    let ptx_short =
        b"2\n1\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n1 2 3\n";
    let decoded_short = point_formats::io::ptx::read(&mut std::io::Cursor::new(ptx_short)).unwrap();
    assert_eq!(decoded_short.points.len(), 1);
    assert!(!decoded_short.metadata.warnings.is_empty());

    // Invalid point records
    let ptx_invalid_pt =
        b"1\n1\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n1 2\n";
    assert!(point_formats::io::ptx::read(&mut std::io::Cursor::new(ptx_invalid_pt)).is_err());
}

#[test]
#[cfg(feature = "copc")]
fn test_copc_codec() {
    let mut cursor = std::io::Cursor::new(vec![0; 10]);
    assert!(point_formats::io::copc::read(&mut cursor).is_err());
}

#[test]
fn test_stl_codec() {
    // 1. Lossy conversion: PointCloud to STL should fail
    let mut cloud = PointCloud::empty();
    cloud.points.push(Point::new(1.0, 2.0, 3.0));
    let mut buf = Vec::new();
    let res = point_formats::io::stl::write(
        &mut buf,
        &Geometry::PointCloud(cloud),
        &point_formats::io::StlOptions { binary: true },
    );
    assert!(res.is_err());

    // 2. Validate mesh: invalid face index
    let bad_mesh = Mesh::new(
        vec![Vertex::new(Vec3::new(0.0, 0.0, 0.0))],
        vec![Face::new(0, 1, 2)], // vertex indices 1 and 2 don't exist
    );
    let res = point_formats::io::stl::write(
        &mut buf,
        &Geometry::Mesh(bad_mesh),
        &point_formats::io::StlOptions { binary: true },
    );
    assert!(res.is_err());

    // 3. ASCII STL roundtrip
    let mesh = Mesh::new(
        vec![
            Vertex::new(Vec3::new(0.0, 0.0, 0.0)),
            Vertex::new(Vec3::new(1.0, 0.0, 0.0)),
            Vertex::new(Vec3::new(0.0, 1.0, 0.0)),
        ],
        vec![Face::new(0, 1, 2)],
    );
    let mut ascii_buf = Vec::new();
    point_formats::io::stl::write(
        &mut ascii_buf,
        &Geometry::Mesh(mesh.clone()),
        &point_formats::io::StlOptions { binary: false },
    )
    .unwrap();

    let mut cursor = std::io::Cursor::new(ascii_buf);
    let decoded_ascii = point_formats::io::stl::read(&mut cursor).unwrap();
    match decoded_ascii {
        Geometry::Mesh(m) => {
            assert_eq!(m.faces.len(), 1);
            assert_eq!(m.vertices.len(), 3);
        }
        _ => panic!("Expected mesh"),
    }

    // 4. ASCII STL errors
    // Facet with less than 3 vertices
    let bad_ascii = b"solid test\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nendloop\nendfacet\nendsolid\n";
    assert!(point_formats::io::stl::read(&mut std::io::Cursor::new(bad_ascii)).is_err());
    // Invalid vertex float
    let bad_vertex = b"solid test\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 abc\nvertex 0 1 0\nendloop\nendfacet\nendsolid\n";
    assert!(point_formats::io::stl::read(&mut std::io::Cursor::new(bad_vertex)).is_err());

    // 5. Binary check short files
    let short_bin = vec![0; 50];
    assert!(point_formats::io::stl::read(&mut std::io::Cursor::new(short_bin)).is_ok());
    // falls back to ASCII read of invalid text which succeeds (empty mesh)
}

#[test]
fn test_obj_codec() {
    // 1. Write PointCloud to OBJ
    let mut cloud = PointCloud::empty();
    cloud
        .points
        .push(Point::new(1.0, 2.0, 3.0).with_color(Color::new(100, 200, 300)));
    let mut buf = Vec::new();
    point_formats::io::obj::write(&mut buf, &Geometry::PointCloud(cloud)).unwrap();

    // 2. Write Mesh to OBJ
    let mesh = Mesh::new(
        vec![
            Vertex {
                position: Vec3::new(0.0, 0.0, 0.0),
                normal: Some(Vec3::new(0.0, 0.0, 1.0)),
                color: Some(Color::new(100, 200, 300)),
            },
            Vertex {
                position: Vec3::new(1.0, 0.0, 0.0),
                normal: None,
                color: None,
            },
            Vertex {
                position: Vec3::new(0.0, 1.0, 0.0),
                normal: None,
                color: None,
            },
        ],
        vec![Face::new(0, 1, 2)],
    );
    let mut mesh_buf = Vec::new();
    point_formats::io::obj::write(&mut mesh_buf, &Geometry::Mesh(mesh.clone())).unwrap();

    // 3. Read OBJ comments and vertices
    let obj_content = b"# comment 1\n# comment 2\nv 0 0 0 0.5 0.5 0.5\nv 1 0 0 1000 2000 3000\nv 0 1 0\nvt 0 0\nvn 0 0 1\nf 1 2 3\n";
    let decoded_geom =
        point_formats::io::obj::read(&mut std::io::Cursor::new(obj_content)).unwrap();
    match decoded_geom {
        Geometry::Mesh(m) => {
            assert_eq!(m.metadata.comments.len(), 2);
            assert_eq!(m.metadata.comments[0], "comment 1");
            assert_eq!(m.vertices.len(), 3);
            assert_eq!(m.faces.len(), 1);
            // Verify warning about vn indices present
            assert!(!m.metadata.warnings.is_empty());
        }
        _ => panic!("Expected mesh"),
    }

    // 4. Read vertex-only OBJ (becomes PointCloud)
    let obj_pts = b"v 0 0 0\nv 1 0 0\n";
    let decoded_pts = point_formats::io::obj::read(&mut std::io::Cursor::new(obj_pts)).unwrap();
    match decoded_pts {
        Geometry::PointCloud(c) => {
            assert_eq!(c.points.len(), 2);
        }
        _ => panic!("Expected point cloud"),
    }

    // 5. Negative face index
    let obj_neg = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\n";
    let decoded_neg = point_formats::io::obj::read(&mut std::io::Cursor::new(obj_neg)).unwrap();
    match decoded_neg {
        Geometry::Mesh(m) => {
            assert_eq!(m.faces.len(), 1);
            assert_eq!(m.faces[0].indices, [0, 1, 2]);
        }
        _ => panic!("Expected mesh"),
    }

    // 6. Errors
    // Invalid index 0
    assert!(
        point_formats::io::obj::read(&mut std::io::Cursor::new(b"v 0 0 0\nf 0 1 2\n")).is_err()
    );
    // Out of bounds negative index
    assert!(
        point_formats::io::obj::read(&mut std::io::Cursor::new(b"v 0 0 0\nf -5 1 2\n")).is_err()
    );
    // Out of bounds positive index
    assert!(
        point_formats::io::obj::read(&mut std::io::Cursor::new(b"v 0 0 0\nf 5 1 2\n")).is_err()
    );
    // Invalid index format
    assert!(
        point_formats::io::obj::read(&mut std::io::Cursor::new(b"v 0 0 0\nf a b c\n")).is_err()
    );
    // Truncated vertex
    assert!(point_formats::io::obj::read(&mut std::io::Cursor::new(b"v 0 0\n")).is_err());
    // Truncated normal
    assert!(point_formats::io::obj::read(&mut std::io::Cursor::new(b"vn 0 0\n")).is_err());
    // Truncated face
    assert!(
        point_formats::io::obj::read(&mut std::io::Cursor::new(b"v 0 0 0\nv 1 0 0\nf 1 2\n"))
            .is_err()
    );
}

#[test]
fn test_pcd_codec() {
    // 1. Binary roundtrip with all attributes
    let mut cloud = PointCloud::empty();
    cloud.points.push(
        Point::new(1.0, 2.0, 3.0)
            .with_intensity(0.5)
            .with_color(Color::new(100, 200, 300))
            .with_classification(5)
            .with_normal(Vec3::new(0.0, 0.0, 1.0)),
    );
    let options = point_formats::io::PcdOptions {
        encoding: point_formats::io::PcdEncoding::Binary,
        precision: 6,
    };
    let mut buf = Vec::new();
    point_formats::io::pcd::write(&mut buf, &cloud, &options).unwrap();

    let mut cursor = std::io::Cursor::new(buf);
    let decoded = point_formats::io::pcd::read(&mut cursor).unwrap();
    match decoded {
        Geometry::PointCloud(c) => {
            assert_eq!(c.points.len(), 1);
            assert_eq!(c.points[0].position.x, 1.0);
            assert_eq!(c.points[0].intensity, Some(0.5));
            assert_eq!(c.points[0].color, Some(Color::new(100, 200, 300)));
            assert_eq!(c.points[0].classification, Some(5));
            assert_eq!(c.points[0].normal, Some(Vec3::new(0.0, 0.0, 1.0)));
        }
        _ => panic!("Expected point cloud"),
    }

    // 2. Packed RGB float parsing
    let pcd_rgb = b"VERSION 0.7\nFIELDS x y z rgb\nSIZE 8 8 8 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1.0 2.0 3.0 4.2108e+06\n";
    let decoded_rgb = point_formats::io::pcd::read(&mut std::io::Cursor::new(pcd_rgb)).unwrap();
    match decoded_rgb {
        Geometry::PointCloud(c) => {
            assert_eq!(c.points.len(), 1);
            assert!(c.points[0].color.is_some());
        }
        _ => panic!("Expected point cloud"),
    }

    // 3. Error cases
    // Truncated header
    assert!(point_formats::io::pcd::read(&mut std::io::Cursor::new(b"VERSION 0.7\n")).is_err());
    // Missing FIELDS
    assert!(point_formats::io::pcd::read(&mut std::io::Cursor::new(
        b"VERSION 0.7\nSIZE 4\nDATA ascii\n"
    ))
    .is_err());
    // Length mismatch
    assert!(point_formats::io::pcd::read(&mut std::io::Cursor::new(
        b"VERSION 0.7\nFIELDS x\nSIZE 4 4\nDATA ascii\n"
    ))
    .is_err());
    // Unknown TYPE
    assert!(point_formats::io::pcd::read(&mut std::io::Cursor::new(
        b"VERSION 0.7\nFIELDS x\nSIZE 4\nTYPE A\nDATA ascii\n"
    ))
    .is_err());
    // Unknown encoding
    assert!(point_formats::io::pcd::read(&mut std::io::Cursor::new(
        b"VERSION 0.7\nFIELDS x\nSIZE 4\nTYPE F\nDATA invalid\n"
    ))
    .is_err());
    // BinaryCompressed unsupported
    assert!(point_formats::io::pcd::read(&mut std::io::Cursor::new(
        b"VERSION 0.7\nFIELDS x\nSIZE 4\nTYPE F\nDATA binary_compressed\n"
    ))
    .is_err());
    // Truncated ASCII points
    let pcd_trunc_ascii = b"VERSION 0.7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n1 2 3\n";
    assert!(point_formats::io::pcd::read(&mut std::io::Cursor::new(pcd_trunc_ascii)).is_err());
    // Truncated ASCII columns
    let pcd_trunc_cols = b"VERSION 0.7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2\n";
    assert!(point_formats::io::pcd::read(&mut std::io::Cursor::new(pcd_trunc_cols)).is_err());
    // Truncated binary points
    let pcd_trunc_bin = b"VERSION 0.7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n1234";
    assert!(point_formats::io::pcd::read(&mut std::io::Cursor::new(pcd_trunc_bin)).is_err());
}

#[test]
fn test_ply_codec() {
    let cloud = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)
        .with_intensity(0.5)
        .with_color(Color::new(100, 200, 300))
        .with_classification(4)
        .with_normal(Vec3::new(0.0, 0.0, 1.0))]);

    // 1. Binary little endian point cloud roundtrip
    let mut bin_buf = Vec::new();
    point_formats::io::ply::write(
        &mut bin_buf,
        &Geometry::PointCloud(cloud.clone()),
        &point_formats::io::PlyOptions {
            encoding: point_formats::io::PlyEncoding::BinaryLittleEndian,
            precision: 6,
        },
    )
    .unwrap();

    let decoded_bin = point_formats::io::ply::read(&mut std::io::Cursor::new(bin_buf)).unwrap();
    match decoded_bin {
        Geometry::PointCloud(c) => {
            assert_eq!(c.points.len(), 1);
            assert_eq!(c.points[0].position.x, 1.0);
            assert_eq!(c.points[0].intensity, Some(0.5));
            assert_eq!(c.points[0].color, Some(Color::new(100, 200, 300)));
            assert_eq!(c.points[0].classification, Some(4));
            assert_eq!(c.points[0].normal, Some(Vec3::new(0.0, 0.0, 1.0)));
        }
        _ => panic!("Expected point cloud"),
    }

    // 2. Binary little endian mesh roundtrip
    let mesh = Mesh::new(
        vec![
            Vertex {
                position: Vec3::new(0.0, 0.0, 0.0),
                normal: Some(Vec3::new(0.0, 0.0, 1.0)),
                color: Some(Color::new(100, 200, 300)),
            },
            Vertex {
                position: Vec3::new(1.0, 0.0, 0.0),
                normal: None,
                color: None,
            },
            Vertex {
                position: Vec3::new(0.0, 1.0, 0.0),
                normal: None,
                color: None,
            },
        ],
        vec![Face::new(0, 1, 2)],
    );
    let mut mesh_bin_buf = Vec::new();
    point_formats::io::ply::write(
        &mut mesh_bin_buf,
        &Geometry::Mesh(mesh.clone()),
        &point_formats::io::PlyOptions {
            encoding: point_formats::io::PlyEncoding::BinaryLittleEndian,
            precision: 6,
        },
    )
    .unwrap();

    let decoded_mesh =
        point_formats::io::ply::read(&mut std::io::Cursor::new(mesh_bin_buf)).unwrap();
    match decoded_mesh {
        Geometry::Mesh(m) => {
            assert_eq!(m.vertices.len(), 3);
            assert_eq!(m.faces.len(), 1);
            assert_eq!(m.vertices[0].normal, Some(Vec3::new(0.0, 0.0, 1.0)));
        }
        _ => panic!("Expected mesh"),
    }

    // 3. ASCII mesh roundtrip
    let mut mesh_ascii_buf = Vec::new();
    point_formats::io::ply::write(
        &mut mesh_ascii_buf,
        &Geometry::Mesh(mesh.clone()),
        &point_formats::io::PlyOptions {
            encoding: point_formats::io::PlyEncoding::Ascii,
            precision: 6,
        },
    )
    .unwrap();

    let decoded_ascii_mesh =
        point_formats::io::ply::read(&mut std::io::Cursor::new(mesh_ascii_buf)).unwrap();
    match decoded_ascii_mesh {
        Geometry::Mesh(m) => {
            assert_eq!(m.vertices.len(), 3);
            assert_eq!(m.faces.len(), 1);
        }
        _ => panic!("Expected mesh"),
    }

    // 4. PLY Errors
    // Invalid magic
    assert!(point_formats::io::ply::read(&mut std::io::Cursor::new(b"invalid\n")).is_err());
    // Truncated header
    assert!(point_formats::io::ply::read(&mut std::io::Cursor::new(b"ply\n")).is_err());
    // Unknown element
    assert!(point_formats::io::ply::read(&mut std::io::Cursor::new(
        b"ply\nformat ascii 1.0\nelement invalid 10\nend_header\n"
    ))
    .is_err());
    // Invalid property type
    assert!(point_formats::io::ply::read(&mut std::io::Cursor::new(
        b"ply\nformat ascii 1.0\nelement vertex 1\nproperty invalid x\nend_header\n"
    ))
    .is_err());
    // Truncated ASCII values
    let bad_ply = b"ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nend_header\n1.0\n";
    assert!(point_formats::io::ply::read(&mut std::io::Cursor::new(bad_ply)).is_err());
}

#[test]
fn test_convert_pipeline() {
    let mesh = Mesh::new(
        vec![
            Vertex::new(Vec3::new(0.0, 0.0, 0.0)),
            Vertex::new(Vec3::new(1.0, 0.0, 0.0)),
            Vertex::new(Vec3::new(0.0, 1.0, 0.0)),
        ],
        vec![Face::new(0, 1, 2)],
    );
    let geom_mesh = Geometry::Mesh(mesh);

    let dir = std::env::temp_dir().join(format!("convert_test_{}", uuid::Uuid::new_v4()));
    std::fs::create_dir_all(&dir).unwrap();
    let mesh_ply = dir.join("mesh.ply");
    let out_xyz = dir.join("out.xyz");
    let out_ply = dir.join("out.ply");

    // Write a mesh PLY first
    point_formats::io::write_path(&mesh_ply, Format::Ply, &geom_mesh, &Default::default()).unwrap();

    // Convert mesh to point cloud with allow_lossy = false -> should fail
    let opts_strict = point_formats::ConvertOptions {
        allow_lossy: false,
        geometry_policy: point_formats::GeometryPolicy::PointsOnly,
        ..Default::default()
    };
    assert!(point_formats::convert_path(&mesh_ply, &out_xyz, &opts_strict).is_err());

    // Convert mesh to point cloud with allow_lossy = true -> should succeed
    let opts_lossy = point_formats::ConvertOptions {
        allow_lossy: true,
        geometry_policy: point_formats::GeometryPolicy::PointsOnly,
        ..Default::default()
    };
    let report = point_formats::convert_path(&mesh_ply, &out_xyz, &opts_lossy).unwrap();
    assert_eq!(report.points_written, 3);
    assert_eq!(report.faces_written, 0);

    // Convert point cloud to mesh only -> should fail
    let opts_mesh_only = point_formats::ConvertOptions {
        geometry_policy: point_formats::GeometryPolicy::MeshOnly,
        ..Default::default()
    };
    assert!(point_formats::convert_path(&out_xyz, &out_ply, &opts_mesh_only).is_err());

    // test geometry_to_point_cloud directly
    let res_cloud =
        point_formats::convert::geometry_to_point_cloud(geom_mesh.clone(), Format::Xyz, true)
            .unwrap();
    assert_eq!(res_cloud.points.len(), 3);
    assert!(
        point_formats::convert::geometry_to_point_cloud(geom_mesh, Format::Xyz, false).is_err()
    );

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_cli_binary() {
    let mut cmd = std::process::Command::new("cargo");
    cmd.args(&["run", "--bin", "lidar-convert", "--", "--help"]);
    let out = cmd.output().unwrap();
    assert!(out.status.success());
    let stdout = String::from_utf8(out.stdout).unwrap();
    assert!(stdout.contains("lidar-convert <input> <output>"));

    let mut cmd2 = std::process::Command::new("cargo");
    cmd2.args(&["run", "--bin", "lidar-convert", "--", "--list-formats"]);
    let out2 = cmd2.output().unwrap();
    assert!(out2.status.success());
    let stdout2 = String::from_utf8(out2.stdout).unwrap();
    assert!(stdout2.contains("format"));

    // Unknown options should return error (exit code 2)
    let mut cmd3 = std::process::Command::new("cargo");
    cmd3.args(&["run", "--bin", "lidar-convert", "--", "--invalid-option"]);
    let out3 = cmd3.output().unwrap();
    assert_eq!(out3.status.code(), Some(2));
}

#[test]
fn test_asciigrid_codec_edge_cases() {
    // Center parsing instead of corner
    let grid_data = b"ncols 2\nnrows 2\nxllcenter 10.0\nyllcenter 10.0\ncellsize 2.0\n1 2\n3 4\n";
    let decoded = point_formats::io::asciigrid::read(&mut &grid_data[..]).unwrap();
    match decoded {
        Geometry::PointCloud(c) => {
            assert_eq!(c.points.len(), 4);
        }
        _ => panic!("Expected point cloud"),
    }

    // Invalid header cases
    assert!(point_formats::io::asciigrid::read(
        &mut &b"ncols abc\nnrows 2\nxllcorner 0.0\nyllcorner 0.0\ncellsize 1.0\n"[..]
    )
    .is_err());
    assert!(point_formats::io::asciigrid::read(
        &mut &b"ncols 2\nnrows abc\nxllcorner 0.0\nyllcorner 0.0\ncellsize 1.0\n"[..]
    )
    .is_err());
    assert!(point_formats::io::asciigrid::read(
        &mut &b"ncols 2\nnrows 2\nxllcorner abc\nyllcorner 0.0\ncellsize 1.0\n"[..]
    )
    .is_err());
    assert!(point_formats::io::asciigrid::read(
        &mut &b"ncols 2\nnrows 2\nxllcorner 0.0\nyllcorner abc\ncellsize 1.0\n"[..]
    )
    .is_err());
    assert!(point_formats::io::asciigrid::read(
        &mut &b"ncols 2\nnrows 2\nxllcenter abc\nyllcenter 0.0\ncellsize 1.0\n"[..]
    )
    .is_err());
    assert!(point_formats::io::asciigrid::read(
        &mut &b"ncols 2\nnrows 2\nxllcenter 0.0\nyllcenter abc\ncellsize 1.0\n"[..]
    )
    .is_err());
    assert!(point_formats::io::asciigrid::read(
        &mut &b"ncols 2\nnrows 2\nxllcorner 0.0\nyllcorner 0.0\ncellsize abc\n"[..]
    )
    .is_err());
    assert!(point_formats::io::asciigrid::read(&mut &b"ncols 2\nnrows 2\nxllcorner 0.0\nyllcorner 0.0\ncellsize 1.0\nnodata_value abc\n"[..]).is_err());

    // Missing keys
    assert!(
        point_formats::io::asciigrid::read(&mut &b"ncols 2\nnrows 2\ncellsize 1.0\n"[..]).is_err()
    ); // missing xllcorner/center
    assert!(point_formats::io::asciigrid::read(
        &mut &b"ncols 2\nxllcorner 0.0\nyllcorner 0.0\ncellsize 1.0\n"[..]
    )
    .is_err()); // missing nrows

    // Body parsing error
    assert!(point_formats::io::asciigrid::read(
        &mut &b"ncols 2\nnrows 2\nxllcorner 0.0\nyllcorner 0.0\ncellsize 1.0\n1.0 abc 3.0 4.0\n"[..]
    )
    .is_err());

    // Empty write error
    let mut buf = Vec::new();
    assert!(point_formats::io::asciigrid::write(&mut buf, &PointCloud::empty()).is_err());
}

#[test]
fn test_delimited_codec_edge_cases() {
    // 1. Missing columns header mapping
    let content_bad_header = b"intensity\n1.0\n";
    let res = point_formats::io::delimited::read(
        &mut &content_bad_header[..],
        Format::Csv,
        &point_formats::io::DelimitedOptions {
            delimiter: point_formats::io::Delimiter::Comma,
            has_header: Some(true),
            ..Default::default()
        },
    );
    assert!(res.is_err());

    // 2. Missing required column value (out of range index mapping)
    let content_short = b"1.0,2.0\n"; // missing z
    let res2 = point_formats::io::delimited::read(
        &mut &content_short[..],
        Format::Csv,
        &point_formats::io::DelimitedOptions {
            delimiter: point_formats::io::Delimiter::Comma,
            has_header: Some(false),
            ..Default::default()
        },
    );
    assert!(res2.is_err());

    // 3. Non-finite coordinates
    let content_nan = b"1.0,2.0,NaN\n";
    let res3 = point_formats::io::delimited::read(
        &mut &content_nan[..],
        Format::Csv,
        &point_formats::io::DelimitedOptions {
            delimiter: point_formats::io::Delimiter::Comma,
            has_header: Some(false),
            ..Default::default()
        },
    );
    assert!(res3.is_err());

    // 4. cloud_with_format helper test (via public write csv)
    let mut cloud = PointCloud::empty();
    cloud.points.push(Point::new(1.0, 2.0, 3.0));
    let mut buf = Vec::new();
    point_formats::io::delimited::write(
        &mut buf,
        Format::Csv,
        &cloud,
        &point_formats::io::DelimitedOptions {
            delimiter: point_formats::io::Delimiter::Comma,
            write_header: true,
            ..Default::default()
        },
    )
    .unwrap();
    assert!(String::from_utf8(buf).unwrap().contains("x,y,z"));
}
