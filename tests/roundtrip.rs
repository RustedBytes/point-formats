use lidar_format_convert::io::{self, PcdEncoding, PlyEncoding};
use lidar_format_convert::{
    convert_path, Color, ConvertOptions, Face, Format, Geometry, Mesh, Point, PointCloud, Vec3,
    Vertex,
};
use std::fs;
use std::io::Cursor;
use std::path::PathBuf;

fn approx(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-9, "{a} != {b}");
}

fn sample_cloud() -> PointCloud {
    PointCloud::new(vec![
        Point::new(1.25, 2.5, -3.75)
            .with_intensity(42.5)
            .with_color(Color::new(1000, 2000, 3000))
            .with_classification(2)
            .with_normal(Vec3::new(0.0, 0.0, 1.0)),
        Point::new(4.0, 5.0, 6.0)
            .with_intensity(7.0)
            .with_color(Color::new(10, 20, 30))
            .with_classification(5),
    ])
}

#[test]
fn detects_compound_extensions() {
    assert_eq!(Format::from_path("tile.copc.laz").unwrap(), Format::Copc);
    assert_eq!(Format::from_path("dem.cog.tif").unwrap(), Format::Cog);
    assert_eq!(Format::from_path("scan.PCD").unwrap(), Format::Pcd);
}

#[test]
fn csv_header_roundtrip() {
    let mut bytes = Vec::new();
    let mut options = io::DelimitedOptions::default();
    options.write_header = true;
    options.delimiter = io::Delimiter::Comma;
    io::delimited::write(&mut bytes, Format::Csv, &sample_cloud(), &options).unwrap();

    let mut cursor = Cursor::new(bytes);
    let decoded = io::delimited::read(&mut cursor, Format::Csv, &options).unwrap();
    assert_eq!(decoded.points.len(), 2);
    approx(decoded.points[0].position.x, 1.25);
    assert_eq!(decoded.points[0].color.unwrap().red, 1000);
    assert_eq!(decoded.points[1].classification, Some(5));
}

#[test]
fn ply_ascii_roundtrip_point_cloud() {
    let geometry = Geometry::PointCloud(sample_cloud());
    let options = io::PlyOptions {
        encoding: PlyEncoding::Ascii,
        precision: 9,
    };
    let mut bytes = Vec::new();
    io::ply::write(&mut bytes, &geometry, &options).unwrap();

    let mut cursor = Cursor::new(bytes);
    let decoded = io::ply::read(&mut cursor).unwrap();
    match decoded {
        Geometry::PointCloud(cloud) => {
            assert_eq!(cloud.points.len(), 2);
            assert_eq!(cloud.points[0].color.unwrap(), Color::new(1000, 2000, 3000));
            assert_eq!(cloud.points[0].classification, Some(2));
        }
        Geometry::Mesh(_) => panic!("expected point cloud"),
    }
}

#[test]
fn pcd_ascii_roundtrip_point_cloud() {
    let options = io::PcdOptions {
        encoding: PcdEncoding::Ascii,
        precision: 9,
    };
    let mut bytes = Vec::new();
    io::pcd::write(&mut bytes, &sample_cloud(), &options).unwrap();

    let mut cursor = Cursor::new(bytes);
    let decoded = io::pcd::read(&mut cursor).unwrap();
    match decoded {
        Geometry::PointCloud(cloud) => {
            assert_eq!(cloud.points.len(), 2);
            approx(cloud.points[0].position.z, -3.75);
            assert_eq!(cloud.points[0].color.unwrap().green, 2000);
        }
        Geometry::Mesh(_) => panic!("expected point cloud"),
    }
}

#[test]
fn obj_mesh_roundtrip_triangulates_quad() {
    let obj = b"v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n";
    let mut cursor = Cursor::new(&obj[..]);
    let decoded = io::obj::read(&mut cursor).unwrap();
    match decoded {
        Geometry::Mesh(mesh) => {
            assert_eq!(mesh.vertices.len(), 4);
            assert_eq!(mesh.faces.len(), 2);
            assert_eq!(mesh.faces[0].indices, [0, 1, 2]);
            assert_eq!(mesh.faces[1].indices, [0, 2, 3]);
        }
        Geometry::PointCloud(_) => panic!("expected mesh"),
    }
}

#[test]
fn stl_binary_roundtrip_mesh() {
    let mesh = Mesh::new(
        vec![
            Vertex::new(Vec3::new(0.0, 0.0, 0.0)),
            Vertex::new(Vec3::new(1.0, 0.0, 0.0)),
            Vertex::new(Vec3::new(0.0, 1.0, 0.0)),
        ],
        vec![Face::new(0, 1, 2)],
    );
    let geometry = Geometry::Mesh(mesh);
    let mut bytes = Vec::new();
    io::stl::write(&mut bytes, &geometry, &io::StlOptions { binary: true }).unwrap();

    let mut cursor = Cursor::new(bytes);
    let decoded = io::stl::read(&mut cursor).unwrap();
    match decoded {
        Geometry::Mesh(mesh) => {
            assert_eq!(mesh.faces.len(), 1);
            assert_eq!(mesh.vertices.len(), 3);
        }
        Geometry::PointCloud(_) => panic!("expected mesh"),
    }
}

#[test]
fn convert_xyz_to_ply_file() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let input = dir.join("scan.xyz");
    let output = dir.join("scan.ply");
    fs::write(&input, "1 2 3\n4 5 6\n").unwrap();

    let report = convert_path(&input, &output, &ConvertOptions::default()).unwrap();
    assert_eq!(report.input_format, Format::Xyz);
    assert_eq!(report.output_format, Format::Ply);
    assert_eq!(report.points_written, 2);
    assert!(output.exists());
    let _ = fs::remove_dir_all(&dir);
}

fn unique_temp_dir() -> PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!(
        "lidar_format_convert_test_{}_{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    path
}
