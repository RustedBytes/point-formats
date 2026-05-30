use point_formats::io::{self, PcdEncoding, PlyEncoding};
use point_formats::{
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
    let options = io::DelimitedOptions {
        write_header: true,
        delimiter: io::Delimiter::Comma,
        ..Default::default()
    };
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

#[test]
#[cfg(feature = "las")]
fn las_roundtrip_point_cloud() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let input = dir.join("scan.xyz");
    let middle = dir.join("scan.las");
    let output = dir.join("scan.ply");
    fs::write(&input, "1.25 2.5 -3.75\n4.0 5.0 6.0\n").unwrap();

    let report1 = convert_path(&input, &middle, &ConvertOptions::default()).unwrap();
    assert_eq!(report1.input_format, Format::Xyz);
    assert_eq!(report1.output_format, Format::Las);
    assert_eq!(report1.points_written, 2);
    assert!(middle.exists());

    let report2 = convert_path(&middle, &output, &ConvertOptions::default()).unwrap();
    assert_eq!(report2.input_format, Format::Las);
    assert_eq!(report2.output_format, Format::Ply);
    assert_eq!(report2.points_written, 2);
    assert!(output.exists());

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "laz")]
fn laz_roundtrip_point_cloud() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let input = dir.join("scan.xyz");
    let middle = dir.join("scan.laz");
    let output = dir.join("scan.ply");
    fs::write(&input, "1.25 2.5 -3.75\n4.0 5.0 6.0\n").unwrap();

    let report1 = convert_path(&input, &middle, &ConvertOptions::default()).unwrap();
    assert_eq!(report1.input_format, Format::Xyz);
    assert_eq!(report1.output_format, Format::Laz);
    assert_eq!(report1.points_written, 2);
    assert!(middle.exists());

    let report2 = convert_path(&middle, &output, &ConvertOptions::default()).unwrap();
    assert_eq!(report2.input_format, Format::Laz);
    assert_eq!(report2.output_format, Format::Ply);
    assert_eq!(report2.points_written, 2);
    assert!(output.exists());

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "e57")]
fn e57_roundtrip_point_cloud() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let input = dir.join("scan.xyz");
    let middle = dir.join("scan.e57");
    let output = dir.join("scan.ply");
    fs::write(&input, "1.25 2.5 -3.75\n4.0 5.0 6.0\n").unwrap();

    let report1 = convert_path(&input, &middle, &ConvertOptions::default()).unwrap();
    assert_eq!(report1.input_format, Format::Xyz);
    assert_eq!(report1.output_format, Format::E57);
    assert_eq!(report1.points_written, 2);
    assert!(middle.exists());

    let report2 = convert_path(&middle, &output, &ConvertOptions::default()).unwrap();
    assert_eq!(report2.input_format, Format::E57);
    assert_eq!(report2.output_format, Format::Ply);
    assert_eq!(report2.points_written, 2);
    assert!(output.exists());

    let _ = fs::remove_dir_all(&dir);
}

#[cfg(any(
    feature = "las",
    feature = "laz",
    feature = "geospatial",
    feature = "shapefile"
))]
fn sample_las_cloud() -> PointCloud {
    let mut p1 = Point::new(1.25, 2.5, -3.75)
        .with_intensity(42.0)
        .with_color(Color::new(1000, 2000, 3000))
        .with_classification(2);
    p1.return_number = Some(1);
    p1.number_of_returns = Some(2);
    p1.gps_time = Some(123456.789);
    p1.scan_angle = Some(-15.0);

    let mut p2 = Point::new(4.0, 5.0, 6.0)
        .with_intensity(7.0)
        .with_color(Color::new(10, 20, 30))
        .with_classification(5);
    p2.return_number = Some(2);
    p2.number_of_returns = Some(2);
    p2.gps_time = Some(123457.0);
    p2.scan_angle = Some(10.0);

    PointCloud::new(vec![p1, p2])
}

#[test]
#[cfg(feature = "las")]
fn las_attributes_roundtrip() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.las");

    let original = sample_las_cloud();
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::Las,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::Las, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), original.points.len());
        for (p_orig, p_dec) in original.points.iter().zip(decoded.points.iter()) {
            approx(p_orig.position.x, p_dec.position.x);
            approx(p_orig.position.y, p_dec.position.y);
            approx(p_orig.position.z, p_dec.position.z);
            assert_eq!(p_orig.intensity.is_some(), p_dec.intensity.is_some());
            if let (Some(i1), Some(i2)) = (p_orig.intensity, p_dec.intensity) {
                assert!((i1 - i2).abs() < 1e-3);
            }
            assert_eq!(p_orig.color, p_dec.color);
            assert_eq!(p_orig.classification, p_dec.classification);
            assert_eq!(p_orig.return_number, p_dec.return_number);
            assert_eq!(p_orig.number_of_returns, p_dec.number_of_returns);
            assert_eq!(p_orig.gps_time, p_dec.gps_time);
            assert_eq!(p_orig.scan_angle, p_dec.scan_angle);
        }
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "laz")]
fn laz_attributes_roundtrip() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.laz");

    let original = sample_las_cloud();
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::Laz,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::Laz, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), original.points.len());
        for (p_orig, p_dec) in original.points.iter().zip(decoded.points.iter()) {
            approx(p_orig.position.x, p_dec.position.x);
            approx(p_orig.position.y, p_dec.position.y);
            approx(p_orig.position.z, p_dec.position.z);
            assert_eq!(p_orig.intensity.is_some(), p_dec.intensity.is_some());
            if let (Some(i1), Some(i2)) = (p_orig.intensity, p_dec.intensity) {
                assert!((i1 - i2).abs() < 1e-3);
            }
            assert_eq!(p_orig.color, p_dec.color);
            assert_eq!(p_orig.classification, p_dec.classification);
            assert_eq!(p_orig.return_number, p_dec.return_number);
            assert_eq!(p_orig.number_of_returns, p_dec.number_of_returns);
            assert_eq!(p_orig.gps_time, p_dec.gps_time);
            assert_eq!(p_orig.scan_angle, p_dec.scan_angle);
        }
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "e57")]
fn e57_attributes_roundtrip() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.e57");

    let mut original = sample_cloud();
    original.metadata.crs_wkt = Some("GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433]]".to_string());
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::E57,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::E57, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), original.points.len());
        assert_eq!(decoded.metadata.crs_wkt, original.metadata.crs_wkt);
        for (p_orig, p_dec) in original.points.iter().zip(decoded.points.iter()) {
            approx(p_orig.position.x, p_dec.position.x);
            approx(p_orig.position.y, p_dec.position.y);
            approx(p_orig.position.z, p_dec.position.z);
            assert_eq!(p_orig.intensity.is_some(), p_dec.intensity.is_some());
            if let (Some(i1), Some(i2)) = (p_orig.intensity, p_dec.intensity) {
                println!("E57 Intensity: original = {}, decoded = {}", i1, i2);
                assert!((i1 - i2).abs() < 1e-3);
            }
            // E57 colors are saved as 8-bit integers internally when we mapped it (color.red >> 8)
            // So we verify that they are within 1 byte precision (approximate)
            if let (Some(c1), Some(c2)) = (p_orig.color, p_dec.color) {
                assert_eq!(c1.red >> 8, c2.red >> 8);
                assert_eq!(c1.green >> 8, c2.green >> 8);
                assert_eq!(c1.blue >> 8, c2.blue >> 8);
            } else {
                assert_eq!(p_orig.color.is_some(), p_dec.color.is_some());
            }
        }
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "geospatial")]
fn geojson_attributes_roundtrip() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.geojson");

    let original = sample_las_cloud();
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::GeoJson,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::GeoJson, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), original.points.len());
        for (p_orig, p_dec) in original.points.iter().zip(decoded.points.iter()) {
            approx(p_orig.position.x, p_dec.position.x);
            approx(p_orig.position.y, p_dec.position.y);
            approx(p_orig.position.z, p_dec.position.z);
            assert_eq!(p_orig.intensity.is_some(), p_dec.intensity.is_some());
            if let (Some(i1), Some(i2)) = (p_orig.intensity, p_dec.intensity) {
                assert!((i1 - i2).abs() < 1e-3);
            }
            if let (Some(c1), Some(c2)) = (p_orig.color, p_dec.color) {
                assert_eq!(c1.red >> 8, c2.red >> 8);
                assert_eq!(c1.green >> 8, c2.green >> 8);
                assert_eq!(c1.blue >> 8, c2.blue >> 8);
            } else {
                assert_eq!(p_orig.color.is_some(), p_dec.color.is_some());
            }
            assert_eq!(p_orig.classification, p_dec.classification);
            assert_eq!(p_orig.return_number, p_dec.return_number);
            assert_eq!(p_orig.number_of_returns, p_dec.number_of_returns);
            assert_eq!(p_orig.gps_time, p_dec.gps_time);
            assert_eq!(p_orig.scan_angle, p_dec.scan_angle);
        }
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "geospatial")]
fn geojson_raw_parsing_test() {
    let raw_geojson = r##"{
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [10.0, 20.0, 30.0]
                },
                "properties": {
                    "intensity": 50.0,
                    "classification": 3,
                    "color": "#ff00ff"
                }
            },
            {
                "type": "Feature",
                "geometry": {
                    "type": "MultiPoint",
                    "coordinates": [
                        [40.0, 50.0, 60.0]
                    ]
                },
                "properties": {
                    "intensity": 10.0
                }
            }
        ]
    }"##;

    let mut cursor = std::io::Cursor::new(raw_geojson.as_bytes());
    let decoded_geom = io::geojson::read(&mut cursor).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), 2);
        approx(decoded.points[0].position.x, 10.0);
        approx(decoded.points[0].position.y, 20.0);
        approx(decoded.points[0].position.z, 30.0);
        assert_eq!(decoded.points[0].intensity, Some(50.0));
        assert_eq!(decoded.points[0].classification, Some(3));
        assert_eq!(decoded.points[0].color, Some(Color::new(65535, 0, 65535)));

        approx(decoded.points[1].position.x, 40.0);
        approx(decoded.points[1].position.y, 50.0);
        approx(decoded.points[1].position.z, 60.0);
        assert_eq!(decoded.points[1].intensity, Some(10.0));
    } else {
        panic!("expected point cloud");
    }
}

#[test]
#[cfg(feature = "geospatial")]
fn geotiff_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.tif");

    let p1 = Point::new(10.0, 20.0, 5.0);
    let p2 = Point::new(20.0, 20.0, 10.0);
    let p3 = Point::new(10.0, 10.0, 15.0);
    let p4 = Point::new(20.0, 10.0, 20.0);
    let original = PointCloud::new(vec![p1, p2, p3, p4]);
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::GeoTiff,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::GeoTiff, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        let min_x = decoded
            .points
            .iter()
            .map(|p| p.position.x)
            .fold(f64::INFINITY, f64::min);
        let max_x = decoded
            .points
            .iter()
            .map(|p| p.position.x)
            .fold(f64::NEG_INFINITY, f64::max);
        let min_y = decoded
            .points
            .iter()
            .map(|p| p.position.y)
            .fold(f64::INFINITY, f64::min);
        let max_y = decoded
            .points
            .iter()
            .map(|p| p.position.y)
            .fold(f64::NEG_INFINITY, f64::max);

        assert!((min_x - 10.0).abs() < 1.0);
        assert!((max_x - 20.0).abs() < 1.0);
        assert!((min_y - 10.0).abs() < 1.0);
        assert!((max_y - 20.0).abs() < 1.0);
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

fn unique_temp_dir() -> PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!("point_formats_test_{}", uuid::Uuid::new_v4()));
    path
}

#[test]
fn asciigrid_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.asc");

    let p1 = Point::new(10.0, 20.0, 5.5);
    let p2 = Point::new(20.0, 20.0, 10.5);
    let p3 = Point::new(10.0, 10.0, 15.5);
    let p4 = Point::new(20.0, 10.0, 20.5);
    let original = PointCloud::new(vec![p1, p2, p3, p4]);
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::AsciiGrid,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::AsciiGrid, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert!(!decoded.points.is_empty());
        let bounds = decoded.bounds().unwrap();
        approx(bounds.min.x, 10.0);
        approx(bounds.min.y, 10.0);
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "dxf")]
fn dxf_point_cloud_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.dxf");

    let original = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)]);
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::Dxf,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::Dxf, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), 2);
        approx(decoded.points[0].position.x, 1.0);
        approx(decoded.points[0].position.y, 2.0);
        approx(decoded.points[0].position.z, 3.0);
        approx(decoded.points[1].position.x, 4.0);
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "dxf")]
fn dxf_mesh_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.dxf");

    let mesh = Mesh::new(
        vec![
            Vertex::new(Vec3::new(0.0, 0.0, 0.0)),
            Vertex::new(Vec3::new(1.0, 0.0, 0.0)),
            Vertex::new(Vec3::new(0.0, 1.0, 0.0)),
        ],
        vec![Face::new(0, 1, 2)],
    );
    let geometry = Geometry::Mesh(mesh);

    io::write_path(
        &file_path,
        Format::Dxf,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::Dxf, &io::NativeOptions::default()).unwrap();
    if let Geometry::Mesh(decoded) = decoded_geom {
        assert_eq!(decoded.faces.len(), 1);
        assert_eq!(decoded.vertices.len(), 3);
        approx(decoded.vertices[1].position.x, 1.0);
    } else {
        panic!("expected mesh");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "shapefile")]
fn shapefile_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.shp");

    let original = sample_las_cloud();
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::Shapefile,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::Shapefile, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), original.points.len());
        for (p_orig, p_dec) in original.points.iter().zip(decoded.points.iter()) {
            approx(p_orig.position.x, p_dec.position.x);
            approx(p_orig.position.y, p_dec.position.y);
            approx(p_orig.position.z, p_dec.position.z);
            assert_eq!(p_orig.intensity, p_dec.intensity);
            assert_eq!(p_orig.classification, p_dec.classification);
            assert_eq!(p_orig.gps_time, p_dec.gps_time);
            assert_eq!(p_orig.scan_angle, p_dec.scan_angle);
            if let (Some(c1), Some(c2)) = (p_orig.color, p_dec.color) {
                assert_eq!(c1.red >> 8, c2.red >> 8);
                assert_eq!(c1.green >> 8, c2.green >> 8);
                assert_eq!(c1.blue >> 8, c2.blue >> 8);
            } else {
                assert_eq!(p_orig.color.is_some(), p_dec.color.is_some());
            }
        }
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "gltf")]
fn gltf_point_cloud_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.gltf");

    let original = PointCloud::new(vec![
        Point::new(1.0, 2.0, 3.0).with_color(Color::new(10000, 20000, 30000)),
        Point::new(4.0, 5.0, 6.0).with_color(Color::new(40000, 50000, 60000)),
    ]);
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::Gltf,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::Gltf, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), 2);
        approx(decoded.points[0].position.x, 1.0);
        approx(decoded.points[0].position.y, 2.0);
        approx(decoded.points[0].position.z, 3.0);
        if let (Some(c1), Some(c2)) = (decoded.points[0].color, original.points[0].color) {
            assert_eq!(c1.red >> 8, c2.red >> 8);
            assert_eq!(c1.green >> 8, c2.green >> 8);
            assert_eq!(c1.blue >> 8, c2.blue >> 8);
        } else {
            panic!("Expected color");
        }
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "gltf")]
fn glb_point_cloud_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.glb");

    let original = PointCloud::new(vec![
        Point::new(1.0, 2.0, 3.0).with_color(Color::new(10000, 20000, 30000)),
        Point::new(4.0, 5.0, 6.0).with_color(Color::new(40000, 50000, 60000)),
    ]);
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::Glb,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::Glb, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), 2);
        approx(decoded.points[0].position.x, 1.0);
        approx(decoded.points[0].position.y, 2.0);
        approx(decoded.points[0].position.z, 3.0);
        if let (Some(c1), Some(c2)) = (decoded.points[0].color, original.points[0].color) {
            assert_eq!(c1.red >> 8, c2.red >> 8);
            assert_eq!(c1.green >> 8, c2.green >> 8);
            assert_eq!(c1.blue >> 8, c2.blue >> 8);
        } else {
            panic!("Expected color");
        }
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "gltf")]
fn gltf_mesh_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.gltf");

    let mesh = Mesh::new(
        vec![
            Vertex::new(Vec3::new(0.0, 0.0, 0.0)),
            Vertex::new(Vec3::new(1.0, 0.0, 0.0)),
            Vertex::new(Vec3::new(0.0, 1.0, 0.0)),
        ],
        vec![Face::new(0, 1, 2)],
    );
    let geometry = Geometry::Mesh(mesh);

    io::write_path(
        &file_path,
        Format::Gltf,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::Gltf, &io::NativeOptions::default()).unwrap();
    if let Geometry::Mesh(decoded) = decoded_geom {
        assert_eq!(decoded.faces.len(), 1);
        assert_eq!(decoded.vertices.len(), 3);
        approx(decoded.vertices[1].position.x, 1.0);
    } else {
        panic!("expected mesh");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "gltf")]
fn glb_mesh_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.glb");

    let mesh = Mesh::new(
        vec![
            Vertex::new(Vec3::new(0.0, 0.0, 0.0)),
            Vertex::new(Vec3::new(1.0, 0.0, 0.0)),
            Vertex::new(Vec3::new(0.0, 1.0, 0.0)),
        ],
        vec![Face::new(0, 1, 2)],
    );
    let geometry = Geometry::Mesh(mesh);

    io::write_path(
        &file_path,
        Format::Glb,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::Glb, &io::NativeOptions::default()).unwrap();
    if let Geometry::Mesh(decoded) = decoded_geom {
        assert_eq!(decoded.faces.len(), 1);
        assert_eq!(decoded.vertices.len(), 3);
        approx(decoded.vertices[1].position.x, 1.0);
    } else {
        panic!("expected mesh");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "gpkg")]
fn gpkg_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.gpkg");

    let original = sample_las_cloud();
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::Gpkg,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::Gpkg, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), original.points.len());
        for (p_orig, p_dec) in original.points.iter().zip(decoded.points.iter()) {
            approx(p_orig.position.x, p_dec.position.x);
            approx(p_orig.position.y, p_dec.position.y);
            approx(p_orig.position.z, p_dec.position.z);
            assert_eq!(p_orig.intensity.is_some(), p_dec.intensity.is_some());
            if let (Some(i1), Some(i2)) = (p_orig.intensity, p_dec.intensity) {
                assert!((i1 - i2).abs() < 1e-3);
            }
            assert_eq!(p_orig.classification, p_dec.classification);
            assert_eq!(p_orig.gps_time, p_dec.gps_time);
            assert_eq!(p_orig.scan_angle, p_dec.scan_angle);
            if let (Some(c1), Some(c2)) = (p_orig.color, p_dec.color) {
                assert_eq!(c1.red >> 8, c2.red >> 8);
                assert_eq!(c1.green >> 8, c2.green >> 8);
                assert_eq!(c1.blue >> 8, c2.blue >> 8);
            } else {
                assert_eq!(p_orig.color.is_some(), p_dec.color.is_some());
            }
        }
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "robotics")]
fn pc2_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.pc2");

    let original = sample_las_cloud();
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::PointCloud2,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom = io::read_path(
        &file_path,
        Format::PointCloud2,
        &io::NativeOptions::default(),
    )
    .unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), original.points.len());
        let expected_gps = original.points.first().unwrap().gps_time.unwrap();
        for (p_orig, p_dec) in original.points.iter().zip(decoded.points.iter()) {
            approx(p_orig.position.x, p_dec.position.x);
            approx(p_orig.position.y, p_dec.position.y);
            approx(p_orig.position.z, p_dec.position.z);
            assert_eq!(p_orig.intensity.is_some(), p_dec.intensity.is_some());
            if let (Some(i1), Some(i2)) = (p_orig.intensity, p_dec.intensity) {
                assert!((i1 - i2).abs() < 1e-3);
            }
            assert_eq!(p_orig.classification, p_dec.classification);
            assert!((p_dec.gps_time.unwrap() - expected_gps).abs() < 1e-3);
            if let (Some(c1), Some(c2)) = (p_orig.color, p_dec.color) {
                assert_eq!(c1.red >> 8, c2.red >> 8);
                assert_eq!(c1.green >> 8, c2.green >> 8);
                assert_eq!(c1.blue >> 8, c2.blue >> 8);
            } else {
                assert_eq!(p_orig.color.is_some(), p_dec.color.is_some());
            }
        }
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "robotics")]
fn rosbag_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.bag");

    let original = sample_las_cloud();
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::RosBag,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::RosBag, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), original.points.len());
        let expected_gps = original.points.first().unwrap().gps_time.unwrap();
        for (p_orig, p_dec) in original.points.iter().zip(decoded.points.iter()) {
            approx(p_orig.position.x, p_dec.position.x);
            approx(p_orig.position.y, p_dec.position.y);
            approx(p_orig.position.z, p_dec.position.z);
            assert_eq!(p_orig.intensity.is_some(), p_dec.intensity.is_some());
            if let (Some(i1), Some(i2)) = (p_orig.intensity, p_dec.intensity) {
                assert!((i1 - i2).abs() < 1e-3);
            }
            assert_eq!(p_orig.classification, p_dec.classification);
            assert!((p_dec.gps_time.unwrap() - expected_gps).abs() < 1e-3);
            if let (Some(c1), Some(c2)) = (p_orig.color, p_dec.color) {
                assert_eq!(c1.red >> 8, c2.red >> 8);
                assert_eq!(c1.green >> 8, c2.green >> 8);
                assert_eq!(c1.blue >> 8, c2.blue >> 8);
            } else {
                assert_eq!(p_orig.color.is_some(), p_dec.color.is_some());
            }
        }
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "robotics")]
fn ros2bag_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.db3");

    let original = sample_las_cloud();
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::Ros2Bag,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::Ros2Bag, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), original.points.len());
        let expected_gps = original.points.first().unwrap().gps_time.unwrap();
        for (p_orig, p_dec) in original.points.iter().zip(decoded.points.iter()) {
            approx(p_orig.position.x, p_dec.position.x);
            approx(p_orig.position.y, p_dec.position.y);
            approx(p_orig.position.z, p_dec.position.z);
            assert_eq!(p_orig.intensity.is_some(), p_dec.intensity.is_some());
            if let (Some(i1), Some(i2)) = (p_orig.intensity, p_dec.intensity) {
                assert!((i1 - i2).abs() < 1e-3);
            }
            assert_eq!(p_orig.classification, p_dec.classification);
            assert!((p_dec.gps_time.unwrap() - expected_gps).abs() < 1e-3);
            if let (Some(c1), Some(c2)) = (p_orig.color, p_dec.color) {
                assert_eq!(c1.red >> 8, c2.red >> 8);
                assert_eq!(c1.green >> 8, c2.green >> 8);
                assert_eq!(c1.blue >> 8, c2.blue >> 8);
            } else {
                assert_eq!(p_orig.color.is_some(), p_dec.color.is_some());
            }
        }
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "sensor")]
fn vendorraw_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.raw");

    let original = sample_las_cloud();
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::VendorRaw,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::VendorRaw, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), original.points.len());
        for (p_orig, p_dec) in original.points.iter().zip(decoded.points.iter()) {
            approx(p_orig.position.x, p_dec.position.x);
            approx(p_orig.position.y, p_dec.position.y);
            approx(p_orig.position.z, p_dec.position.z);
            assert_eq!(p_orig.intensity.is_some(), p_dec.intensity.is_some());
            if let (Some(i1), Some(i2)) = (p_orig.intensity, p_dec.intensity) {
                assert!((i1 - i2).abs() < 1e-3);
            }
            assert_eq!(p_orig.classification, p_dec.classification);
            assert_eq!(p_orig.gps_time.is_some(), p_dec.gps_time.is_some());
            if let (Some(g1), Some(g2)) = (p_orig.gps_time, p_dec.gps_time) {
                approx(g1, g2);
            }
            assert_eq!(p_orig.normal.is_some(), p_dec.normal.is_some());
            if let (Some(n1), Some(n2)) = (p_orig.normal, p_dec.normal) {
                assert!((n1.x - n2.x).abs() < 1e-5);
                assert!((n1.y - n2.y).abs() < 1e-5);
                assert!((n1.z - n2.z).abs() < 1e-5);
            }
            if let (Some(c1), Some(c2)) = (p_orig.color, p_dec.color) {
                assert_eq!(c1.red >> 8, c2.red >> 8);
                assert_eq!(c1.green >> 8, c2.green >> 8);
                assert_eq!(c1.blue >> 8, c2.blue >> 8);
            } else {
                assert_eq!(p_orig.color.is_some(), p_dec.color.is_some());
            }
        }
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "sensor")]
fn udppackets_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.udp");

    let original = sample_las_cloud();
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::UdpPackets,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom = io::read_path(
        &file_path,
        Format::UdpPackets,
        &io::NativeOptions::default(),
    )
    .unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), original.points.len());
        for (p_orig, p_dec) in original.points.iter().zip(decoded.points.iter()) {
            assert!((p_orig.position.x - p_dec.position.x).abs() < 1e-6);
            assert!((p_orig.position.y - p_dec.position.y).abs() < 1e-6);
            assert!((p_orig.position.z - p_dec.position.z).abs() < 1e-6);
            assert_eq!(p_orig.intensity.is_some(), p_dec.intensity.is_some());
            if let (Some(i1), Some(i2)) = (p_orig.intensity, p_dec.intensity) {
                assert!((i1 - i2).abs() < 1e-3);
            }
            assert_eq!(p_orig.classification, p_dec.classification);
            assert_eq!(p_orig.gps_time.is_some(), p_dec.gps_time.is_some());
            if let (Some(g1), Some(g2)) = (p_orig.gps_time, p_dec.gps_time) {
                approx(g1, g2);
            }
            if let (Some(c1), Some(c2)) = (p_orig.color, p_dec.color) {
                assert_eq!(c1.red >> 8, c2.red >> 8);
                assert_eq!(c1.green >> 8, c2.green >> 8);
                assert_eq!(c1.blue >> 8, c2.blue >> 8);
            } else {
                assert_eq!(p_orig.color.is_some(), p_dec.color.is_some());
            }
        }
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}

#[test]
#[cfg(feature = "sensor")]
fn pcap_roundtrip_test() {
    let dir = unique_temp_dir();
    fs::create_dir_all(&dir).unwrap();
    let file_path = dir.join("test.pcap");

    let original = sample_las_cloud();
    let geometry = Geometry::PointCloud(original.clone());

    io::write_path(
        &file_path,
        Format::Pcap,
        &geometry,
        &io::NativeOptions::default(),
    )
    .unwrap();

    let decoded_geom =
        io::read_path(&file_path, Format::Pcap, &io::NativeOptions::default()).unwrap();
    if let Geometry::PointCloud(decoded) = decoded_geom {
        assert_eq!(decoded.points.len(), original.points.len());
        for (p_orig, p_dec) in original.points.iter().zip(decoded.points.iter()) {
            assert!((p_orig.position.x - p_dec.position.x).abs() < 1e-6);
            assert!((p_orig.position.y - p_dec.position.y).abs() < 1e-6);
            assert!((p_orig.position.z - p_dec.position.z).abs() < 1e-6);
            assert_eq!(p_orig.intensity.is_some(), p_dec.intensity.is_some());
            if let (Some(i1), Some(i2)) = (p_orig.intensity, p_dec.intensity) {
                assert!((i1 - i2).abs() < 1e-3);
            }
            assert_eq!(p_orig.classification, p_dec.classification);
            assert_eq!(p_orig.gps_time.is_some(), p_dec.gps_time.is_some());
            if let (Some(g1), Some(g2)) = (p_orig.gps_time, p_dec.gps_time) {
                assert!((g1 - g2).abs() < 1e-5);
            }
            if let (Some(c1), Some(c2)) = (p_orig.color, p_dec.color) {
                assert_eq!(c1.red >> 8, c2.red >> 8);
                assert_eq!(c1.green >> 8, c2.green >> 8);
                assert_eq!(c1.blue >> 8, c2.blue >> 8);
            } else {
                assert_eq!(p_orig.color.is_some(), p_dec.color.is_some());
            }
        }
    } else {
        panic!("expected point cloud");
    }

    let _ = fs::remove_dir_all(&dir);
}
