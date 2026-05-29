use crate::error::{Error, Result};
use crate::types::{Color, Geometry, Point, PointCloud};
use rusqlite::{params, Connection};
use std::path::Path;

/// Reads a point cloud from a GeoPackage file.
pub fn read(path: impl AsRef<Path>) -> Result<Geometry> {
    let conn = Connection::open(path)
        .map_err(|e| Error::invalid(format!("Failed to open GeoPackage SQLite: {}", e)))?;

    let mut stmt = conn
        .prepare("SELECT table_name, column_name FROM gpkg_geometry_columns")
        .map_err(|e| Error::invalid(format!("Failed to query gpkg_geometry_columns: {}", e)))?;
    let mut rows = stmt.query([])?;

    let mut table_name = "points".to_string();
    let mut geom_column = "geom".to_string();
    if let Some(row) = rows.next()? {
        table_name = row.get(0)?;
        geom_column = row.get(1)?;
    }

    let mut has_intensity = false;
    let mut has_class = false;
    let mut has_color = false;
    let mut has_gps_time = false;
    let mut has_scan_angle = false;

    let mut class_col_name = "class".to_string();
    let mut color_col_name = "color".to_string();

    let mut info_stmt = conn.prepare(&format!("PRAGMA table_info({})", table_name))?;
    let mut info_rows = info_stmt.query([])?;
    while let Some(row) = info_rows.next()? {
        let col_name: String = row.get(1)?;
        match col_name.as_str() {
            "intensity" => has_intensity = true,
            "class" => {
                has_class = true;
                class_col_name = "class".to_string();
            }
            "classification" => {
                has_class = true;
                class_col_name = "classification".to_string();
            }
            "color" => {
                has_color = true;
                color_col_name = "color".to_string();
            }
            "hex_color" => {
                has_color = true;
                color_col_name = "hex_color".to_string();
            }
            "gps_time" => has_gps_time = true,
            "scan_angle" => has_scan_angle = true,
            _ => {}
        }
    }

    let mut query_parts = vec![geom_column.clone()];
    if has_intensity {
        query_parts.push("intensity".to_string());
    }
    if has_class {
        query_parts.push(class_col_name.clone());
    }
    if has_color {
        query_parts.push(color_col_name.clone());
    }
    if has_gps_time {
        query_parts.push("gps_time".to_string());
    }
    if has_scan_angle {
        query_parts.push("scan_angle".to_string());
    }

    let query_str = format!("SELECT {} FROM {}", query_parts.join(", "), table_name);
    let mut stmt = conn.prepare(&query_str)?;
    let mut rows = stmt.query([])?;

    let mut points = Vec::new();
    while let Some(row) = rows.next()? {
        let geom_blob: Vec<u8> = row.get(0)?;
        let (x, y, z) = parse_gpb_geometry(&geom_blob)?;
        let mut pt = Point::new(x, y, z);

        let mut idx = 1;
        if has_intensity {
            let val: Option<f64> = row.get(idx)?;
            if let Some(v) = val {
                pt.intensity = Some(v as f32);
            }
            idx += 1;
        }
        if has_class {
            let val: Option<i64> = row.get(idx)?;
            if let Some(v) = val {
                pt.classification = Some(v as u8);
            }
            idx += 1;
        }
        if has_color {
            let val: Option<String> = row.get(idx)?;
            if let Some(hex_str) = val {
                if let Some(c) = parse_hex_color(&hex_str) {
                    pt.color = Some(c);
                }
            }
            idx += 1;
        }
        if has_gps_time {
            let val: Option<f64> = row.get(idx)?;
            pt.gps_time = val;
            idx += 1;
        }
        if has_scan_angle {
            let val: Option<f64> = row.get(idx)?;
            if let Some(v) = val {
                pt.scan_angle = Some(v as f32);
            }
        }

        points.push(pt);
    }

    Ok(Geometry::PointCloud(PointCloud::new(points)))
}

/// Writes a point cloud to a GeoPackage file.
pub fn write(path: impl AsRef<Path>, cloud: &PointCloud) -> Result<()> {
    if cloud.points.is_empty() {
        return Err(Error::invalid(
            "Cannot write empty point cloud to GeoPackage",
        ));
    }

    // Delete existing file if any
    if path.as_ref().exists() {
        let _ = std::fs::remove_file(path.as_ref());
    }

    let mut conn = Connection::open(path)
        .map_err(|e| Error::invalid(format!("Failed to open GeoPackage SQLite: {}", e)))?;

    // Create GeoPackage metadata tables
    conn.execute(
        "CREATE TABLE gpkg_spatial_ref_sys (
            srs_name TEXT NOT NULL,
            srs_id INTEGER NOT NULL PRIMARY KEY,
            organization TEXT NOT NULL,
            organization_coordsys_id INTEGER NOT NULL,
            definition TEXT NOT NULL,
            description TEXT
        )",
        [],
    )?;
    conn.execute(
        "CREATE TABLE gpkg_contents (
            table_name TEXT NOT NULL PRIMARY KEY,
            data_type TEXT NOT NULL,
            identifier TEXT UNIQUE,
            description TEXT,
            last_change TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ','now')),
            min_x DOUBLE,
            min_y DOUBLE,
            max_x DOUBLE,
            max_y DOUBLE,
            srs_id INTEGER,
            CONSTRAINT fk_gc_r_srs_id FOREIGN KEY (srs_id) REFERENCES gpkg_spatial_ref_sys(srs_id)
        )",
        [],
    )?;
    conn.execute(
        "CREATE TABLE gpkg_geometry_columns (
            table_name TEXT NOT NULL,
            column_name TEXT NOT NULL,
            geometry_type_name TEXT NOT NULL,
            srs_id INTEGER NOT NULL,
            z INTEGER NOT NULL,
            m INTEGER NOT NULL,
            CONSTRAINT pk_geom_cols PRIMARY KEY (table_name, column_name),
            CONSTRAINT fk_gc_rel_r_3 FOREIGN KEY (srs_id) REFERENCES gpkg_spatial_ref_sys(srs_id)
        )",
        [],
    )?;

    // Populate SRS (standard EPSG:4326 and EPSG:0)
    conn.execute(
        "INSERT INTO gpkg_spatial_ref_sys VALUES ('Undefined cartesian coordinate reference system', -1, 'NONE', -1, 'undefined', 'undefined')",
        [],
    )?;
    conn.execute(
        "INSERT INTO gpkg_spatial_ref_sys VALUES ('Undefined geographic coordinate reference system', 0, 'NONE', 0, 'undefined', 'undefined')",
        [],
    )?;
    conn.execute(
        "INSERT INTO gpkg_spatial_ref_sys VALUES ('WGS 84', 4326, 'EPSG', 4326, 'GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433]]', 'world')",
        [],
    )?;

    // Detect attributes
    let has_intensity = cloud.has_intensity();
    let has_classification = cloud.has_classification();
    let has_color = cloud.has_color();
    let has_gps_time = cloud.has_gps_time();
    let has_scan_angle = cloud.points.iter().any(|p| p.scan_angle.is_some());

    // Create features table
    let mut columns_sql = vec![
        "id INTEGER PRIMARY KEY AUTOINCREMENT".to_string(),
        "geom BLOB NOT NULL".to_string(),
    ];
    if has_intensity {
        columns_sql.push("intensity DOUBLE".to_string());
    }
    if has_classification {
        columns_sql.push("class INTEGER".to_string());
    }
    if has_color {
        columns_sql.push("color TEXT".to_string());
    }
    if has_gps_time {
        columns_sql.push("gps_time DOUBLE".to_string());
    }
    if has_scan_angle {
        columns_sql.push("scan_angle DOUBLE".to_string());
    }

    let create_table_sql = format!("CREATE TABLE points ({})", columns_sql.join(", "));
    conn.execute(&create_table_sql, [])?;

    // Bounding Box
    let bounds = cloud.bounds().unwrap_or(crate::types::Bounds3::empty());

    // Register the table in gpkg_contents
    conn.execute(
        "INSERT INTO gpkg_contents (table_name, data_type, identifier, min_x, min_y, max_x, max_y, srs_id)
         VALUES ('points', 'features', 'points', ?1, ?2, ?3, ?4, 4326)",
        params![bounds.min.x, bounds.min.y, bounds.max.x, bounds.max.y],
    )?;

    // Register geometry column
    conn.execute(
        "INSERT INTO gpkg_geometry_columns (table_name, column_name, geometry_type_name, srs_id, z, m)
         VALUES ('points', 'geom', 'POINTZ', 4326, 1, 0)",
        [],
    )?;

    // Insert Points
    let tx = conn.transaction()?;
    {
        let mut insert_cols = vec!["geom".to_string()];
        let mut insert_placeholders = vec!["?1".to_string()];
        let mut col_idx = 2;

        if has_intensity {
            insert_cols.push("intensity".to_string());
            insert_placeholders.push(format!("?{}", col_idx));
            col_idx += 1;
        }
        if has_classification {
            insert_cols.push("class".to_string());
            insert_placeholders.push(format!("?{}", col_idx));
            col_idx += 1;
        }
        if has_color {
            insert_cols.push("color".to_string());
            insert_placeholders.push(format!("?{}", col_idx));
            col_idx += 1;
        }
        if has_gps_time {
            insert_cols.push("gps_time".to_string());
            insert_placeholders.push(format!("?{}", col_idx));
            col_idx += 1;
        }
        if has_scan_angle {
            insert_cols.push("scan_angle".to_string());
            insert_placeholders.push(format!("?{}", col_idx));
        }

        let insert_sql = format!(
            "INSERT INTO points ({}) VALUES ({})",
            insert_cols.join(", "),
            insert_placeholders.join(", ")
        );

        let mut stmt = tx.prepare(&insert_sql)?;

        for p in &cloud.points {
            let mut gpb = Vec::with_capacity(37);

            // Header (8 bytes): magic 'GP', version 0, flags 1 (little endian, no envelope), srs_id 4326
            gpb.extend_from_slice(b"GP");
            gpb.push(0x00);
            gpb.push(0x01);
            gpb.extend_from_slice(&4326_i32.to_le_bytes());

            // WKB PointZ payload: byte order 1, type 1001, coords x, y, z
            gpb.push(0x01);
            gpb.extend_from_slice(&1001_u32.to_le_bytes());
            gpb.extend_from_slice(&p.position.x.to_le_bytes());
            gpb.extend_from_slice(&p.position.y.to_le_bytes());
            gpb.extend_from_slice(&p.position.z.to_le_bytes());

            let mut params_vec: Vec<Box<dyn rusqlite::ToSql>> = vec![Box::new(gpb)];

            if has_intensity {
                params_vec.push(Box::new(p.intensity.map(|i| i as f64)));
            }
            if has_classification {
                params_vec.push(Box::new(p.classification.map(|c| c as i64)));
            }
            if has_color {
                let hex = p
                    .color
                    .map(|c| format!("#{:02x}{:02x}{:02x}", c.red >> 8, c.green >> 8, c.blue >> 8));
                params_vec.push(Box::new(hex));
            }
            if has_gps_time {
                params_vec.push(Box::new(p.gps_time));
            }
            if has_scan_angle {
                params_vec.push(Box::new(p.scan_angle.map(|a| a as f64)));
            }

            let params_ref: Vec<&dyn rusqlite::ToSql> =
                params_vec.iter().map(|b| b.as_ref()).collect();
            stmt.execute(&*params_ref)?;
        }
    }
    tx.commit()?;

    Ok(())
}

fn parse_gpb_geometry(blob: &[u8]) -> Result<(f64, f64, f64)> {
    if blob.len() < 8 {
        return Err(Error::invalid("GeoPackage binary geometry blob too short"));
    }

    if blob[0] != 0x47 || blob[1] != 0x50 {
        return Err(Error::invalid("Invalid GeoPackage binary geometry magic"));
    }

    let flags = blob[3];
    let envelope_type = (flags >> 1) & 0x07;

    let envelope_size = match envelope_type {
        0 => 0,
        1 => 32,
        2 => 48,
        3 => 48,
        4 => 64,
        _ => {
            return Err(Error::invalid(format!(
                "Invalid GeoPackage binary geometry envelope type: {}",
                envelope_type
            )))
        }
    };

    let wkb_offset = 8 + envelope_size;
    if blob.len() < wkb_offset + 5 {
        return Err(Error::invalid(
            "GeoPackage binary geometry blob contains no WKB payload",
        ));
    }

    let wkb = &blob[wkb_offset..];
    let wkb_is_little = wkb[0] != 0;

    let wkb_type = if wkb_is_little {
        u32::from_le_bytes(wkb[1..5].try_into().unwrap())
    } else {
        u32::from_be_bytes(wkb[1..5].try_into().unwrap())
    };

    match wkb_type {
        1 => {
            if wkb.len() < 21 {
                return Err(Error::invalid("WKB Point too short"));
            }
            let (x, y) = if wkb_is_little {
                (
                    f64::from_le_bytes(wkb[5..13].try_into().unwrap()),
                    f64::from_le_bytes(wkb[13..21].try_into().unwrap()),
                )
            } else {
                (
                    f64::from_be_bytes(wkb[5..13].try_into().unwrap()),
                    f64::from_be_bytes(wkb[13..21].try_into().unwrap()),
                )
            };
            Ok((x, y, 0.0))
        }
        1001 => {
            if wkb.len() < 29 {
                return Err(Error::invalid("WKB PointZ too short"));
            }
            let (x, y, z) = if wkb_is_little {
                (
                    f64::from_le_bytes(wkb[5..13].try_into().unwrap()),
                    f64::from_le_bytes(wkb[13..21].try_into().unwrap()),
                    f64::from_le_bytes(wkb[21..29].try_into().unwrap()),
                )
            } else {
                (
                    f64::from_be_bytes(wkb[5..13].try_into().unwrap()),
                    f64::from_be_bytes(wkb[13..21].try_into().unwrap()),
                    f64::from_be_bytes(wkb[21..29].try_into().unwrap()),
                )
            };
            Ok((x, y, z))
        }
        _ => Err(Error::invalid(format!(
            "Unsupported geometry type in WKB: {}",
            wkb_type
        ))),
    }
}

fn parse_hex_color(s: &str) -> Option<Color> {
    let s = s.trim_start_matches('#');
    if s.len() == 6 || s.len() == 8 {
        let r = u8::from_str_radix(&s[0..2], 16).ok()? as u16;
        let g = u8::from_str_radix(&s[2..4], 16).ok()? as u16;
        let b = u8::from_str_radix(&s[4..6], 16).ok()? as u16;
        Some(Color::new(r * 257, g * 257, b * 257))
    } else {
        None
    }
}
