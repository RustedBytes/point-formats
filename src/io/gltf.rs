use crate::error::{Error, Result};
use crate::types::{Color, Face, Geometry, Mesh, Point, PointCloud, Vec3, Vertex};
use std::io::Write;
use std::path::Path;

/// Reads a point cloud or mesh from a glTF or GLB file.
pub fn read(path: impl AsRef<Path>) -> Result<Geometry> {
    let (document, buffers, _) =
        gltf::import(path).map_err(|e| Error::invalid(format!("Failed to parse glTF: {}", e)))?;

    let mut points = Vec::new();
    let mut vertices = Vec::new();
    let mut faces = Vec::new();

    for mesh in document.meshes() {
        for primitive in mesh.primitives() {
            let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()]));

            // Try to extract positions
            let mut prim_positions = Vec::new();
            if let Some(pos_iter) = reader.read_positions() {
                for pos in pos_iter {
                    prim_positions.push(Vec3::new(pos[0] as f64, pos[1] as f64, pos[2] as f64));
                }
            }

            if prim_positions.is_empty() {
                continue;
            }

            // Try to extract colors
            let mut prim_colors = Vec::new();
            if let Some(color_iter) = reader.read_colors(0) {
                for col in color_iter.into_rgba_f32() {
                    let r = (col[0].clamp(0.0, 1.0) * 65535.0).round() as u16;
                    let g = (col[1].clamp(0.0, 1.0) * 65535.0).round() as u16;
                    let b = (col[2].clamp(0.0, 1.0) * 65535.0).round() as u16;
                    prim_colors.push(Color::new(r, g, b));
                }
            }

            let mode = primitive.mode();
            if mode == gltf::mesh::Mode::Points {
                // Point Cloud primitive
                for (i, pos) in prim_positions.into_iter().enumerate() {
                    let mut pt = Point::new(pos.x, pos.y, pos.z);
                    if i < prim_colors.len() {
                        pt.color = Some(prim_colors[i]);
                    }
                    points.push(pt);
                }
            } else {
                // Mesh primitive
                let base_vertex_idx = vertices.len();

                // Append vertices
                for (i, pos) in prim_positions.into_iter().enumerate() {
                    let mut vertex = Vertex::new(pos);
                    if i < prim_colors.len() {
                        vertex.color = Some(prim_colors[i]);
                    }
                    vertices.push(vertex);
                }

                // Read indices
                let mut prim_indices = Vec::new();
                if let Some(indices_iter) = reader.read_indices() {
                    for idx in indices_iter.into_u32() {
                        prim_indices.push(idx as usize + base_vertex_idx);
                    }
                } else {
                    // Non-indexed: use sequential indices
                    let count = vertices.len() - base_vertex_idx;
                    for idx in 0..count {
                        prim_indices.push(idx + base_vertex_idx);
                    }
                }

                // Map topology mode to triangle faces
                match mode {
                    gltf::mesh::Mode::Triangles => {
                        for chunk in prim_indices.chunks_exact(3) {
                            faces.push(Face::new(chunk[0], chunk[1], chunk[2]));
                        }
                    }
                    gltf::mesh::Mode::TriangleStrip => {
                        if prim_indices.len() >= 3 {
                            for i in 0..(prim_indices.len() - 2) {
                                let (a, b, c) = if i % 2 == 0 {
                                    (prim_indices[i], prim_indices[i + 1], prim_indices[i + 2])
                                } else {
                                    (prim_indices[i + 1], prim_indices[i], prim_indices[i + 2])
                                };
                                faces.push(Face::new(a, b, c));
                            }
                        }
                    }
                    gltf::mesh::Mode::TriangleFan if prim_indices.len() >= 3 => {
                        let root = prim_indices[0];
                        for i in 1..(prim_indices.len() - 1) {
                            faces.push(Face::new(root, prim_indices[i], prim_indices[i + 1]));
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    if !faces.is_empty() {
        Ok(Geometry::Mesh(Mesh::new(vertices, faces)))
    } else if !points.is_empty() {
        Ok(Geometry::PointCloud(PointCloud::new(points)))
    } else {
        Err(Error::invalid(
            "Empty glTF file (no points or mesh triangles found)",
        ))
    }
}

/// Writes a point cloud or mesh to a glTF file.
pub fn write_gltf(path: impl AsRef<Path>, geometry: &Geometry) -> Result<()> {
    let (gltf_json, bin_data) = build_gltf_json_and_bin(geometry)?;

    let mut gltf_json = gltf_json;
    let base64_str = encode_base64(&bin_data);
    gltf_json["buffers"][0]["uri"] = serde_json::json!(format!(
        "data:application/octet-stream;base64,{}",
        base64_str
    ));

    let writer = std::fs::File::create(path)?;
    serde_json::to_writer_pretty(writer, &gltf_json)
        .map_err(|e| Error::invalid(format!("Failed to serialize glTF JSON: {}", e)))?;

    Ok(())
}

/// Writes a point cloud or mesh to a GLB file.
pub fn write_glb(path: impl AsRef<Path>, geometry: &Geometry) -> Result<()> {
    let (gltf_json, bin_data) = build_gltf_json_and_bin(geometry)?;

    let json_bytes = serde_json::to_vec(&gltf_json)
        .map_err(|e| Error::invalid(format!("Failed to serialize GLB JSON: {}", e)))?;

    let json_padding = (4 - (json_bytes.len() % 4)) % 4;
    let mut padded_json = json_bytes;
    padded_json.resize(padded_json.len() + json_padding, 0x20);

    let bin_padding = (4 - (bin_data.len() % 4)) % 4;
    let mut padded_bin = bin_data;
    padded_bin.resize(padded_bin.len() + bin_padding, 0x00);

    let file = std::fs::File::create(path)?;
    let mut writer = std::io::BufWriter::new(file);

    // GLB Header
    writer.write_all(b"glTF")?;
    writer.write_all(&2_u32.to_le_bytes())?;
    let total_len = 12 + 8 + padded_json.len() + 8 + padded_bin.len();
    writer.write_all(&(total_len as u32).to_le_bytes())?;

    // JSON Chunk
    writer.write_all(&(padded_json.len() as u32).to_le_bytes())?;
    writer.write_all(b"JSON")?;
    writer.write_all(&padded_json)?;

    // BIN Chunk
    writer.write_all(&(padded_bin.len() as u32).to_le_bytes())?;
    writer.write_all(b"BIN\0")?;
    writer.write_all(&padded_bin)?;

    Ok(())
}

fn build_gltf_json_and_bin(geometry: &Geometry) -> Result<(serde_json::Value, Vec<u8>)> {
    let mut bin_data = Vec::new();

    let (num_vertices, has_color) = match geometry {
        Geometry::PointCloud(cloud) => (cloud.points.len(), cloud.has_color()),
        Geometry::Mesh(mesh) => (
            mesh.vertices.len(),
            mesh.vertices.iter().any(|v| v.color.is_some()),
        ),
    };

    let mut min_pos = [f64::MAX, f64::MAX, f64::MAX];
    let mut max_pos = [f64::MIN, f64::MIN, f64::MIN];

    // Write Positions
    let pos_offset = bin_data.len();
    match geometry {
        Geometry::PointCloud(cloud) => {
            for p in &cloud.points {
                let x = p.position.x as f32;
                let y = p.position.y as f32;
                let z = p.position.z as f32;
                bin_data.extend_from_slice(&x.to_le_bytes());
                bin_data.extend_from_slice(&y.to_le_bytes());
                bin_data.extend_from_slice(&z.to_le_bytes());

                min_pos[0] = min_pos[0].min(p.position.x);
                min_pos[1] = min_pos[1].min(p.position.y);
                min_pos[2] = min_pos[2].min(p.position.z);
                max_pos[0] = max_pos[0].max(p.position.x);
                max_pos[1] = max_pos[1].max(p.position.y);
                max_pos[2] = max_pos[2].max(p.position.z);
            }
        }
        Geometry::Mesh(mesh) => {
            for v in &mesh.vertices {
                let x = v.position.x as f32;
                let y = v.position.y as f32;
                let z = v.position.z as f32;
                bin_data.extend_from_slice(&x.to_le_bytes());
                bin_data.extend_from_slice(&y.to_le_bytes());
                bin_data.extend_from_slice(&z.to_le_bytes());

                min_pos[0] = min_pos[0].min(v.position.x);
                min_pos[1] = min_pos[1].min(v.position.y);
                min_pos[2] = min_pos[2].min(v.position.z);
                max_pos[0] = max_pos[0].max(v.position.x);
                max_pos[1] = max_pos[1].max(v.position.y);
                max_pos[2] = max_pos[2].max(v.position.z);
            }
        }
    }
    let pos_len = bin_data.len() - pos_offset;

    // Write Colors
    let mut col_offset = 0;
    let mut col_len = 0;
    if has_color {
        col_offset = bin_data.len();
        match geometry {
            Geometry::PointCloud(cloud) => {
                for p in &cloud.points {
                    let c = p.color.unwrap_or(Color::new(0, 0, 0));
                    let r = c.red as f32 / 65535.0;
                    let g = c.green as f32 / 65535.0;
                    let b = c.blue as f32 / 65535.0;
                    bin_data.extend_from_slice(&r.to_le_bytes());
                    bin_data.extend_from_slice(&g.to_le_bytes());
                    bin_data.extend_from_slice(&b.to_le_bytes());
                }
            }
            Geometry::Mesh(mesh) => {
                for v in &mesh.vertices {
                    let c = v.color.unwrap_or(Color::new(0, 0, 0));
                    let r = c.red as f32 / 65535.0;
                    let g = c.green as f32 / 65535.0;
                    let b = c.blue as f32 / 65535.0;
                    bin_data.extend_from_slice(&r.to_le_bytes());
                    bin_data.extend_from_slice(&g.to_le_bytes());
                    bin_data.extend_from_slice(&b.to_le_bytes());
                }
            }
        }
        col_len = bin_data.len() - col_offset;
    }

    // Write Indices
    let mut idx_offset = 0;
    let mut idx_len = 0;
    let mut num_indices = 0;
    if let Geometry::Mesh(mesh) = geometry {
        idx_offset = bin_data.len();
        for face in &mesh.faces {
            for &idx in &face.indices {
                let idx_u32 = idx as u32;
                bin_data.extend_from_slice(&idx_u32.to_le_bytes());
                num_indices += 1;
            }
        }
        idx_len = bin_data.len() - idx_offset;
    }

    let mut json_accessors = vec![serde_json::json!({
        "bufferView": 0,
        "byteOffset": 0,
        "componentType": 5126, // FLOAT
        "count": num_vertices,
        "type": "VEC3",
        "max": [max_pos[0] as f32, max_pos[1] as f32, max_pos[2] as f32],
        "min": [min_pos[0] as f32, min_pos[1] as f32, min_pos[2] as f32]
    })];
    let mut json_buffer_views = vec![serde_json::json!({
        "buffer": 0,
        "byteOffset": pos_offset,
        "byteLength": pos_len,
        "target": 34962 // ARRAY_BUFFER
    })];
    let mut json_attributes = serde_json::json!({
        "POSITION": 0
    });

    let mut accessor_counter = 1;
    let mut buffer_view_counter = 1;

    if has_color {
        json_accessors.push(serde_json::json!({
            "bufferView": buffer_view_counter,
            "byteOffset": 0,
            "componentType": 5126, // FLOAT
            "count": num_vertices,
            "type": "VEC3"
        }));
        json_buffer_views.push(serde_json::json!({
            "buffer": 0,
            "byteOffset": col_offset,
            "byteLength": col_len,
            "target": 34962
        }));
        json_attributes["COLOR_0"] = serde_json::json!(accessor_counter);

        accessor_counter += 1;
        buffer_view_counter += 1;
    }

    let mut primitive_json = serde_json::json!({
        "attributes": json_attributes,
        "mode": match geometry {
            Geometry::PointCloud(_) => 0,
            Geometry::Mesh(_) => 4,
        }
    });

    if let Geometry::Mesh(_) = geometry {
        json_accessors.push(serde_json::json!({
            "bufferView": buffer_view_counter,
            "byteOffset": 0,
            "componentType": 5125, // UNSIGNED_INT
            "count": num_indices,
            "type": "SCALAR"
        }));
        json_buffer_views.push(serde_json::json!({
            "buffer": 0,
            "byteOffset": idx_offset,
            "byteLength": idx_len,
            "target": 34963 // ELEMENT_ARRAY_BUFFER
        }));
        primitive_json["indices"] = serde_json::json!(accessor_counter);
    }

    let gltf_json = serde_json::json!({
        "asset": {
            "version": "2.0",
            "generator": "lidar-format-convert"
        },
        "scene": 0,
        "scenes": [
            {
                "nodes": [0]
            }
        ],
        "nodes": [
            {
                "mesh": 0
            }
        ],
        "meshes": [
            {
                "primitives": [
                    primitive_json
                ]
            }
        ],
        "accessors": json_accessors,
        "bufferViews": json_buffer_views,
        "buffers": [
            {
                "byteLength": bin_data.len()
            }
        ]
    });

    Ok((gltf_json, bin_data))
}

fn encode_base64(data: &[u8]) -> String {
    const CHARSET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut result = String::with_capacity(data.len().div_ceil(3) * 4);
    for chunk in data.chunks(3) {
        match chunk.len() {
            3 => {
                let n = ((chunk[0] as u32) << 16) | ((chunk[1] as u32) << 8) | chunk[2] as u32;
                result.push(CHARSET[((n >> 18) & 63) as usize] as char);
                result.push(CHARSET[((n >> 12) & 63) as usize] as char);
                result.push(CHARSET[((n >> 6) & 63) as usize] as char);
                result.push(CHARSET[(n & 63) as usize] as char);
            }
            2 => {
                let n = ((chunk[0] as u32) << 8) | chunk[1] as u32;
                result.push(CHARSET[((n >> 10) & 63) as usize] as char);
                result.push(CHARSET[((n >> 4) & 63) as usize] as char);
                result.push(CHARSET[((n << 2) & 63) as usize] as char);
                result.push('=');
            }
            1 => {
                let n = chunk[0] as u32;
                result.push(CHARSET[((n >> 2) & 63) as usize] as char);
                result.push(CHARSET[((n << 4) & 63) as usize] as char);
                result.push('=');
                result.push('=');
            }
            _ => {}
        }
    }
    result
}
