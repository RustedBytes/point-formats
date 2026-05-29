use crate::error::{Error, Result};
use crate::format::Format;
use crate::io::{
    fmt_f64, read_exact, read_f32_le, read_u16_le, read_u32_le, write_f32_le, write_u16_le,
    write_u32_le, StlOptions,
};
use crate::types::{Face, Geometry, Mesh, Vec3, Vertex};
use std::io::{Read, Write};

pub fn read<R: Read>(reader: &mut R) -> Result<Geometry> {
    let mut bytes = Vec::new();
    reader.read_to_end(&mut bytes)?;
    if looks_like_binary_stl(&bytes) {
        read_binary(&bytes)
    } else {
        read_ascii(&String::from_utf8_lossy(&bytes))
    }
}

pub fn write<W: Write>(writer: &mut W, geometry: &Geometry, options: &StlOptions) -> Result<()> {
    let mesh = match geometry {
        Geometry::Mesh(mesh) => mesh,
        Geometry::PointCloud(_) => {
            return Err(Error::LossyConversionBlocked {
                from: "point cloud",
                to: Format::Stl,
                reason: "STL stores triangles only; point clouds must be meshed before STL export"
                    .to_string(),
            })
        }
    };
    validate_mesh(mesh)?;
    if options.binary {
        write_binary(writer, mesh)
    } else {
        write_ascii(writer, mesh)
    }
}

fn looks_like_binary_stl(bytes: &[u8]) -> bool {
    if bytes.len() < 84 {
        return false;
    }
    let mut count_bytes = [0_u8; 4];
    count_bytes.copy_from_slice(&bytes[80..84]);
    let count = u32::from_le_bytes(count_bytes) as usize;
    84usize
        .checked_add(count.saturating_mul(50))
        .map(|expected| expected == bytes.len())
        .unwrap_or(false)
}

fn read_binary(bytes: &[u8]) -> Result<Geometry> {
    let mut cursor = std::io::Cursor::new(bytes);
    let mut header = [0_u8; 80];
    cursor.read_exact(&mut header)?;
    let count = read_u32_le(&mut cursor)? as usize;
    let mut vertices = Vec::with_capacity(count * 3);
    let mut faces = Vec::with_capacity(count);

    for tri in 0..count {
        let normal = Vec3::new(
            read_f32_le(&mut cursor)? as f64,
            read_f32_le(&mut cursor)? as f64,
            read_f32_le(&mut cursor)? as f64,
        );
        let base = vertices.len();
        for _ in 0..3 {
            vertices.push(Vertex {
                position: Vec3::new(
                    read_f32_le(&mut cursor)? as f64,
                    read_f32_le(&mut cursor)? as f64,
                    read_f32_le(&mut cursor)? as f64,
                ),
                normal: Some(normal),
                color: None,
            });
        }
        let _attribute_byte_count = read_u16_le(&mut cursor)?;
        faces.push(Face::new(base, base + 1, base + 2));
        if tri == count - 1 {
            break;
        }
    }

    let mut mesh = Mesh::new(vertices, faces);
    mesh.metadata.source_format = Some(Format::Stl);
    mesh.metadata.point_count_hint = Some(mesh.vertices.len());
    mesh.metadata.comments.push(
        String::from_utf8_lossy(&header)
            .trim_matches(char::from(0))
            .trim()
            .to_string(),
    );
    Ok(Geometry::Mesh(mesh))
}

fn read_ascii(text: &str) -> Result<Geometry> {
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    let mut current_normal = Vec3::ZERO;
    let mut current_vertices = Vec::new();

    for (line_idx, line) in text.lines().enumerate() {
        let line_no = line_idx + 1;
        let parts: Vec<&str> = line.split_whitespace().collect();
        match parts.as_slice() {
            ["facet", "normal", nx, ny, nz] => {
                current_normal = Vec3::new(
                    parse_ascii_f64(line_no, nx)?,
                    parse_ascii_f64(line_no, ny)?,
                    parse_ascii_f64(line_no, nz)?,
                );
            }
            ["vertex", x, y, z] => {
                current_vertices.push(Vertex {
                    position: Vec3::new(
                        parse_ascii_f64(line_no, x)?,
                        parse_ascii_f64(line_no, y)?,
                        parse_ascii_f64(line_no, z)?,
                    ),
                    normal: Some(current_normal),
                    color: None,
                });
            }
            ["endfacet"] => {
                if current_vertices.len() < 3 {
                    return Err(Error::parse(
                        Format::Stl,
                        line_no,
                        "facet has fewer than three vertices",
                    ));
                }
                let base = vertices.len();
                vertices.extend(current_vertices.drain(..));
                faces.push(Face::new(base, base + 1, base + 2));
            }
            _ => {}
        }
    }

    let mut mesh = Mesh::new(vertices, faces);
    mesh.metadata.source_format = Some(Format::Stl);
    mesh.metadata.point_count_hint = Some(mesh.vertices.len());
    Ok(Geometry::Mesh(mesh))
}

fn write_binary<W: Write>(writer: &mut W, mesh: &Mesh) -> Result<()> {
    let mut header = [0_u8; 80];
    let label = b"binary STL generated by lidar-format-convert";
    header[..label.len()].copy_from_slice(label);
    writer.write_all(&header)?;
    let face_count = u32::try_from(mesh.faces.len())
        .map_err(|_| Error::invalid("STL binary cannot store more than u32::MAX triangles"))?;
    write_u32_le(writer, face_count)?;
    for face in &mesh.faces {
        let normal = face_normal(mesh, face);
        write_f32_le(writer, normal.x as f32)?;
        write_f32_le(writer, normal.y as f32)?;
        write_f32_le(writer, normal.z as f32)?;
        for &idx in &face.indices {
            let position = mesh.vertices[idx].position;
            write_f32_le(writer, position.x as f32)?;
            write_f32_le(writer, position.y as f32)?;
            write_f32_le(writer, position.z as f32)?;
        }
        write_u16_le(writer, 0)?;
    }
    Ok(())
}

fn write_ascii<W: Write>(writer: &mut W, mesh: &Mesh) -> Result<()> {
    writeln!(writer, "solid lidar_format_convert")?;
    for face in &mesh.faces {
        let normal = face_normal(mesh, face);
        writeln!(
            writer,
            "  facet normal {} {} {}",
            fmt_f64(normal.x, 9),
            fmt_f64(normal.y, 9),
            fmt_f64(normal.z, 9)
        )?;
        writeln!(writer, "    outer loop")?;
        for &idx in &face.indices {
            let position = mesh.vertices[idx].position;
            writeln!(
                writer,
                "      vertex {} {} {}",
                fmt_f64(position.x, 9),
                fmt_f64(position.y, 9),
                fmt_f64(position.z, 9)
            )?;
        }
        writeln!(writer, "    endloop")?;
        writeln!(writer, "  endfacet")?;
    }
    writeln!(writer, "endsolid lidar_format_convert")?;
    Ok(())
}

fn face_normal(mesh: &Mesh, face: &Face) -> Vec3 {
    let a = mesh.vertices[face.indices[0]].position;
    let b = mesh.vertices[face.indices[1]].position;
    let c = mesh.vertices[face.indices[2]].position;
    b.sub(a).cross(c.sub(a)).normalized().unwrap_or(Vec3::ZERO)
}

fn validate_mesh(mesh: &Mesh) -> Result<()> {
    for (face_idx, face) in mesh.faces.iter().enumerate() {
        for &idx in &face.indices {
            if idx >= mesh.vertices.len() {
                return Err(Error::invalid(format!(
                    "face {face_idx} references vertex {idx}, but mesh has only {} vertices",
                    mesh.vertices.len()
                )));
            }
        }
    }
    Ok(())
}

fn parse_ascii_f64(line: usize, value: &str) -> Result<f64> {
    value
        .parse::<f64>()
        .map_err(|_| Error::parse(Format::Stl, line, format!("invalid number '{value}'")))
}
