use crate::error::{Error, Result};
use crate::format::Format;
use crate::io::{
    fmt_f64, read_exact, read_f32_le, read_f64_le, read_u16_le, read_u32_le, write_f32_le,
    write_f64_le, write_u16_le, write_u32_le, PlyEncoding, PlyOptions,
};
use crate::types::{Color, Face, Geometry, Mesh, Point, PointCloud, Vec3, Vertex};
use std::io::{BufRead, Read, Write};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PlyDataFormat {
    Ascii,
    BinaryLittleEndian,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ScalarType {
    Char,
    UChar,
    Short,
    UShort,
    Int,
    UInt,
    Float,
    Double,
}

impl ScalarType {
    fn parse(value: &str) -> Option<Self> {
        match value {
            "char" | "int8" => Some(Self::Char),
            "uchar" | "uint8" => Some(Self::UChar),
            "short" | "int16" => Some(Self::Short),
            "ushort" | "uint16" => Some(Self::UShort),
            "int" | "int32" => Some(Self::Int),
            "uint" | "uint32" => Some(Self::UInt),
            "float" | "float32" => Some(Self::Float),
            "double" | "float64" => Some(Self::Double),
            _ => None,
        }
    }

    fn read_binary<R: Read>(self, reader: &mut R) -> Result<ScalarValue> {
        Ok(match self {
            Self::Char => ScalarValue::Int(i8::from_le_bytes(read_exact(reader)?) as i64),
            Self::UChar => ScalarValue::UInt(u8::from_le_bytes(read_exact(reader)?) as u64),
            Self::Short => ScalarValue::Int(i16::from_le_bytes(read_exact(reader)?) as i64),
            Self::UShort => ScalarValue::UInt(read_u16_le(reader)? as u64),
            Self::Int => ScalarValue::Int(i32::from_le_bytes(read_exact(reader)?) as i64),
            Self::UInt => ScalarValue::UInt(read_u32_le(reader)? as u64),
            Self::Float => ScalarValue::Float(read_f32_le(reader)? as f64),
            Self::Double => ScalarValue::Float(read_f64_le(reader)?),
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ScalarValue {
    Int(i64),
    UInt(u64),
    Float(f64),
}

impl ScalarValue {
    fn parse_ascii(kind: ScalarType, value: &str) -> Result<Self> {
        let parsed = match kind {
            ScalarType::Char | ScalarType::Short | ScalarType::Int => {
                Self::Int(value.parse::<i64>().map_err(|_| {
                    Error::parse(
                        Format::Ply,
                        None,
                        format!("invalid integer scalar '{value}'"),
                    )
                })?)
            }
            ScalarType::UChar | ScalarType::UShort | ScalarType::UInt => {
                Self::UInt(value.parse::<u64>().map_err(|_| {
                    Error::parse(
                        Format::Ply,
                        None,
                        format!("invalid unsigned scalar '{value}'"),
                    )
                })?)
            }
            ScalarType::Float | ScalarType::Double => {
                Self::Float(value.parse::<f64>().map_err(|_| {
                    Error::parse(
                        Format::Ply,
                        None,
                        format!("invalid floating scalar '{value}'"),
                    )
                })?)
            }
        };
        Ok(parsed)
    }

    fn as_f64(self) -> f64 {
        match self {
            Self::Int(v) => v as f64,
            Self::UInt(v) => v as f64,
            Self::Float(v) => v,
        }
    }

    fn as_u64(self) -> Result<u64> {
        match self {
            Self::UInt(v) => Ok(v),
            Self::Int(v) if v >= 0 => Ok(v as u64),
            Self::Float(v) if v.is_finite() && v.fract() == 0.0 && v >= 0.0 => Ok(v as u64),
            _ => Err(Error::parse(
                Format::Ply,
                None,
                "expected non-negative integer scalar",
            )),
        }
    }
}

#[derive(Debug, Clone)]
enum Property {
    Scalar {
        name: String,
        kind: ScalarType,
    },
    List {
        name: String,
        count_kind: ScalarType,
        item_kind: ScalarType,
    },
}

#[derive(Debug, Clone)]
struct Element {
    name: String,
    count: usize,
    properties: Vec<Property>,
}

#[derive(Debug, Clone)]
struct Header {
    data_format: PlyDataFormat,
    elements: Vec<Element>,
    comments: Vec<String>,
}

pub fn read<R: BufRead>(reader: &mut R) -> Result<Geometry> {
    let header = read_header(reader)?;
    let vertex_capacity = header
        .elements
        .iter()
        .find(|element| element.name == "vertex")
        .map(|element| element.count)
        .ok_or_else(|| Error::parse(Format::Ply, None, "missing vertex element"))?;

    let mut vertices: Vec<Vertex> = Vec::with_capacity(vertex_capacity);
    let mut points: Vec<Point> = Vec::with_capacity(vertex_capacity);
    let mut faces: Vec<Face> = Vec::new();

    match header.data_format {
        PlyDataFormat::Ascii => {
            for element in &header.elements {
                match element.name.as_str() {
                    "vertex" => read_ascii_vertices(reader, element, &mut vertices, &mut points)?,
                    "face" => read_ascii_faces(reader, element, &mut faces)?,
                    _ => skip_ascii_element(reader, element)?,
                }
            }
        }
        PlyDataFormat::BinaryLittleEndian => {
            for element in &header.elements {
                match element.name.as_str() {
                    "vertex" => read_binary_vertices(reader, element, &mut vertices, &mut points)?,
                    "face" => read_binary_faces(reader, element, &mut faces)?,
                    _ => skip_binary_element(reader, element)?,
                }
            }
        }
    }

    if faces.is_empty() {
        let mut cloud = PointCloud::new(points);
        cloud.metadata.source_format = Some(Format::Ply);
        cloud.metadata.point_count_hint = Some(cloud.points.len());
        cloud.metadata.comments = header.comments;
        Ok(Geometry::PointCloud(cloud))
    } else {
        let mut mesh = Mesh::new(vertices, faces);
        mesh.metadata.source_format = Some(Format::Ply);
        mesh.metadata.point_count_hint = Some(mesh.vertices.len());
        mesh.metadata.comments = header.comments;
        Ok(Geometry::Mesh(mesh))
    }
}

pub fn write<W: Write>(writer: &mut W, geometry: &Geometry, options: &PlyOptions) -> Result<()> {
    match options.encoding {
        PlyEncoding::Ascii => write_ascii(writer, geometry, options.precision),
        PlyEncoding::BinaryLittleEndian => write_binary(writer, geometry),
    }
}

fn read_header<R: BufRead>(reader: &mut R) -> Result<Header> {
    let mut first = String::new();
    reader.read_line(&mut first)?;
    if first.trim() != "ply" {
        return Err(Error::parse(
            Format::Ply,
            Some(1),
            "expected PLY magic 'ply'",
        ));
    }

    let mut data_format = None;
    let mut elements: Vec<Element> = Vec::new();
    let mut current: Option<Element> = None;
    let mut comments = Vec::new();

    for line_no in 2usize.. {
        let mut line = String::new();
        if reader.read_line(&mut line)? == 0 {
            return Err(Error::parse(
                Format::Ply,
                line_no,
                "unexpected EOF in header",
            ));
        }
        let trimmed = line.trim();
        if trimmed == "end_header" {
            if let Some(element) = current.take() {
                elements.push(element);
            }
            break;
        }
        if trimmed.is_empty() {
            continue;
        }
        let parts: Vec<&str> = trimmed.split_whitespace().collect();
        match parts.as_slice() {
            ["format", "ascii", "1.0"] => data_format = Some(PlyDataFormat::Ascii),
            ["format", "binary_little_endian", "1.0"] => {
                data_format = Some(PlyDataFormat::BinaryLittleEndian)
            }
            ["format", "binary_big_endian", "1.0"] => {
                return Err(Error::unsupported(
                    Format::Ply,
                    "read",
                    "binary big-endian PLY is not implemented; convert to ASCII or binary_little_endian first",
                ));
            }
            ["comment", rest @ ..] => comments.push(rest.join(" ")),
            ["element", name, count] => {
                if let Some(element) = current.take() {
                    elements.push(element);
                }
                let count = count.parse::<usize>().map_err(|_| {
                    Error::parse(
                        Format::Ply,
                        line_no,
                        format!("invalid element count '{count}'"),
                    )
                })?;
                current = Some(Element {
                    name: (*name).to_string(),
                    count,
                    properties: Vec::new(),
                });
            }
            ["property", "list", count_type, item_type, name] => {
                let element = current.as_mut().ok_or_else(|| {
                    Error::parse(Format::Ply, line_no, "property appeared before element")
                })?;
                element.properties.push(Property::List {
                    name: (*name).to_string(),
                    count_kind: ScalarType::parse(count_type).ok_or_else(|| {
                        Error::parse(
                            Format::Ply,
                            line_no,
                            format!("unknown scalar type '{count_type}'"),
                        )
                    })?,
                    item_kind: ScalarType::parse(item_type).ok_or_else(|| {
                        Error::parse(
                            Format::Ply,
                            line_no,
                            format!("unknown scalar type '{item_type}'"),
                        )
                    })?,
                });
            }
            ["property", scalar_type, name] => {
                let element = current.as_mut().ok_or_else(|| {
                    Error::parse(Format::Ply, line_no, "property appeared before element")
                })?;
                element.properties.push(Property::Scalar {
                    name: (*name).to_string(),
                    kind: ScalarType::parse(scalar_type).ok_or_else(|| {
                        Error::parse(
                            Format::Ply,
                            line_no,
                            format!("unknown scalar type '{scalar_type}'"),
                        )
                    })?,
                });
            }
            _ => {
                // Preserve forward compatibility with obj_info and unknown header metadata.
                if trimmed.starts_with("obj_info") {
                    comments.push(trimmed.to_string());
                } else {
                    return Err(Error::parse(
                        Format::Ply,
                        line_no,
                        format!("unsupported header line '{trimmed}'"),
                    ));
                }
            }
        }
    }

    Ok(Header {
        data_format: data_format
            .ok_or_else(|| Error::parse(Format::Ply, None, "missing format line"))?,
        elements,
        comments,
    })
}

fn read_ascii_vertices<R: BufRead>(
    reader: &mut R,
    element: &Element,
    vertices: &mut Vec<Vertex>,
    points: &mut Vec<Point>,
) -> Result<()> {
    for row in 0..element.count {
        let mut line = String::new();
        if reader.read_line(&mut line)? == 0 {
            return Err(Error::parse(
                Format::Ply,
                None,
                format!("unexpected EOF while reading vertex {row}"),
            ));
        }
        let values: Vec<&str> = line.split_whitespace().collect();
        let parsed = read_ascii_element_values(element, &values)?;
        let (vertex, point) = vertex_point_from_properties(&parsed)?;
        vertices.push(vertex);
        points.push(point);
    }
    Ok(())
}

fn read_ascii_faces<R: BufRead>(
    reader: &mut R,
    element: &Element,
    faces: &mut Vec<Face>,
) -> Result<()> {
    for row in 0..element.count {
        let mut line = String::new();
        if reader.read_line(&mut line)? == 0 {
            return Err(Error::parse(
                Format::Ply,
                None,
                format!("unexpected EOF while reading face {row}"),
            ));
        }
        let tokens: Vec<&str> = line.split_whitespace().collect();
        let lists = read_ascii_element_lists(element, &tokens)?;
        append_faces_from_lists(&lists, faces)?;
    }
    Ok(())
}

fn read_binary_vertices<R: Read>(
    reader: &mut R,
    element: &Element,
    vertices: &mut Vec<Vertex>,
    points: &mut Vec<Point>,
) -> Result<()> {
    for _ in 0..element.count {
        let parsed = read_binary_element_values(reader, element)?;
        let (vertex, point) = vertex_point_from_properties(&parsed)?;
        vertices.push(vertex);
        points.push(point);
    }
    Ok(())
}

fn read_binary_faces<R: Read>(
    reader: &mut R,
    element: &Element,
    faces: &mut Vec<Face>,
) -> Result<()> {
    for _ in 0..element.count {
        let lists = read_binary_element_lists(reader, element)?;
        append_faces_from_lists(&lists, faces)?;
    }
    Ok(())
}

fn skip_ascii_element<R: BufRead>(reader: &mut R, element: &Element) -> Result<()> {
    for row in 0..element.count {
        let mut line = String::new();
        if reader.read_line(&mut line)? == 0 {
            return Err(Error::parse(
                Format::Ply,
                None,
                format!(
                    "unexpected EOF while skipping element '{}' row {row}",
                    element.name
                ),
            ));
        }
    }
    Ok(())
}

fn skip_binary_element<R: Read>(reader: &mut R, element: &Element) -> Result<()> {
    for _ in 0..element.count {
        let _ = read_binary_element_values(reader, element)?;
    }
    Ok(())
}

fn read_ascii_element_values(
    element: &Element,
    tokens: &[&str],
) -> Result<Vec<(String, ScalarValue)>> {
    let mut values = Vec::new();
    let mut token_index = 0usize;
    for property in &element.properties {
        match property {
            Property::Scalar { name, kind } => {
                let token = tokens.get(token_index).copied().ok_or_else(|| {
                    Error::parse(
                        Format::Ply,
                        None,
                        format!("missing scalar property '{name}'"),
                    )
                })?;
                token_index += 1;
                values.push((name.clone(), ScalarValue::parse_ascii(*kind, token)?));
            }
            Property::List {
                name,
                count_kind,
                item_kind,
            } => {
                let count_token = tokens.get(token_index).copied().ok_or_else(|| {
                    Error::parse(
                        Format::Ply,
                        None,
                        format!("missing list count for '{name}'"),
                    )
                })?;
                token_index += 1;
                let count = ScalarValue::parse_ascii(*count_kind, count_token)?.as_u64()? as usize;
                for _ in 0..count {
                    let token = tokens.get(token_index).copied().ok_or_else(|| {
                        Error::parse(Format::Ply, None, format!("missing list item for '{name}'"))
                    })?;
                    token_index += 1;
                    let _ = ScalarValue::parse_ascii(*item_kind, token)?;
                }
            }
        }
    }
    Ok(values)
}

fn read_ascii_element_lists(
    element: &Element,
    tokens: &[&str],
) -> Result<Vec<(String, Vec<usize>)>> {
    let mut lists = Vec::new();
    let mut token_index = 0usize;
    for property in &element.properties {
        match property {
            Property::Scalar { kind, .. } => {
                let token = tokens.get(token_index).copied().ok_or_else(|| {
                    Error::parse(
                        Format::Ply,
                        None,
                        "missing scalar while reading face element",
                    )
                })?;
                token_index += 1;
                let _ = ScalarValue::parse_ascii(*kind, token)?;
            }
            Property::List {
                name,
                count_kind,
                item_kind,
            } => {
                let count_token = tokens.get(token_index).copied().ok_or_else(|| {
                    Error::parse(
                        Format::Ply,
                        None,
                        format!("missing list count for '{name}'"),
                    )
                })?;
                token_index += 1;
                let count = ScalarValue::parse_ascii(*count_kind, count_token)?.as_u64()? as usize;
                let mut values = Vec::with_capacity(count);
                for _ in 0..count {
                    let token = tokens.get(token_index).copied().ok_or_else(|| {
                        Error::parse(Format::Ply, None, format!("missing list item for '{name}'"))
                    })?;
                    token_index += 1;
                    values.push(ScalarValue::parse_ascii(*item_kind, token)?.as_u64()? as usize);
                }
                lists.push((name.clone(), values));
            }
        }
    }
    Ok(lists)
}

fn read_binary_element_values<R: Read>(
    reader: &mut R,
    element: &Element,
) -> Result<Vec<(String, ScalarValue)>> {
    let mut values = Vec::new();
    for property in &element.properties {
        match property {
            Property::Scalar { name, kind } => {
                values.push((name.clone(), kind.read_binary(reader)?))
            }
            Property::List {
                count_kind,
                item_kind,
                ..
            } => {
                let count = count_kind.read_binary(reader)?.as_u64()?;
                for _ in 0..count {
                    let _ = item_kind.read_binary(reader)?;
                }
            }
        }
    }
    Ok(values)
}

fn read_binary_element_lists<R: Read>(
    reader: &mut R,
    element: &Element,
) -> Result<Vec<(String, Vec<usize>)>> {
    let mut lists = Vec::new();
    for property in &element.properties {
        match property {
            Property::Scalar { kind, .. } => {
                let _ = kind.read_binary(reader)?;
            }
            Property::List {
                name,
                count_kind,
                item_kind,
            } => {
                let count = count_kind.read_binary(reader)?.as_u64()? as usize;
                let mut values = Vec::with_capacity(count);
                for _ in 0..count {
                    values.push(item_kind.read_binary(reader)?.as_u64()? as usize);
                }
                lists.push((name.clone(), values));
            }
        }
    }
    Ok(lists)
}

fn vertex_point_from_properties(values: &[(String, ScalarValue)]) -> Result<(Vertex, Point)> {
    fn find(values: &[(String, ScalarValue)], names: &[&str]) -> Option<ScalarValue> {
        values
            .iter()
            .find(|(name, _)| {
                names
                    .iter()
                    .any(|candidate| name.eq_ignore_ascii_case(candidate))
            })
            .map(|(_, value)| *value)
    }

    let x = find(values, &["x"])
        .ok_or_else(|| Error::parse(Format::Ply, None, "missing x"))?
        .as_f64();
    let y = find(values, &["y"])
        .ok_or_else(|| Error::parse(Format::Ply, None, "missing y"))?
        .as_f64();
    let z = find(values, &["z"])
        .ok_or_else(|| Error::parse(Format::Ply, None, "missing z"))?
        .as_f64();
    let position = Vec3::new(x, y, z);
    if !position.is_finite() {
        return Err(Error::parse(
            Format::Ply,
            None,
            "vertex coordinates must be finite",
        ));
    }

    let mut point = Point::new(x, y, z);
    let mut vertex = Vertex::new(position);

    if let Some(intensity) = find(values, &["intensity", "scalar_intensity"]) {
        point.intensity = Some(intensity.as_f64() as f32);
    }
    let red = find(values, &["red", "diffuse_red", "r"]);
    let green = find(values, &["green", "diffuse_green", "g"]);
    let blue = find(values, &["blue", "diffuse_blue", "b"]);
    if let (Some(red), Some(green), Some(blue)) = (red, green, blue) {
        let color = Color::new(
            color_component(red)?,
            color_component(green)?,
            color_component(blue)?,
        );
        point.color = Some(color);
        vertex.color = Some(color);
    }

    if let Some(classification) = find(values, &["classification", "class", "label"]) {
        let value = classification.as_u64()?;
        if value > u8::MAX as u64 {
            return Err(Error::parse(
                Format::Ply,
                None,
                "classification exceeds u8 range",
            ));
        }
        point.classification = Some(value as u8);
    }

    let nx = find(values, &["nx", "normal_x"]);
    let ny = find(values, &["ny", "normal_y"]);
    let nz = find(values, &["nz", "normal_z"]);
    if let (Some(nx), Some(ny), Some(nz)) = (nx, ny, nz) {
        let normal = Vec3::new(nx.as_f64(), ny.as_f64(), nz.as_f64());
        point.normal = Some(normal);
        vertex.normal = Some(normal);
    }

    Ok((vertex, point))
}

fn color_component(value: ScalarValue) -> Result<u16> {
    let f = value.as_f64();
    if !f.is_finite() || f < 0.0 || f > u16::MAX as f64 {
        return Err(Error::parse(
            Format::Ply,
            None,
            "color component outside u16 range",
        ));
    }
    if matches!(value, ScalarValue::Float(_)) && f <= 1.0 {
        Ok((f * u16::MAX as f64).round() as u16)
    } else {
        Ok(f.round() as u16)
    }
}

fn append_faces_from_lists(lists: &[(String, Vec<usize>)], faces: &mut Vec<Face>) -> Result<()> {
    let list = lists
        .iter()
        .find(|(name, _)| name == "vertex_indices" || name == "vertex_index")
        .map(|(_, values)| values);
    if let Some(indices) = list {
        match indices.len() {
            0..=2 => {}
            3 => faces.push(Face::new(indices[0], indices[1], indices[2])),
            n => {
                for i in 1..(n - 1) {
                    faces.push(Face::new(indices[0], indices[i], indices[i + 1]));
                }
            }
        }
    }
    Ok(())
}

fn write_ascii<W: Write>(writer: &mut W, geometry: &Geometry, precision: usize) -> Result<()> {
    let view = WriteGeometryView::new(geometry);

    writeln!(writer, "ply")?;
    writeln!(writer, "format ascii 1.0")?;
    writeln!(writer, "comment created by lidar-format-convert")?;
    write_common_header(
        writer,
        view.vertices().len(),
        view.faces().len(),
        view.has_normals,
        view.has_color,
        view.has_intensity,
        view.has_classification,
    )?;

    for (idx, vertex) in view.vertices().iter().enumerate() {
        let mut fields = vec![
            fmt_f64(vertex.position.x, precision),
            fmt_f64(vertex.position.y, precision),
            fmt_f64(vertex.position.z, precision),
        ];
        if view.has_normals {
            let normal = vertex.normal.unwrap_or(Vec3::ZERO);
            fields.extend([
                fmt_f64(normal.x, precision),
                fmt_f64(normal.y, precision),
                fmt_f64(normal.z, precision),
            ]);
        }
        if view.has_intensity {
            let intensity = view.point_at(idx).and_then(|p| p.intensity).unwrap_or(0.0);
            fields.push(format!("{:.*}", precision, intensity));
        }
        if view.has_color {
            let color = vertex.color.unwrap_or(Color::new(0, 0, 0));
            fields.extend([
                color.red.to_string(),
                color.green.to_string(),
                color.blue.to_string(),
            ]);
        }
        if view.has_classification {
            let class = view
                .point_at(idx)
                .and_then(|p| p.classification)
                .unwrap_or(0);
            fields.push(class.to_string());
        }
        writeln!(writer, "{}", fields.join(" "))?;
    }

    for face in view.faces() {
        writeln!(
            writer,
            "3 {} {} {}",
            face.indices[0], face.indices[1], face.indices[2]
        )?;
    }
    Ok(())
}

fn write_binary<W: Write>(writer: &mut W, geometry: &Geometry) -> Result<()> {
    let view = WriteGeometryView::new(geometry);

    writeln!(writer, "ply")?;
    writeln!(writer, "format binary_little_endian 1.0")?;
    writeln!(writer, "comment created by lidar-format-convert")?;
    write_common_header(
        writer,
        view.vertices().len(),
        view.faces().len(),
        view.has_normals,
        view.has_color,
        view.has_intensity,
        view.has_classification,
    )?;

    for (idx, vertex) in view.vertices().iter().enumerate() {
        write_f64_le(writer, vertex.position.x)?;
        write_f64_le(writer, vertex.position.y)?;
        write_f64_le(writer, vertex.position.z)?;
        if view.has_normals {
            let normal = vertex.normal.unwrap_or(Vec3::ZERO);
            write_f64_le(writer, normal.x)?;
            write_f64_le(writer, normal.y)?;
            write_f64_le(writer, normal.z)?;
        }
        if view.has_intensity {
            let intensity = view.point_at(idx).and_then(|p| p.intensity).unwrap_or(0.0);
            write_f32_le(writer, intensity)?;
        }
        if view.has_color {
            let color = vertex.color.unwrap_or(Color::new(0, 0, 0));
            write_u16_le(writer, color.red)?;
            write_u16_le(writer, color.green)?;
            write_u16_le(writer, color.blue)?;
        }
        if view.has_classification {
            let class = view
                .point_at(idx)
                .and_then(|p| p.classification)
                .unwrap_or(0);
            writer.write_all(&[class])?;
        }
    }

    for face in view.faces() {
        writer.write_all(&[3])?;
        let a = u32::try_from(face.indices[0])
            .map_err(|_| Error::invalid("PLY binary face index exceeds u32 range"))?;
        let b = u32::try_from(face.indices[1])
            .map_err(|_| Error::invalid("PLY binary face index exceeds u32 range"))?;
        let c = u32::try_from(face.indices[2])
            .map_err(|_| Error::invalid("PLY binary face index exceeds u32 range"))?;
        write_u32_le(writer, a)?;
        write_u32_le(writer, b)?;
        write_u32_le(writer, c)?;
    }
    Ok(())
}

fn write_common_header<W: Write>(
    writer: &mut W,
    vertex_count: usize,
    face_count: usize,
    has_normals: bool,
    has_color: bool,
    has_intensity: bool,
    has_classification: bool,
) -> Result<()> {
    writeln!(writer, "element vertex {vertex_count}")?;
    writeln!(writer, "property double x")?;
    writeln!(writer, "property double y")?;
    writeln!(writer, "property double z")?;
    if has_normals {
        writeln!(writer, "property double nx")?;
        writeln!(writer, "property double ny")?;
        writeln!(writer, "property double nz")?;
    }
    if has_intensity {
        writeln!(writer, "property float intensity")?;
    }
    if has_color {
        // Preserve LAS-style 16-bit colors. Many PLY viewers prefer uchar; users
        // can downscale with Color::to_u8_lossy before writing if needed.
        writeln!(writer, "property ushort red")?;
        writeln!(writer, "property ushort green")?;
        writeln!(writer, "property ushort blue")?;
    }
    if has_classification {
        writeln!(writer, "property uchar classification")?;
    }
    if face_count > 0 {
        writeln!(writer, "element face {face_count}")?;
        writeln!(writer, "property list uchar uint vertex_indices")?;
    }
    writeln!(writer, "end_header")?;
    Ok(())
}

struct WriteGeometryView<'a> {
    owned_vertices: Option<Vec<Vertex>>,
    borrowed_vertices: Option<&'a [Vertex]>,
    faces: &'a [Face],
    cloud: Option<&'a PointCloud>,
    has_normals: bool,
    has_color: bool,
    has_intensity: bool,
    has_classification: bool,
}

impl<'a> WriteGeometryView<'a> {
    fn new(geometry: &'a Geometry) -> Self {
        match geometry {
            Geometry::PointCloud(cloud) => Self {
                owned_vertices: Some(vertices_from_cloud(cloud)),
                borrowed_vertices: None,
                faces: &[],
                cloud: Some(cloud),
                has_normals: cloud.has_normals(),
                has_color: cloud.has_color(),
                has_intensity: cloud.has_intensity(),
                has_classification: cloud.has_classification(),
            },
            Geometry::Mesh(mesh) => Self {
                owned_vertices: None,
                borrowed_vertices: Some(&mesh.vertices),
                faces: &mesh.faces,
                cloud: None,
                has_normals: mesh.vertices.iter().any(|v| v.normal.is_some()),
                has_color: mesh.vertices.iter().any(|v| v.color.is_some()),
                has_intensity: false,
                has_classification: false,
            },
        }
    }

    fn vertices(&self) -> &[Vertex] {
        self.borrowed_vertices
            .or(self.owned_vertices.as_deref())
            .unwrap_or(&[])
    }

    fn faces(&self) -> &[Face] {
        self.faces
    }

    fn point_at(&self, idx: usize) -> Option<&Point> {
        self.cloud.and_then(|cloud| cloud.points.get(idx))
    }
}

fn vertices_from_cloud(cloud: &PointCloud) -> Vec<Vertex> {
    cloud
        .points
        .iter()
        .map(|point| Vertex {
            position: point.position,
            normal: point.normal,
            color: point.color,
        })
        .collect()
}
