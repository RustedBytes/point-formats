use crate::error::{Error, Result};
use crate::format::Format;
use crate::io::{
    read_exact, read_f32_le, read_f64_le, read_u16_le, read_u32_le, write_f32_le,
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

    #[inline]
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

    #[inline]
    fn as_f64(self) -> f64 {
        match self {
            Self::Int(v) => v as f64,
            Self::UInt(v) => v as f64,
            Self::Float(v) => v,
        }
    }

    #[inline]
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

struct PlyVertexLayout {
    x_idx: Option<usize>,
    y_idx: Option<usize>,
    z_idx: Option<usize>,
    intensity_idx: Option<usize>,
    red_idx: Option<usize>,
    green_idx: Option<usize>,
    blue_idx: Option<usize>,
    classification_idx: Option<usize>,
    nx_idx: Option<usize>,
    ny_idx: Option<usize>,
    nz_idx: Option<usize>,
    flat_properties: Vec<Property>,
}

impl PlyVertexLayout {
    fn from_element(element: &Element) -> Self {
        let mut layout = Self {
            x_idx: None,
            y_idx: None,
            z_idx: None,
            intensity_idx: None,
            red_idx: None,
            green_idx: None,
            blue_idx: None,
            classification_idx: None,
            nx_idx: None,
            ny_idx: None,
            nz_idx: None,
            flat_properties: element.properties.clone(),
        };
        for (idx, prop) in element.properties.iter().enumerate() {
            if let Property::Scalar { name, .. } = prop {
                match name.to_ascii_lowercase().as_str() {
                    "x" => layout.x_idx = Some(idx),
                    "y" => layout.y_idx = Some(idx),
                    "z" => layout.z_idx = Some(idx),
                    "intensity" | "scalar_intensity" => layout.intensity_idx = Some(idx),
                    "red" | "diffuse_red" | "r" => layout.red_idx = Some(idx),
                    "green" | "diffuse_green" | "g" => layout.green_idx = Some(idx),
                    "blue" | "diffuse_blue" | "b" => layout.blue_idx = Some(idx),
                    "classification" | "class" | "label" => layout.classification_idx = Some(idx),
                    "nx" | "normal_x" => layout.nx_idx = Some(idx),
                    "ny" | "normal_y" => layout.ny_idx = Some(idx),
                    "nz" | "normal_z" => layout.nz_idx = Some(idx),
                    _ => {}
                }
            }
        }
        layout
    }
}

fn read_ascii_vertices<R: BufRead>(
    reader: &mut R,
    element: &Element,
    vertices: &mut Vec<Vertex>,
    points: &mut Vec<Point>,
) -> Result<()> {
    let layout = PlyVertexLayout::from_element(element);
    let mut line = String::new();
    for row in 0..element.count {
        line.clear();
        if reader.read_line(&mut line)? == 0 {
            return Err(Error::parse(
                Format::Ply,
                None,
                format!("unexpected EOF while reading vertex {row}"),
            ));
        }
        let trimmed = line.trim();
        let mut values_buf = [""; 64];
        let mut count = 0;
        for part in trimmed.split_whitespace() {
            if count < 64 {
                values_buf[count] = part;
                count += 1;
            } else {
                break;
            }
        }
        let values = &values_buf[..count];
        let (vertex, point) = read_ascii_vertex(&layout, values)?;
        vertices.push(vertex);
        points.push(point);
    }
    Ok(())
}

fn read_ascii_vertex(layout: &PlyVertexLayout, tokens: &[&str]) -> Result<(Vertex, Point)> {
    let get_scalar = |idx: usize| -> Result<ScalarValue> {
        let prop = &layout.flat_properties[idx];
        match prop {
            Property::Scalar { kind, name } => {
                let token = tokens.get(idx).copied().ok_or_else(|| {
                    Error::parse(
                        Format::Ply,
                        None,
                        format!("missing scalar property '{name}'"),
                    )
                })?;
                ScalarValue::parse_ascii(*kind, token)
            }
            _ => Err(Error::parse(
                Format::Ply,
                None,
                "unexpected list property in vertex element",
            )),
        }
    };

    let x = get_scalar(
        layout
            .x_idx
            .ok_or_else(|| Error::parse(Format::Ply, None, "missing x"))?,
    )?
    .as_f64();
    let y = get_scalar(
        layout
            .y_idx
            .ok_or_else(|| Error::parse(Format::Ply, None, "missing y"))?,
    )?
    .as_f64();
    let z = get_scalar(
        layout
            .z_idx
            .ok_or_else(|| Error::parse(Format::Ply, None, "missing z"))?,
    )?
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

    if let Some(intensity_idx) = layout.intensity_idx {
        point.intensity = Some(get_scalar(intensity_idx)?.as_f64() as f32);
    }
    if let (Some(r_idx), Some(g_idx), Some(b_idx)) =
        (layout.red_idx, layout.green_idx, layout.blue_idx)
    {
        let color = Color::new(
            color_component(get_scalar(r_idx)?)?,
            color_component(get_scalar(g_idx)?)?,
            color_component(get_scalar(b_idx)?)?,
        );
        point.color = Some(color);
        vertex.color = Some(color);
    }
    if let Some(class_idx) = layout.classification_idx {
        let value = get_scalar(class_idx)?.as_u64()?;
        if value > u8::MAX as u64 {
            return Err(Error::parse(
                Format::Ply,
                None,
                "classification exceeds u8 range",
            ));
        }
        point.classification = Some(value as u8);
    }
    if let (Some(nx_idx), Some(ny_idx), Some(nz_idx)) =
        (layout.nx_idx, layout.ny_idx, layout.nz_idx)
    {
        let normal = Vec3::new(
            get_scalar(nx_idx)?.as_f64(),
            get_scalar(ny_idx)?.as_f64(),
            get_scalar(nz_idx)?.as_f64(),
        );
        point.normal = Some(normal);
        vertex.normal = Some(normal);
    }

    Ok((vertex, point))
}

fn read_binary_vertex<R: Read>(
    reader: &mut R,
    layout: &PlyVertexLayout,
) -> Result<(Vertex, Point)> {
    let mut x = None;
    let mut y = None;
    let mut z = None;
    let mut intensity = None;
    let mut red = None;
    let mut green = None;
    let mut blue = None;
    let mut classification = None;
    let mut nx = None;
    let mut ny = None;
    let mut nz = None;

    for (idx, prop) in layout.flat_properties.iter().enumerate() {
        match prop {
            Property::Scalar { kind, .. } => {
                let val = kind.read_binary(reader)?;
                if Some(idx) == layout.x_idx {
                    x = Some(val.as_f64());
                } else if Some(idx) == layout.y_idx {
                    y = Some(val.as_f64());
                } else if Some(idx) == layout.z_idx {
                    z = Some(val.as_f64());
                } else if Some(idx) == layout.intensity_idx {
                    intensity = Some(val.as_f64() as f32);
                } else if Some(idx) == layout.red_idx {
                    red = Some(val);
                } else if Some(idx) == layout.green_idx {
                    green = Some(val);
                } else if Some(idx) == layout.blue_idx {
                    blue = Some(val);
                } else if Some(idx) == layout.classification_idx {
                    classification = Some(val.as_u64()?);
                } else if Some(idx) == layout.nx_idx {
                    nx = Some(val.as_f64());
                } else if Some(idx) == layout.ny_idx {
                    ny = Some(val.as_f64());
                } else if Some(idx) == layout.nz_idx {
                    nz = Some(val.as_f64());
                }
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

    let x = x.ok_or_else(|| Error::parse(Format::Ply, None, "missing x"))?;
    let y = y.ok_or_else(|| Error::parse(Format::Ply, None, "missing y"))?;
    let z = z.ok_or_else(|| Error::parse(Format::Ply, None, "missing z"))?;
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
    point.intensity = intensity;

    if let (Some(r), Some(g), Some(b)) = (red, green, blue) {
        let color = Color::new(
            color_component(r)?,
            color_component(g)?,
            color_component(b)?,
        );
        point.color = Some(color);
        vertex.color = Some(color);
    }
    if let Some(class) = classification {
        if class > u8::MAX as u64 {
            return Err(Error::parse(
                Format::Ply,
                None,
                "classification exceeds u8 range",
            ));
        }
        point.classification = Some(class as u8);
    }
    if let (Some(nx_val), Some(ny_val), Some(nz_val)) = (nx, ny, nz) {
        let normal = Vec3::new(nx_val, ny_val, nz_val);
        point.normal = Some(normal);
        vertex.normal = Some(normal);
    }

    Ok((vertex, point))
}

fn read_ascii_faces<R: BufRead>(
    reader: &mut R,
    element: &Element,
    faces: &mut Vec<Face>,
) -> Result<()> {
    let mut line = String::new();
    let mut values = Vec::new();
    for row in 0..element.count {
        line.clear();
        if reader.read_line(&mut line)? == 0 {
            return Err(Error::parse(
                Format::Ply,
                None,
                format!("unexpected EOF while reading face {row}"),
            ));
        }
        let trimmed = line.trim();
        let mut tokens_buf = [""; 64];
        let mut count = 0;
        for part in trimmed.split_whitespace() {
            if count < 64 {
                tokens_buf[count] = part;
                count += 1;
            } else {
                break;
            }
        }
        let tokens = &tokens_buf[..count];
        read_ascii_face_element(element, tokens, &mut values, faces)?;
    }
    Ok(())
}

fn read_ascii_face_element(
    element: &Element,
    tokens: &[&str],
    values: &mut Vec<usize>,
    faces: &mut Vec<Face>,
) -> Result<()> {
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
                values.clear();
                values.reserve(count);
                for _ in 0..count {
                    let token = tokens.get(token_index).copied().ok_or_else(|| {
                        Error::parse(Format::Ply, None, format!("missing list item for '{name}'"))
                    })?;
                    token_index += 1;
                    values.push(ScalarValue::parse_ascii(*item_kind, token)?.as_u64()? as usize);
                }
                if name == "vertex_indices" || name == "vertex_index" {
                    match values.len() {
                        0..=2 => {}
                        3 => faces.push(Face::new(values[0], values[1], values[2])),
                        n => {
                            for i in 1..(n - 1) {
                                faces.push(Face::new(values[0], values[i], values[i + 1]));
                            }
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

fn read_binary_vertices<R: Read>(
    reader: &mut R,
    element: &Element,
    vertices: &mut Vec<Vertex>,
    points: &mut Vec<Point>,
) -> Result<()> {
    let layout = PlyVertexLayout::from_element(element);
    for _ in 0..element.count {
        let (vertex, point) = read_binary_vertex(reader, &layout)?;
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
    let mut values = Vec::new();
    for _ in 0..element.count {
        read_binary_face_element(reader, element, &mut values, faces)?;
    }
    Ok(())
}

fn read_binary_face_element<R: Read>(
    reader: &mut R,
    element: &Element,
    values: &mut Vec<usize>,
    faces: &mut Vec<Face>,
) -> Result<()> {
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
                values.clear();
                values.reserve(count);
                for _ in 0..count {
                    values.push(item_kind.read_binary(reader)?.as_u64()? as usize);
                }
                if name == "vertex_indices" || name == "vertex_index" {
                    match values.len() {
                        0..=2 => {}
                        3 => faces.push(Face::new(values[0], values[1], values[2])),
                        n => {
                            for i in 1..(n - 1) {
                                faces.push(Face::new(values[0], values[i], values[i + 1]));
                            }
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

fn skip_ascii_element<R: BufRead>(reader: &mut R, element: &Element) -> Result<()> {
    let mut line = String::new();
    for row in 0..element.count {
        line.clear();
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
        for property in &element.properties {
            match property {
                Property::Scalar { kind, .. } => {
                    let _ = kind.read_binary(reader)?;
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
    }
    Ok(())
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

fn write_ascii_cloud<W: Write>(writer: &mut W, cloud: &PointCloud, precision: usize) -> Result<()> {
    let has_normals = cloud.has_normals();
    let has_color = cloud.has_color();
    let has_intensity = cloud.has_intensity();
    let has_classification = cloud.has_classification();

    write_common_header(
        writer,
        cloud.points.len(),
        0,
        has_normals,
        has_color,
        has_intensity,
        has_classification,
    )?;

    for point in &cloud.points {
        crate::io::write_fmt_f64(writer, point.position.x, precision)?;
        write!(writer, " ")?;
        crate::io::write_fmt_f64(writer, point.position.y, precision)?;
        write!(writer, " ")?;
        crate::io::write_fmt_f64(writer, point.position.z, precision)?;
        if has_normals {
            let normal = point.normal.unwrap_or(Vec3::ZERO);
            write!(writer, " ")?;
            crate::io::write_fmt_f64(writer, normal.x, precision)?;
            write!(writer, " ")?;
            crate::io::write_fmt_f64(writer, normal.y, precision)?;
            write!(writer, " ")?;
            crate::io::write_fmt_f64(writer, normal.z, precision)?;
        }
        if has_intensity {
            let intensity = point.intensity.unwrap_or(0.0);
            write!(writer, " {:.*}", precision, intensity)?;
        }
        if has_color {
            let color = point.color.unwrap_or(Color::new(0, 0, 0));
            write!(writer, " {} {} {}", color.red, color.green, color.blue)?;
        }
        if has_classification {
            let class = point.classification.unwrap_or(0);
            write!(writer, " {}", class)?;
        }
        writeln!(writer)?;
    }
    Ok(())
}

fn write_ascii_mesh<W: Write>(writer: &mut W, mesh: &Mesh, precision: usize) -> Result<()> {
    let has_normals = mesh.vertices.iter().any(|v| v.normal.is_some());
    let has_color = mesh.vertices.iter().any(|v| v.color.is_some());

    write_common_header(
        writer,
        mesh.vertices.len(),
        mesh.faces.len(),
        has_normals,
        has_color,
        false,
        false,
    )?;

    for vertex in &mesh.vertices {
        crate::io::write_fmt_f64(writer, vertex.position.x, precision)?;
        write!(writer, " ")?;
        crate::io::write_fmt_f64(writer, vertex.position.y, precision)?;
        write!(writer, " ")?;
        crate::io::write_fmt_f64(writer, vertex.position.z, precision)?;
        if has_normals {
            let normal = vertex.normal.unwrap_or(Vec3::ZERO);
            write!(writer, " ")?;
            crate::io::write_fmt_f64(writer, normal.x, precision)?;
            write!(writer, " ")?;
            crate::io::write_fmt_f64(writer, normal.y, precision)?;
            write!(writer, " ")?;
            crate::io::write_fmt_f64(writer, normal.z, precision)?;
        }
        if has_color {
            let color = vertex.color.unwrap_or(Color::new(0, 0, 0));
            write!(writer, " {} {} {}", color.red, color.green, color.blue)?;
        }
        writeln!(writer)?;
    }

    for face in &mesh.faces {
        writeln!(
            writer,
            "3 {} {} {}",
            face.indices[0], face.indices[1], face.indices[2]
        )?;
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
    writeln!(writer, "ply")?;
    writeln!(writer, "format ascii 1.0")?;
    writeln!(writer, "comment created by point-formats")?;
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

pub fn write<W: Write>(writer: &mut W, geometry: &Geometry, options: &PlyOptions) -> Result<()> {
    match options.encoding {
        PlyEncoding::Ascii => match geometry {
            Geometry::PointCloud(cloud) => write_ascii_cloud(writer, cloud, options.precision),
            Geometry::Mesh(mesh) => write_ascii_mesh(writer, mesh, options.precision),
        },
        PlyEncoding::BinaryLittleEndian => match geometry {
            Geometry::PointCloud(cloud) => {
                writeln!(writer, "ply")?;
                writeln!(writer, "format binary_little_endian 1.0")?;
                writeln!(writer, "comment created by point-formats")?;
                writeln!(writer, "element vertex {}", cloud.points.len())?;
                writeln!(writer, "property double x")?;
                writeln!(writer, "property double y")?;
                writeln!(writer, "property double z")?;
                let has_normals = cloud.has_normals();
                let has_color = cloud.has_color();
                let has_intensity = cloud.has_intensity();
                let has_classification = cloud.has_classification();
                if has_normals {
                    writeln!(writer, "property double nx")?;
                    writeln!(writer, "property double ny")?;
                    writeln!(writer, "property double nz")?;
                }
                if has_intensity {
                    writeln!(writer, "property float intensity")?;
                }
                if has_color {
                    writeln!(writer, "property ushort red")?;
                    writeln!(writer, "property ushort green")?;
                    writeln!(writer, "property ushort blue")?;
                }
                if has_classification {
                    writeln!(writer, "property uchar classification")?;
                }
                writeln!(writer, "end_header")?;

                for point in &cloud.points {
                    write_f64_le(writer, point.position.x)?;
                    write_f64_le(writer, point.position.y)?;
                    write_f64_le(writer, point.position.z)?;
                    if has_normals {
                        let normal = point.normal.unwrap_or(Vec3::ZERO);
                        write_f64_le(writer, normal.x)?;
                        write_f64_le(writer, normal.y)?;
                        write_f64_le(writer, normal.z)?;
                    }
                    if has_intensity {
                        let intensity = point.intensity.unwrap_or(0.0);
                        write_f32_le(writer, intensity)?;
                    }
                    if has_color {
                        let color = point.color.unwrap_or(Color::new(0, 0, 0));
                        write_u16_le(writer, color.red)?;
                        write_u16_le(writer, color.green)?;
                        write_u16_le(writer, color.blue)?;
                    }
                    if has_classification {
                        let class = point.classification.unwrap_or(0);
                        writer.write_all(&[class])?;
                    }
                }
                Ok(())
            }
            Geometry::Mesh(mesh) => {
                writeln!(writer, "ply")?;
                writeln!(writer, "format binary_little_endian 1.0")?;
                writeln!(writer, "comment created by point-formats")?;
                writeln!(writer, "element vertex {}", mesh.vertices.len())?;
                writeln!(writer, "property double x")?;
                writeln!(writer, "property double y")?;
                writeln!(writer, "property double z")?;
                let has_normals = mesh.vertices.iter().any(|v| v.normal.is_some());
                let has_color = mesh.vertices.iter().any(|v| v.color.is_some());
                if has_normals {
                    writeln!(writer, "property double nx")?;
                    writeln!(writer, "property double ny")?;
                    writeln!(writer, "property double nz")?;
                }
                if has_color {
                    writeln!(writer, "property ushort red")?;
                    writeln!(writer, "property ushort green")?;
                    writeln!(writer, "property ushort blue")?;
                }
                if !mesh.faces.is_empty() {
                    writeln!(writer, "element face {}", mesh.faces.len())?;
                    writeln!(writer, "property list uchar uint vertex_indices")?;
                }
                writeln!(writer, "end_header")?;

                for vertex in &mesh.vertices {
                    write_f64_le(writer, vertex.position.x)?;
                    write_f64_le(writer, vertex.position.y)?;
                    write_f64_le(writer, vertex.position.z)?;
                    if has_normals {
                        let normal = vertex.normal.unwrap_or(Vec3::ZERO);
                        write_f64_le(writer, normal.x)?;
                        write_f64_le(writer, normal.y)?;
                        write_f64_le(writer, normal.z)?;
                    }
                    if has_color {
                        let color = vertex.color.unwrap_or(Color::new(0, 0, 0));
                        write_u16_le(writer, color.red)?;
                        write_u16_le(writer, color.green)?;
                        write_u16_le(writer, color.blue)?;
                    }
                }

                for face in &mesh.faces {
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
        },
    }
}
