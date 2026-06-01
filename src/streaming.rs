//! Synchronous point-cloud streaming APIs for large files.

use crate::convert::{apply_geometry_policy, ConversionReport, ConvertOptions};
use crate::error::{Error, Result};
use crate::format::Format;
use crate::io::{ColumnMapping, DelimitedOptions, Delimiter, PcdEncoding, PlyEncoding};
use crate::types::{Geometry, Metadata, Point, PointCloud, Vec3};
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;

/// Fields known to be present in a point stream before writing starts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PointFieldSet {
    pub intensity: bool,
    pub color: bool,
    pub classification: bool,
    pub gps_time: bool,
    pub normals: bool,
}

impl PointFieldSet {
    pub fn from_point(point: &Point) -> Self {
        Self {
            intensity: point.intensity.is_some(),
            color: point.color.is_some(),
            classification: point.classification.is_some(),
            gps_time: point.gps_time.is_some(),
            normals: point.normal.is_some(),
        }
    }

    pub fn from_cloud(cloud: &PointCloud) -> Self {
        Self {
            intensity: cloud.has_intensity(),
            color: cloud.has_color(),
            classification: cloud.has_classification(),
            gps_time: cloud.has_gps_time(),
            normals: cloud.has_normals(),
        }
    }

    pub fn include_point(&mut self, point: &Point) {
        self.intensity |= point.intensity.is_some();
        self.color |= point.color.is_some();
        self.classification |= point.classification.is_some();
        self.gps_time |= point.gps_time.is_some();
        self.normals |= point.normal.is_some();
    }
}

/// Metadata and schema available before consuming a point stream.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PointStreamInfo {
    pub source_format: Option<Format>,
    pub metadata: Metadata,
    pub point_count_hint: Option<usize>,
    pub fields: PointFieldSet,
}

impl PointStreamInfo {
    pub fn new(source_format: Format) -> Self {
        let metadata = Metadata {
            source_format: Some(source_format),
            ..Metadata::default()
        };
        Self {
            source_format: Some(source_format),
            metadata,
            point_count_hint: None,
            fields: PointFieldSet::default(),
        }
    }

    pub fn from_cloud(source_format: Format, cloud: &PointCloud) -> Self {
        let mut metadata = cloud.metadata.clone();
        metadata.source_format = Some(source_format);
        Self {
            source_format: Some(source_format),
            point_count_hint: metadata.point_count_hint.or(Some(cloud.points.len())),
            fields: PointFieldSet::from_cloud(cloud),
            metadata,
        }
    }
}

/// Object-safe iterator over normalized points.
pub trait PointStream: Send {
    fn info(&self) -> &PointStreamInfo;
    fn next_point(&mut self) -> Result<Option<Point>>;
}

/// Streaming point writer.
pub trait PointStreamWriter: Send {
    fn write_point(&mut self, point: &Point) -> Result<()>;
    fn finish(&mut self) -> Result<()>;
}

/// Adapter trait for heavyweight point formats that can stream.
pub trait StreamingCodec: Send + Sync {
    fn can_stream_read(&self, format: Format) -> bool;
    fn can_stream_write(&self, format: Format) -> bool;

    fn open_point_stream(
        &self,
        path: &Path,
        format: Format,
        options: &ConvertOptions,
    ) -> Result<Box<dyn PointStream>>;

    fn create_point_writer(
        &self,
        path: &Path,
        format: Format,
        info: &PointStreamInfo,
        options: &ConvertOptions,
    ) -> Result<Box<dyn PointStreamWriter>>;
}

/// Static registry for custom streaming codecs.
#[derive(Default)]
pub struct StreamingCodecRegistry {
    codecs: Vec<Box<dyn StreamingCodec>>,
}

impl StreamingCodecRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_codec(mut self, codec: Box<dyn StreamingCodec>) -> Self {
        self.codecs.push(codec);
        self
    }

    pub fn register(&mut self, codec: Box<dyn StreamingCodec>) {
        self.codecs.push(codec);
    }

    pub fn reader(&self, format: Format) -> Option<&dyn StreamingCodec> {
        self.codecs
            .iter()
            .find(|codec| codec.can_stream_read(format))
            .map(|codec| codec.as_ref())
    }

    pub fn writer(&self, format: Format) -> Option<&dyn StreamingCodec> {
        self.codecs
            .iter()
            .find(|codec| codec.can_stream_write(format))
            .map(|codec| codec.as_ref())
    }
}

/// Converts point-cloud files with streaming adapters when possible.
pub fn convert_path_streaming(
    input: impl AsRef<Path>,
    output: impl AsRef<Path>,
    options: &ConvertOptions,
) -> Result<ConversionReport> {
    convert_path_with_streaming_adapters(input, output, options, &StreamingCodecRegistry::new())
}

/// Converts point-cloud files using native streams and an explicit streaming adapter registry.
pub fn convert_path_with_streaming_adapters(
    input: impl AsRef<Path>,
    output: impl AsRef<Path>,
    options: &ConvertOptions,
    registry: &StreamingCodecRegistry,
) -> Result<ConversionReport> {
    let input = input.as_ref();
    let output = output.as_ref();
    let input_format = options
        .input_format
        .map(Ok)
        .unwrap_or_else(|| Format::from_path(input))?;
    let output_format = options
        .output_format
        .map(Ok)
        .unwrap_or_else(|| Format::from_path(output))?;

    if !matches!(
        (input_format.family(), output_format.family()),
        (
            crate::format::FormatFamily::PointCloud,
            crate::format::FormatFamily::PointCloud
        )
    ) {
        return Err(Error::unsupported(
            input_format,
            "stream read",
            "streaming currently supports point-cloud to point-cloud conversions only",
        ));
    }

    let mut stream = open_stream(input, input_format, options, registry)?;
    let info = stream.info().clone();
    let mut writer = create_writer(output, output_format, &info, options, registry)?;
    let mut points = 0usize;

    while let Some(point) = stream.next_point()? {
        writer.write_point(&point)?;
        points += 1;
    }
    writer.finish()?;

    let mut warnings = info.metadata.warnings.clone();
    if let Some(expected) = info.point_count_hint {
        if expected != points {
            warnings.push(format!(
                "stream declared {expected} points but produced {points} point records"
            ));
        }
    }

    Ok(ConversionReport {
        input_format,
        output_format,
        points_read: points,
        points_written: points,
        faces_read: 0,
        faces_written: 0,
        warnings,
    })
}

pub(crate) fn can_native_stream_read(format: Format) -> bool {
    matches!(
        format,
        Format::Xyz | Format::Txt | Format::Csv | Format::Pts
    )
}

pub(crate) fn can_native_stream_write(format: Format, info: &PointStreamInfo) -> bool {
    match format {
        Format::Xyz | Format::Txt | Format::Csv => true,
        Format::Pts | Format::Ply | Format::Pcd => info.point_count_hint.is_some(),
        _ => false,
    }
}

fn open_stream(
    path: &Path,
    format: Format,
    options: &ConvertOptions,
    registry: &StreamingCodecRegistry,
) -> Result<Box<dyn PointStream>> {
    if can_native_stream_read(format) {
        return open_native_point_stream(path, format, options);
    }
    if let Some(codec) = registry.reader(format) {
        return codec.open_point_stream(path, format, options);
    }
    Err(Error::unsupported(
        format,
        "stream read",
        format.adapter_hint(),
    ))
}

fn create_writer(
    path: &Path,
    format: Format,
    info: &PointStreamInfo,
    options: &ConvertOptions,
    registry: &StreamingCodecRegistry,
) -> Result<Box<dyn PointStreamWriter>> {
    if can_native_stream_write(format, info) {
        return create_native_point_writer(path, format, info, options);
    }
    if let Some(codec) = registry.writer(format) {
        return codec.create_point_writer(path, format, info, options);
    }
    Err(Error::unsupported(
        format,
        "stream write",
        "streaming writer needs a point count/schema hint or an adapter",
    ))
}

pub(crate) fn try_convert_path_streaming(
    input: &Path,
    output: &Path,
    options: &ConvertOptions,
) -> Result<Option<ConversionReport>> {
    let input_format = options
        .input_format
        .map(Ok)
        .unwrap_or_else(|| Format::from_path(input))?;
    let output_format = options
        .output_format
        .map(Ok)
        .unwrap_or_else(|| Format::from_path(output))?;

    if !can_native_stream_read(input_format) {
        return Ok(None);
    }

    let mut stream = open_native_point_stream(input, input_format, options)?;
    let info = stream.info().clone();
    if !can_native_stream_write(output_format, &info) {
        return Ok(None);
    }
    let mut writer = create_native_point_writer(output, output_format, &info, options)?;
    let mut points = 0usize;
    while let Some(point) = stream.next_point()? {
        writer.write_point(&point)?;
        points += 1;
    }
    writer.finish()?;

    Ok(Some(ConversionReport {
        input_format,
        output_format,
        points_read: points,
        points_written: points,
        faces_read: 0,
        faces_written: 0,
        warnings: info.metadata.warnings.clone(),
    }))
}

fn open_native_point_stream(
    path: &Path,
    format: Format,
    options: &ConvertOptions,
) -> Result<Box<dyn PointStream>> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);
    match format {
        Format::Xyz | Format::Txt | Format::Csv => {
            let mut opts = options.native.delimited.clone();
            if matches!(format, Format::Csv) && matches!(opts.delimiter, Delimiter::Auto) {
                opts.delimiter = Delimiter::Comma;
                opts.write_header = true;
            }
            Ok(Box::new(DelimitedPointStream::new(reader, format, opts)?))
        }
        Format::Pts => Ok(Box::new(PtsPointStream::new(reader)?)),
        _ => Err(Error::unsupported(
            format,
            "stream read",
            format.adapter_hint(),
        )),
    }
}

fn create_native_point_writer(
    path: &Path,
    format: Format,
    info: &PointStreamInfo,
    options: &ConvertOptions,
) -> Result<Box<dyn PointStreamWriter>> {
    let file = File::create(path)?;
    let writer = BufWriter::new(file);
    match format {
        Format::Xyz | Format::Txt | Format::Csv => {
            let mut opts = options.native.delimited.clone();
            if matches!(format, Format::Csv) {
                if matches!(opts.delimiter, Delimiter::Auto) {
                    opts.delimiter = Delimiter::Comma;
                }
                opts.write_header = true;
            }
            Ok(Box::new(DelimitedPointWriter::new(
                writer, format, info, opts,
            )?))
        }
        Format::Pts => Ok(Box::new(PtsPointWriter::new(writer, info)?)),
        Format::Ply => Ok(Box::new(PlyPointWriter::new(
            writer,
            info,
            options.native.ply.encoding,
            options.native.ply.precision,
        )?)),
        Format::Pcd => Ok(Box::new(PcdPointWriter::new(
            writer,
            info,
            options.native.pcd.encoding,
            options.native.pcd.precision,
        )?)),
        _ => Err(Error::unsupported(
            format,
            "stream write",
            format.adapter_hint(),
        )),
    }
}

struct DelimitedPointStream<R: BufRead + Send> {
    reader: R,
    info: PointStreamInfo,
    format: Format,
    mapping: ColumnMapping,
    delimiter: Delimiter,
    header_decided: Option<bool>,
    first_data_seen: bool,
    line: String,
    line_no: usize,
    buffered: Option<Point>,
}

impl<R: BufRead + Send> DelimitedPointStream<R> {
    fn new(reader: R, format: Format, options: DelimitedOptions) -> Result<Self> {
        let mut stream = Self {
            reader,
            info: PointStreamInfo::new(format),
            format,
            mapping: options.columns,
            delimiter: options.delimiter,
            header_decided: options.has_header,
            first_data_seen: false,
            line: String::new(),
            line_no: 0,
            buffered: None,
        };
        stream.buffered = stream.read_next_point()?;
        if let Some(point) = &stream.buffered {
            stream.info.fields.include_point(point);
        }
        Ok(stream)
    }

    fn read_next_point(&mut self) -> Result<Option<Point>> {
        loop {
            self.line.clear();
            let bytes_read = self.reader.read_line(&mut self.line)?;
            if bytes_read == 0 {
                return Ok(None);
            }
            self.line_no += 1;
            let trimmed = self.line.trim();
            if trimmed.is_empty() || trimmed.starts_with('#') || trimmed.starts_with("//") {
                continue;
            }

            if self.delimiter == Delimiter::Auto {
                self.delimiter = Delimiter::detect(trimmed);
            }
            let mut fields_buf = [""; 64];
            let fields_len = self.delimiter.split_into_slice(trimmed, &mut fields_buf);
            let fields = &fields_buf[..fields_len];
            if fields.is_empty() {
                continue;
            }

            if !self.first_data_seen {
                let line_is_header = match self.header_decided {
                    Some(value) => value,
                    None => {
                        !looks_like_point_line(self.format, self.line_no, &self.mapping, fields)
                    }
                };
                self.header_decided = Some(line_is_header);
                self.first_data_seen = true;
                if line_is_header {
                    if let Some(header_mapping) = ColumnMapping::from_header(fields) {
                        self.info.fields = fields_from_mapping(&header_mapping);
                        self.mapping = header_mapping;
                    } else {
                        return Err(Error::parse(
                            self.format,
                            self.line_no,
                            "header must include x/y/z columns",
                        ));
                    }
                    continue;
                }
            }

            let point = crate::io::delimited::parse_point_fields(
                self.format,
                self.line_no,
                &self.mapping,
                fields,
            )?;
            return Ok(Some(point));
        }
    }
}

impl<R: BufRead + Send> PointStream for DelimitedPointStream<R> {
    fn info(&self) -> &PointStreamInfo {
        &self.info
    }

    fn next_point(&mut self) -> Result<Option<Point>> {
        if let Some(point) = self.buffered.take() {
            return Ok(Some(point));
        }
        self.read_next_point()
    }
}

struct PtsPointStream<R: BufRead + Send> {
    reader: R,
    info: PointStreamInfo,
    line: String,
    line_no: usize,
    first_payload_line: bool,
    buffered: Option<Point>,
}

impl<R: BufRead + Send> PtsPointStream<R> {
    fn new(reader: R) -> Result<Self> {
        let mut stream = Self {
            reader,
            info: PointStreamInfo::new(Format::Pts),
            line: String::new(),
            line_no: 0,
            first_payload_line: true,
            buffered: None,
        };
        stream.buffered = stream.read_next_point()?;
        if let Some(point) = &stream.buffered {
            stream.info.fields.include_point(point);
        }
        Ok(stream)
    }

    fn read_next_point(&mut self) -> Result<Option<Point>> {
        loop {
            self.line.clear();
            if self.reader.read_line(&mut self.line)? == 0 {
                return Ok(None);
            }
            self.line_no += 1;
            let trimmed = self.line.trim();
            if trimmed.is_empty() || trimmed.starts_with('#') || trimmed.starts_with("//") {
                continue;
            }
            let mut parts_buf = [""; 16];
            let mut count = 0;
            for part in trimmed.split_whitespace() {
                if count < 16 {
                    parts_buf[count] = part;
                    count += 1;
                } else {
                    break;
                }
            }
            let parts = &parts_buf[..count];
            if self.first_payload_line && parts.len() == 1 {
                if let Ok(count) = parts[0].parse::<usize>() {
                    self.info.point_count_hint = Some(count);
                    self.info.metadata.point_count_hint = Some(count);
                    self.first_payload_line = false;
                    continue;
                }
            }
            self.first_payload_line = false;
            return Ok(Some(crate::io::pts::parse_pts_point(self.line_no, parts)?));
        }
    }
}

impl<R: BufRead + Send> PointStream for PtsPointStream<R> {
    fn info(&self) -> &PointStreamInfo {
        &self.info
    }

    fn next_point(&mut self) -> Result<Option<Point>> {
        if let Some(point) = self.buffered.take() {
            return Ok(Some(point));
        }
        self.read_next_point()
    }
}

struct DelimitedPointWriter<W: Write + Send> {
    writer: W,
    format: Format,
    fields: PointFieldSet,
    delimiter: Delimiter,
    precision: usize,
}

impl<W: Write + Send> DelimitedPointWriter<W> {
    fn new(
        mut writer: W,
        format: Format,
        info: &PointStreamInfo,
        options: DelimitedOptions,
    ) -> Result<Self> {
        let delimiter = match options.delimiter {
            Delimiter::Auto => {
                if matches!(format, Format::Csv) {
                    Delimiter::Comma
                } else {
                    Delimiter::Whitespace
                }
            }
            other => other,
        };
        let fields = info.fields;
        if options.write_header || matches!(format, Format::Csv) {
            write_delimited_header(&mut writer, delimiter.as_str(), fields)?;
        }
        Ok(Self {
            writer,
            format,
            fields,
            delimiter,
            precision: options.precision,
        })
    }
}

impl<W: Write + Send> PointStreamWriter for DelimitedPointWriter<W> {
    fn write_point(&mut self, point: &Point) -> Result<()> {
        let sep = self.delimiter.as_str();
        write_point_delimited(
            &mut self.writer,
            self.format,
            point,
            self.fields,
            sep,
            self.precision,
        )
    }

    fn finish(&mut self) -> Result<()> {
        self.writer.flush()?;
        Ok(())
    }
}

struct PtsPointWriter<W: Write + Send> {
    writer: W,
    fields: PointFieldSet,
}

impl<W: Write + Send> PtsPointWriter<W> {
    fn new(mut writer: W, info: &PointStreamInfo) -> Result<Self> {
        let count = required_count(info, Format::Pts)?;
        writeln!(writer, "{count}")?;
        Ok(Self {
            writer,
            fields: info.fields,
        })
    }
}

impl<W: Write + Send> PointStreamWriter for PtsPointWriter<W> {
    fn write_point(&mut self, point: &Point) -> Result<()> {
        crate::io::write_fmt_f64(&mut self.writer, point.position.x, 6)?;
        write!(self.writer, " ")?;
        crate::io::write_fmt_f64(&mut self.writer, point.position.y, 6)?;
        write!(self.writer, " ")?;
        crate::io::write_fmt_f64(&mut self.writer, point.position.z, 6)?;
        if self.fields.intensity {
            write!(self.writer, " {}", point.intensity.unwrap_or(0.0))?;
        }
        if self.fields.color {
            let color = point.color.unwrap_or(crate::Color::new(0, 0, 0));
            write!(self.writer, " {} {} {}", color.red, color.green, color.blue)?;
        }
        writeln!(self.writer)?;
        Ok(())
    }

    fn finish(&mut self) -> Result<()> {
        self.writer.flush()?;
        Ok(())
    }
}

struct PlyPointWriter<W: Write + Send> {
    writer: W,
    fields: PointFieldSet,
    encoding: PlyEncoding,
    precision: usize,
}

impl<W: Write + Send> PlyPointWriter<W> {
    fn new(
        mut writer: W,
        info: &PointStreamInfo,
        encoding: PlyEncoding,
        precision: usize,
    ) -> Result<Self> {
        let count = required_count(info, Format::Ply)?;
        write_ply_header(&mut writer, count, info.fields, encoding)?;
        Ok(Self {
            writer,
            fields: info.fields,
            encoding,
            precision,
        })
    }
}

impl<W: Write + Send> PointStreamWriter for PlyPointWriter<W> {
    fn write_point(&mut self, point: &Point) -> Result<()> {
        match self.encoding {
            PlyEncoding::Ascii => {
                write_ply_ascii_point(&mut self.writer, point, self.fields, self.precision)
            }
            PlyEncoding::BinaryLittleEndian => {
                write_ply_binary_point(&mut self.writer, point, self.fields)
            }
        }
    }

    fn finish(&mut self) -> Result<()> {
        self.writer.flush()?;
        Ok(())
    }
}

struct PcdPointWriter<W: Write + Send> {
    writer: W,
    fields: Vec<PcdWriteField>,
    encoding: PcdEncoding,
    precision: usize,
}

impl<W: Write + Send> PcdPointWriter<W> {
    fn new(
        mut writer: W,
        info: &PointStreamInfo,
        encoding: PcdEncoding,
        precision: usize,
    ) -> Result<Self> {
        let count = required_count(info, Format::Pcd)?;
        let fields = pcd_fields(info.fields);
        write_pcd_header(&mut writer, count, &fields, encoding)?;
        Ok(Self {
            writer,
            fields,
            encoding,
            precision,
        })
    }
}

impl<W: Write + Send> PointStreamWriter for PcdPointWriter<W> {
    fn write_point(&mut self, point: &Point) -> Result<()> {
        match self.encoding {
            PcdEncoding::Ascii => {
                write_pcd_ascii_point(&mut self.writer, point, &self.fields, self.precision)
            }
            PcdEncoding::Binary => write_pcd_binary_point(&mut self.writer, point, &self.fields),
        }
    }

    fn finish(&mut self) -> Result<()> {
        self.writer.flush()?;
        Ok(())
    }
}

struct CloudPointStream {
    info: PointStreamInfo,
    points: std::vec::IntoIter<Point>,
}

impl CloudPointStream {
    #[allow(dead_code)]
    fn new(format: Format, cloud: PointCloud) -> Self {
        let info = PointStreamInfo::from_cloud(format, &cloud);
        Self {
            info,
            points: cloud.points.into_iter(),
        }
    }
}

impl PointStream for CloudPointStream {
    fn info(&self) -> &PointStreamInfo {
        &self.info
    }

    fn next_point(&mut self) -> Result<Option<Point>> {
        Ok(self.points.next())
    }
}

fn required_count(info: &PointStreamInfo, format: Format) -> Result<usize> {
    info.point_count_hint.ok_or_else(|| {
        Error::unsupported(
            format,
            "stream write",
            "this format requires point_count_hint before streaming can write its header",
        )
    })
}

fn fields_from_mapping(mapping: &ColumnMapping) -> PointFieldSet {
    PointFieldSet {
        intensity: mapping.intensity.is_some(),
        color: mapping.red.is_some() && mapping.green.is_some() && mapping.blue.is_some(),
        classification: mapping.classification.is_some(),
        gps_time: mapping.gps_time.is_some(),
        normals: mapping.normal_x.is_some()
            && mapping.normal_y.is_some()
            && mapping.normal_z.is_some(),
    }
}

fn looks_like_point_line(
    format: Format,
    line_no: usize,
    mapping: &ColumnMapping,
    fields: &[&str],
) -> bool {
    [mapping.x, mapping.y, mapping.z].iter().all(|&idx| {
        fields
            .get(idx)
            .copied()
            .map(|value| crate::io::parse_f64(format, line_no, "coordinate", value).is_ok())
            .unwrap_or(false)
    })
}

fn write_delimited_header<W: Write>(
    writer: &mut W,
    sep: &str,
    fields: PointFieldSet,
) -> Result<()> {
    let mut header = vec!["x", "y", "z"];
    if fields.intensity {
        header.push("intensity");
    }
    if fields.color {
        header.extend(["red", "green", "blue"]);
    }
    if fields.classification {
        header.push("classification");
    }
    if fields.gps_time {
        header.push("gps_time");
    }
    if fields.normals {
        header.extend(["normal_x", "normal_y", "normal_z"]);
    }
    writeln!(writer, "{}", header.join(sep))?;
    Ok(())
}

fn write_point_delimited<W: Write>(
    writer: &mut W,
    _format: Format,
    point: &Point,
    fields: PointFieldSet,
    sep: &str,
    precision: usize,
) -> Result<()> {
    crate::io::write_fmt_f64(writer, point.position.x, precision)?;
    write!(writer, "{sep}")?;
    crate::io::write_fmt_f64(writer, point.position.y, precision)?;
    write!(writer, "{sep}")?;
    crate::io::write_fmt_f64(writer, point.position.z, precision)?;
    if fields.intensity {
        write!(writer, "{sep}")?;
        if let Some(v) = point.intensity {
            write!(writer, "{v:.*}", precision)?;
        }
    }
    if fields.color {
        write!(writer, "{sep}")?;
        if let Some(color) = point.color {
            write!(
                writer,
                "{}{sep}{}{sep}{}",
                color.red, color.green, color.blue
            )?;
        } else {
            write!(writer, "{sep}{sep}")?;
        }
    }
    if fields.classification {
        write!(writer, "{sep}")?;
        if let Some(v) = point.classification {
            write!(writer, "{v}")?;
        }
    }
    if fields.gps_time {
        write!(writer, "{sep}")?;
        if let Some(v) = point.gps_time {
            crate::io::write_fmt_f64(writer, v, precision)?;
        }
    }
    if fields.normals {
        write!(writer, "{sep}")?;
        if let Some(normal) = point.normal {
            crate::io::write_fmt_f64(writer, normal.x, precision)?;
            write!(writer, "{sep}")?;
            crate::io::write_fmt_f64(writer, normal.y, precision)?;
            write!(writer, "{sep}")?;
            crate::io::write_fmt_f64(writer, normal.z, precision)?;
        } else {
            write!(writer, "{sep}{sep}")?;
        }
    }
    writeln!(writer)?;
    Ok(())
}

fn write_ply_header<W: Write>(
    writer: &mut W,
    count: usize,
    fields: PointFieldSet,
    encoding: PlyEncoding,
) -> Result<()> {
    writeln!(writer, "ply")?;
    match encoding {
        PlyEncoding::Ascii => writeln!(writer, "format ascii 1.0")?,
        PlyEncoding::BinaryLittleEndian => writeln!(writer, "format binary_little_endian 1.0")?,
    }
    writeln!(writer, "comment created by point-formats")?;
    writeln!(writer, "element vertex {count}")?;
    writeln!(writer, "property double x")?;
    writeln!(writer, "property double y")?;
    writeln!(writer, "property double z")?;
    if fields.normals {
        writeln!(writer, "property double nx")?;
        writeln!(writer, "property double ny")?;
        writeln!(writer, "property double nz")?;
    }
    if fields.intensity {
        writeln!(writer, "property float intensity")?;
    }
    if fields.color {
        writeln!(writer, "property ushort red")?;
        writeln!(writer, "property ushort green")?;
        writeln!(writer, "property ushort blue")?;
    }
    if fields.classification {
        writeln!(writer, "property uchar classification")?;
    }
    writeln!(writer, "end_header")?;
    Ok(())
}

fn write_ply_ascii_point<W: Write>(
    writer: &mut W,
    point: &Point,
    fields: PointFieldSet,
    precision: usize,
) -> Result<()> {
    crate::io::write_fmt_f64(writer, point.position.x, precision)?;
    write!(writer, " ")?;
    crate::io::write_fmt_f64(writer, point.position.y, precision)?;
    write!(writer, " ")?;
    crate::io::write_fmt_f64(writer, point.position.z, precision)?;
    if fields.normals {
        let normal = point.normal.unwrap_or(Vec3::ZERO);
        write!(writer, " ")?;
        crate::io::write_fmt_f64(writer, normal.x, precision)?;
        write!(writer, " ")?;
        crate::io::write_fmt_f64(writer, normal.y, precision)?;
        write!(writer, " ")?;
        crate::io::write_fmt_f64(writer, normal.z, precision)?;
    }
    if fields.intensity {
        write!(writer, " {:.*}", precision, point.intensity.unwrap_or(0.0))?;
    }
    if fields.color {
        let color = point.color.unwrap_or(crate::Color::new(0, 0, 0));
        write!(writer, " {} {} {}", color.red, color.green, color.blue)?;
    }
    if fields.classification {
        write!(writer, " {}", point.classification.unwrap_or(0))?;
    }
    writeln!(writer)?;
    Ok(())
}

fn write_ply_binary_point<W: Write>(
    writer: &mut W,
    point: &Point,
    fields: PointFieldSet,
) -> Result<()> {
    writer.write_all(&point.position.x.to_le_bytes())?;
    writer.write_all(&point.position.y.to_le_bytes())?;
    writer.write_all(&point.position.z.to_le_bytes())?;
    if fields.normals {
        let normal = point.normal.unwrap_or(Vec3::ZERO);
        writer.write_all(&normal.x.to_le_bytes())?;
        writer.write_all(&normal.y.to_le_bytes())?;
        writer.write_all(&normal.z.to_le_bytes())?;
    }
    if fields.intensity {
        writer.write_all(&point.intensity.unwrap_or(0.0).to_le_bytes())?;
    }
    if fields.color {
        let color = point.color.unwrap_or(crate::Color::new(0, 0, 0));
        writer.write_all(&color.red.to_le_bytes())?;
        writer.write_all(&color.green.to_le_bytes())?;
        writer.write_all(&color.blue.to_le_bytes())?;
    }
    if fields.classification {
        writer.write_all(&[point.classification.unwrap_or(0)])?;
    }
    Ok(())
}

#[derive(Debug, Clone, Copy)]
struct PcdWriteField {
    name: &'static str,
    size: usize,
    ty: char,
    kind: PcdWriteFieldKind,
}

#[derive(Debug, Clone, Copy)]
enum PcdWriteFieldKind {
    X,
    Y,
    Z,
    Intensity,
    Red,
    Green,
    Blue,
    Classification,
    NormalX,
    NormalY,
    NormalZ,
}

fn pcd_fields(fields: PointFieldSet) -> Vec<PcdWriteField> {
    let mut out = vec![
        PcdWriteField {
            name: "x",
            size: 8,
            ty: 'F',
            kind: PcdWriteFieldKind::X,
        },
        PcdWriteField {
            name: "y",
            size: 8,
            ty: 'F',
            kind: PcdWriteFieldKind::Y,
        },
        PcdWriteField {
            name: "z",
            size: 8,
            ty: 'F',
            kind: PcdWriteFieldKind::Z,
        },
    ];
    if fields.intensity {
        out.push(PcdWriteField {
            name: "intensity",
            size: 4,
            ty: 'F',
            kind: PcdWriteFieldKind::Intensity,
        });
    }
    if fields.color {
        out.extend([
            PcdWriteField {
                name: "red",
                size: 2,
                ty: 'U',
                kind: PcdWriteFieldKind::Red,
            },
            PcdWriteField {
                name: "green",
                size: 2,
                ty: 'U',
                kind: PcdWriteFieldKind::Green,
            },
            PcdWriteField {
                name: "blue",
                size: 2,
                ty: 'U',
                kind: PcdWriteFieldKind::Blue,
            },
        ]);
    }
    if fields.classification {
        out.push(PcdWriteField {
            name: "classification",
            size: 1,
            ty: 'U',
            kind: PcdWriteFieldKind::Classification,
        });
    }
    if fields.normals {
        out.extend([
            PcdWriteField {
                name: "normal_x",
                size: 8,
                ty: 'F',
                kind: PcdWriteFieldKind::NormalX,
            },
            PcdWriteField {
                name: "normal_y",
                size: 8,
                ty: 'F',
                kind: PcdWriteFieldKind::NormalY,
            },
            PcdWriteField {
                name: "normal_z",
                size: 8,
                ty: 'F',
                kind: PcdWriteFieldKind::NormalZ,
            },
        ]);
    }
    out
}

fn write_pcd_header<W: Write>(
    writer: &mut W,
    count: usize,
    fields: &[PcdWriteField],
    encoding: PcdEncoding,
) -> Result<()> {
    writeln!(
        writer,
        "# .PCD v0.7 - Point Cloud Data file generated by point-formats"
    )?;
    writeln!(writer, "VERSION 0.7")?;
    writeln!(
        writer,
        "FIELDS {}",
        fields.iter().map(|f| f.name).collect::<Vec<_>>().join(" ")
    )?;
    writeln!(
        writer,
        "SIZE {}",
        fields
            .iter()
            .map(|f| f.size.to_string())
            .collect::<Vec<_>>()
            .join(" ")
    )?;
    writeln!(
        writer,
        "TYPE {}",
        fields
            .iter()
            .map(|f| f.ty.to_string())
            .collect::<Vec<_>>()
            .join(" ")
    )?;
    writeln!(writer, "COUNT {}", vec!["1"; fields.len()].join(" "))?;
    writeln!(writer, "WIDTH {count}")?;
    writeln!(writer, "HEIGHT 1")?;
    writeln!(writer, "VIEWPOINT 0 0 0 1 0 0 0")?;
    writeln!(writer, "POINTS {count}")?;
    match encoding {
        PcdEncoding::Ascii => writeln!(writer, "DATA ascii")?,
        PcdEncoding::Binary => writeln!(writer, "DATA binary")?,
    }
    Ok(())
}

fn write_pcd_ascii_point<W: Write>(
    writer: &mut W,
    point: &Point,
    fields: &[PcdWriteField],
    precision: usize,
) -> Result<()> {
    for (idx, field) in fields.iter().enumerate() {
        if idx > 0 {
            write!(writer, " ")?;
        }
        match field.kind {
            PcdWriteFieldKind::X => crate::io::write_fmt_f64(writer, point.position.x, precision)?,
            PcdWriteFieldKind::Y => crate::io::write_fmt_f64(writer, point.position.y, precision)?,
            PcdWriteFieldKind::Z => crate::io::write_fmt_f64(writer, point.position.z, precision)?,
            PcdWriteFieldKind::Intensity => {
                write!(writer, "{:.*}", precision, point.intensity.unwrap_or(0.0))?
            }
            PcdWriteFieldKind::Red => write!(
                writer,
                "{}",
                point.color.unwrap_or(crate::Color::new(0, 0, 0)).red
            )?,
            PcdWriteFieldKind::Green => write!(
                writer,
                "{}",
                point.color.unwrap_or(crate::Color::new(0, 0, 0)).green
            )?,
            PcdWriteFieldKind::Blue => write!(
                writer,
                "{}",
                point.color.unwrap_or(crate::Color::new(0, 0, 0)).blue
            )?,
            PcdWriteFieldKind::Classification => {
                write!(writer, "{}", point.classification.unwrap_or(0))?
            }
            PcdWriteFieldKind::NormalX => {
                crate::io::write_fmt_f64(writer, point.normal.unwrap_or(Vec3::ZERO).x, precision)?
            }
            PcdWriteFieldKind::NormalY => {
                crate::io::write_fmt_f64(writer, point.normal.unwrap_or(Vec3::ZERO).y, precision)?
            }
            PcdWriteFieldKind::NormalZ => {
                crate::io::write_fmt_f64(writer, point.normal.unwrap_or(Vec3::ZERO).z, precision)?
            }
        }
    }
    writeln!(writer)?;
    Ok(())
}

fn write_pcd_binary_point<W: Write>(
    writer: &mut W,
    point: &Point,
    fields: &[PcdWriteField],
) -> Result<()> {
    for field in fields {
        match field.kind {
            PcdWriteFieldKind::X => writer.write_all(&point.position.x.to_le_bytes())?,
            PcdWriteFieldKind::Y => writer.write_all(&point.position.y.to_le_bytes())?,
            PcdWriteFieldKind::Z => writer.write_all(&point.position.z.to_le_bytes())?,
            PcdWriteFieldKind::Intensity => {
                writer.write_all(&point.intensity.unwrap_or(0.0).to_le_bytes())?
            }
            PcdWriteFieldKind::Red => writer.write_all(
                &point
                    .color
                    .unwrap_or(crate::Color::new(0, 0, 0))
                    .red
                    .to_le_bytes(),
            )?,
            PcdWriteFieldKind::Green => writer.write_all(
                &point
                    .color
                    .unwrap_or(crate::Color::new(0, 0, 0))
                    .green
                    .to_le_bytes(),
            )?,
            PcdWriteFieldKind::Blue => writer.write_all(
                &point
                    .color
                    .unwrap_or(crate::Color::new(0, 0, 0))
                    .blue
                    .to_le_bytes(),
            )?,
            PcdWriteFieldKind::Classification => {
                writer.write_all(&[point.classification.unwrap_or(0)])?
            }
            PcdWriteFieldKind::NormalX => {
                writer.write_all(&point.normal.unwrap_or(Vec3::ZERO).x.to_le_bytes())?
            }
            PcdWriteFieldKind::NormalY => {
                writer.write_all(&point.normal.unwrap_or(Vec3::ZERO).y.to_le_bytes())?
            }
            PcdWriteFieldKind::NormalZ => {
                writer.write_all(&point.normal.unwrap_or(Vec3::ZERO).z.to_le_bytes())?
            }
        }
    }
    Ok(())
}

#[allow(dead_code)]
fn geometry_to_stream(
    geometry: Geometry,
    format: Format,
    output_format: Format,
    options: &ConvertOptions,
) -> Result<CloudPointStream> {
    let geometry = apply_geometry_policy(geometry, output_format, options)?;
    match geometry {
        Geometry::PointCloud(cloud) => Ok(CloudPointStream::new(format, cloud)),
        Geometry::Mesh(_) => Err(Error::invalid(
            "streaming point conversion kept mesh geometry",
        )),
    }
}
