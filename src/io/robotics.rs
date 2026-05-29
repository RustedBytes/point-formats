use crate::error::{Error, Result};
use crate::types::{Color, Geometry, Point, PointCloud};
use rusqlite::{params, Connection};
use std::io::{Read, Write};
use std::path::Path;

/// Reads a point cloud from a raw PointCloud2 message file.
pub fn read_pc2(path: impl AsRef<Path>) -> Result<Geometry> {
    let data = std::fs::read(path)?;
    let points = deserialize_pointcloud2(&data)?;
    Ok(Geometry::PointCloud(PointCloud::new(points)))
}

/// Writes a point cloud to a raw PointCloud2 message file.
pub fn write_pc2(path: impl AsRef<Path>, cloud: &PointCloud) -> Result<()> {
    let data = serialize_pointcloud2(cloud);
    std::fs::write(path, data)?;
    Ok(())
}

/// Reads a point cloud from a ROS 1 bag file.
pub fn read_rosbag(path: impl AsRef<Path>) -> Result<Geometry> {
    let file = std::fs::File::open(path)?;
    let mut reader = std::io::BufReader::new(file);

    let mut magic = [0_u8; 13];
    reader.read_exact(&mut magic)?;
    if &magic != b"#ROSBAG V2.0\n" {
        return Err(Error::invalid("Invalid ROS 1 bag file magic"));
    }

    let mut points = Vec::new();
    let mut target_conn = None;

    loop {
        let mut header_len_bytes = [0_u8; 4];
        if reader.read_exact(&mut header_len_bytes).is_err() {
            break; // End of file
        }
        let header_len = u32::from_le_bytes(header_len_bytes) as usize;

        let mut header_bytes = vec![0_u8; header_len];
        reader.read_exact(&mut header_bytes)?;

        let mut data_len_bytes = [0_u8; 4];
        reader.read_exact(&mut data_len_bytes)?;
        let data_len = u32::from_le_bytes(data_len_bytes) as usize;

        let mut data_bytes = vec![0_u8; data_len];
        reader.read_exact(&mut data_bytes)?;

        // Parse header key=value fields
        let mut op = None;
        let mut conn = None;
        let mut topic = None;

        let mut offset = 0;
        while offset < header_bytes.len() {
            if offset + 4 > header_bytes.len() {
                break;
            }
            let field_len =
                u32::from_le_bytes(header_bytes[offset..offset + 4].try_into().unwrap()) as usize;
            offset += 4;
            if offset + field_len > header_bytes.len() {
                break;
            }
            let field = &header_bytes[offset..offset + field_len];
            offset += field_len;

            if let Some(pos) = field.iter().position(|&x| x == b'=') {
                let key = &field[0..pos];
                let val = &field[pos + 1..];
                match key {
                    b"op" => {
                        if !val.is_empty() {
                            op = Some(val[0]);
                        }
                    }
                    b"conn" => {
                        if val.len() == 4 {
                            conn = Some(u32::from_le_bytes(val.try_into().unwrap()));
                        }
                    }
                    b"topic" => {
                        topic = Some(String::from_utf8_lossy(val).into_owned());
                    }
                    _ => {}
                }
            }
        }

        match op {
            Some(0x07) => {
                if let (Some(c), Some(t)) = (conn, topic) {
                    if t.contains("point")
                        || t.contains("points")
                        || t.contains("lidar")
                        || target_conn.is_none()
                    {
                        target_conn = Some(c);
                    }
                }
            }
            Some(0x02) => {
                if let Some(c) = conn {
                    if Some(c) == target_conn {
                        let parsed = deserialize_pointcloud2(&data_bytes)?;
                        points.extend(parsed);
                    }
                }
            }
            _ => {}
        }
    }

    if points.is_empty() {
        return Err(Error::invalid("No PointCloud2 messages found in ROS 1 bag"));
    }

    Ok(Geometry::PointCloud(PointCloud::new(points)))
}

/// Writes a point cloud to a ROS 1 bag file.
pub fn write_rosbag(path: impl AsRef<Path>, cloud: &PointCloud) -> Result<()> {
    let file = std::fs::File::create(path)?;
    let mut writer = std::io::BufWriter::new(file);

    writer.write_all(b"#ROSBAG V2.0\n")?;

    // BagHeader record
    let mut header_fields = Vec::new();
    header_fields.push(("op", &[0x03][..]));
    let index_pos_bytes = 0_u64.to_le_bytes();
    header_fields.push(("index_pos", &index_pos_bytes[..]));
    let conn_count_bytes = 1_u32.to_le_bytes();
    header_fields.push(("conn_count", &conn_count_bytes[..]));
    let chunk_count_bytes = 0_u32.to_le_bytes();
    header_fields.push(("chunk_count", &chunk_count_bytes[..]));

    let padding = vec![0x20; 4096];
    write_record(&mut writer, &header_fields, &padding)?;

    // Connection record
    let mut conn_fields = Vec::new();
    conn_fields.push(("op", &[0x07][..]));
    let conn_bytes = 0_u32.to_le_bytes();
    conn_fields.push(("conn", &conn_bytes[..]));
    conn_fields.push(("topic", b"/points"));

    let conn_data = serialize_connection_data("/points");
    write_record(&mut writer, &conn_fields, &conn_data)?;

    // MessageData record
    let mut msg_fields = Vec::new();
    msg_fields.push(("op", &[0x02][..]));
    msg_fields.push(("conn", &conn_bytes[..]));
    let stamp_sec = cloud.points.first().and_then(|p| p.gps_time).unwrap_or(0.0) as u32;
    let stamp_nsec = ((cloud
        .points
        .first()
        .and_then(|p| p.gps_time)
        .unwrap_or(0.0)
        .fract())
        * 1e9) as u32;
    let mut time_bytes = [0_u8; 8];
    time_bytes[0..4].copy_from_slice(&stamp_sec.to_le_bytes());
    time_bytes[4..8].copy_from_slice(&stamp_nsec.to_le_bytes());
    msg_fields.push(("time", &time_bytes[..]));

    let msg_data = serialize_pointcloud2(cloud);
    write_record(&mut writer, &msg_fields, &msg_data)?;

    Ok(())
}

/// Reads a point cloud from a ROS 2 SQLite bag file.
pub fn read_ros2bag(path: impl AsRef<Path>) -> Result<Geometry> {
    let conn = Connection::open(path)
        .map_err(|e| Error::invalid(format!("Failed to open ROS 2 SQLite bag: {}", e)))?;

    let mut stmt = conn
        .prepare("SELECT id FROM topics WHERE type = 'sensor_msgs/msg/PointCloud2' OR type = 'sensor_msgs/PointCloud2'")
        .map_err(|e| Error::invalid(format!("Failed to query topics table: {}", e)))?;
    let mut rows = stmt.query([])?;

    let mut topic_id = None;
    if let Some(row) = rows.next()? {
        let tid: i64 = row.get(0)?;
        topic_id = Some(tid);
    }

    let tid = match topic_id {
        Some(id) => id,
        None => {
            // Try any topic if PointCloud2 was not matched exactly
            let mut stmt_any = conn.prepare("SELECT id FROM topics")?;
            let mut rows_any = stmt_any.query([])?;
            if let Some(row) = rows_any.next()? {
                row.get(0)?
            } else {
                return Err(Error::invalid("No topics registered in ROS 2 bag"));
            }
        }
    };

    let mut msg_stmt =
        conn.prepare("SELECT data FROM messages WHERE topic_id = ?1 ORDER BY timestamp")?;
    let mut msg_rows = msg_stmt.query(params![tid])?;

    let mut points = Vec::new();
    while let Some(row) = msg_rows.next()? {
        let cdr_blob: Vec<u8> = row.get(0)?;
        let parsed = deserialize_pointcloud2_cdr(&cdr_blob)?;
        points.extend(parsed);
    }

    if points.is_empty() {
        return Err(Error::invalid("No PointCloud2 messages found in ROS 2 bag"));
    }

    Ok(Geometry::PointCloud(PointCloud::new(points)))
}

/// Writes a point cloud to a ROS 2 SQLite bag file.
pub fn write_ros2bag(path: impl AsRef<Path>, cloud: &PointCloud) -> Result<()> {
    if path.as_ref().exists() {
        let _ = std::fs::remove_file(path.as_ref());
    }

    let conn = Connection::open(path)
        .map_err(|e| Error::invalid(format!("Failed to open ROS 2 SQLite bag: {}", e)))?;

    conn.execute(
        "CREATE TABLE topics (
            id INTEGER PRIMARY KEY,
            name TEXT NOT NULL,
            type TEXT NOT NULL,
            serialization_format TEXT NOT NULL,
            offered_qos_profiles TEXT NOT NULL
        )",
        [],
    )?;

    conn.execute(
        "CREATE TABLE messages (
            id INTEGER PRIMARY KEY,
            topic_id INTEGER NOT NULL,
            timestamp INTEGER NOT NULL,
            data BLOB NOT NULL
        )",
        [],
    )?;

    // Register topic
    conn.execute(
        "INSERT INTO topics (id, name, type, serialization_format, offered_qos_profiles)
         VALUES (1, '/points', 'sensor_msgs/msg/PointCloud2', 'cdr', '')",
        [],
    )?;

    let stamp_ns = cloud
        .points
        .first()
        .and_then(|p| p.gps_time)
        .map(|t| (t * 1_000_000_000.0) as i64)
        .unwrap_or_else(|| {
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as i64
        });

    let cdr_data = serialize_pointcloud2_cdr(cloud);

    conn.execute(
        "INSERT INTO messages (topic_id, timestamp, data) VALUES (1, ?1, ?2)",
        params![stamp_ns, cdr_data],
    )?;

    Ok(())
}

fn write_record<W: Write>(
    writer: &mut W,
    header: &[(&str, &[u8])],
    data: &[u8],
) -> std::io::Result<()> {
    let mut header_bytes = Vec::new();
    for &(key, val) in header {
        let field_len = key.len() + 1 + val.len();
        header_bytes.extend_from_slice(&(field_len as u32).to_le_bytes());
        header_bytes.extend_from_slice(key.as_bytes());
        header_bytes.push(b'=');
        header_bytes.extend_from_slice(val);
    }

    writer.write_all(&(header_bytes.len() as u32).to_le_bytes())?;
    writer.write_all(&header_bytes)?;
    writer.write_all(&(data.len() as u32).to_le_bytes())?;
    writer.write_all(data)?;
    Ok(())
}

fn serialize_connection_data(topic: &str) -> Vec<u8> {
    let mut header_bytes = Vec::new();
    let fields = [
        ("topic", topic),
        ("type", "sensor_msgs/PointCloud2"),
        ("md5sum", "1158d486dd51d683ce2f1be655c3c181"),
        ("message_definition", "Header header\nuint32 height\nuint32 width\nPointField[] fields\nbool is_bigendian\nuint32 point_step\nuint32 row_step\nuint8[] data\nbool is_dense\n================================================================================\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n================================================================================\nMSG: sensor_msgs/PointField\nuint8 INT8=1\nuint8 UINT8=2\nuint8 INT16=3\nuint8 UINT16=4\nuint8 INT32=5\nuint8 UINT32=6\nuint8 FLOAT32=7\nuint8 FLOAT64=8\nstring name\nuint32 offset\nuint8 datatype\nuint32 count\n"),
        ("callerid", "/point-formats"),
    ];

    for &(key, val) in &fields {
        let field_len = key.len() + 1 + val.len();
        header_bytes.extend_from_slice(&(field_len as u32).to_le_bytes());
        header_bytes.extend_from_slice(key.as_bytes());
        header_bytes.push(b'=');
        header_bytes.extend_from_slice(val.as_bytes());
    }
    header_bytes
}

fn serialize_pointcloud2(cloud: &PointCloud) -> Vec<u8> {
    let mut buf = Vec::new();

    // 1. Header
    buf.extend_from_slice(&0_u32.to_le_bytes()); // seq
    let stamp_sec = cloud.points.first().and_then(|p| p.gps_time).unwrap_or(0.0) as u32;
    let stamp_nsec = ((cloud
        .points
        .first()
        .and_then(|p| p.gps_time)
        .unwrap_or(0.0)
        .fract())
        * 1e9) as u32;
    buf.extend_from_slice(&stamp_sec.to_le_bytes());
    buf.extend_from_slice(&stamp_nsec.to_le_bytes());
    let frame_id = "map";
    buf.extend_from_slice(&(frame_id.len() as u32).to_le_bytes());
    buf.extend_from_slice(frame_id.as_bytes());

    // 2. height & width
    buf.extend_from_slice(&1_u32.to_le_bytes());
    buf.extend_from_slice(&(cloud.points.len() as u32).to_le_bytes());

    // 3. fields
    let has_intensity = cloud.has_intensity();
    let has_color = cloud.has_color();
    let has_class = cloud.has_classification();

    let mut fields: Vec<(&str, u32, u8, u32)> =
        vec![("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1)];
    let mut point_step = 12;
    if has_intensity {
        fields.push(("intensity", 12, 7, 1));
        point_step += 4;
    }
    if has_color {
        fields.push(("rgb", point_step, 6, 1));
        point_step += 4;
    }
    if has_class {
        fields.push(("classification", point_step, 2, 1));
        point_step += 4; // pad to 4 bytes boundary
    }

    buf.extend_from_slice(&(fields.len() as u32).to_le_bytes());
    for &(name, offset, datatype, count) in &fields {
        buf.extend_from_slice(&(name.len() as u32).to_le_bytes());
        buf.extend_from_slice(name.as_bytes());
        buf.extend_from_slice(&offset.to_le_bytes());
        buf.push(datatype);
        buf.extend_from_slice(&count.to_le_bytes());
    }

    // 4. Config
    buf.push(0x00); // little endian
    buf.extend_from_slice(&point_step.to_le_bytes());
    let row_step = point_step as usize * cloud.points.len();
    buf.extend_from_slice(&(row_step as u32).to_le_bytes());

    // 5. Data payload
    let mut data_bytes = Vec::with_capacity(row_step);
    for p in &cloud.points {
        let x = p.position.x as f32;
        let y = p.position.y as f32;
        let z = p.position.z as f32;
        data_bytes.extend_from_slice(&x.to_le_bytes());
        data_bytes.extend_from_slice(&y.to_le_bytes());
        data_bytes.extend_from_slice(&z.to_le_bytes());

        if has_intensity {
            let val = p.intensity.unwrap_or(0.0);
            data_bytes.extend_from_slice(&val.to_le_bytes());
        }
        if has_color {
            let color = p.color.unwrap_or(Color::new(0, 0, 0));
            let r = (color.red >> 8) as u32;
            let g = (color.green >> 8) as u32;
            let b = (color.blue >> 8) as u32;
            let packed = (r << 16) | (g << 8) | b;
            data_bytes.extend_from_slice(&packed.to_le_bytes());
        }
        if has_class {
            let class = p.classification.unwrap_or(0);
            data_bytes.push(class);
            data_bytes.push(0);
            data_bytes.push(0);
            data_bytes.push(0);
        }
    }
    buf.extend_from_slice(&(data_bytes.len() as u32).to_le_bytes());
    buf.extend_from_slice(&data_bytes);

    // 6. is_dense
    buf.push(0x01);

    buf
}

fn deserialize_pointcloud2(data: &[u8]) -> Result<Vec<Point>> {
    if data.len() < 32 {
        return Err(Error::invalid("Serialized PointCloud2 message too short"));
    }

    let mut offset = 0;

    let _seq = u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap());
    let sec = u32::from_le_bytes(data[offset + 4..offset + 8].try_into().unwrap());
    let nsec = u32::from_le_bytes(data[offset + 8..offset + 12].try_into().unwrap());
    let gps_time = sec as f64 + nsec as f64 * 1e-9;
    offset += 12;

    let frame_id_len = u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap()) as usize;
    offset += 4;
    if offset + frame_id_len > data.len() {
        return Err(Error::invalid("Failed to parse frame_id"));
    }
    offset += frame_id_len;

    let height = u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap());
    offset += 4;
    let width = u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap());
    offset += 4;
    let num_points = (height * width) as usize;

    let fields_len = u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap()) as usize;
    offset += 4;

    struct ParsedField {
        name: String,
        offset: usize,
        datatype: u8,
        _count: u32,
    }
    let mut fields = Vec::new();
    for _ in 0..fields_len {
        let name_len = u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap()) as usize;
        offset += 4;
        let name = String::from_utf8_lossy(&data[offset..offset + name_len]).into_owned();
        offset += name_len;

        let f_offset = u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap()) as usize;
        let datatype = data[offset + 4];
        let count = u32::from_le_bytes(data[offset + 5..offset + 9].try_into().unwrap());
        offset += 9;

        fields.push(ParsedField {
            name,
            offset: f_offset,
            datatype,
            _count: count,
        });
    }

    let is_bigendian = data[offset] != 0;
    offset += 1;

    let point_step = u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap()) as usize;
    offset += 4;

    let _row_step = u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap()) as usize;
    offset += 4;

    let data_len = u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap()) as usize;
    offset += 4;

    let point_data = &data[offset..offset + data_len];
    offset += data_len;

    let _is_dense = if offset < data.len() {
        data[offset] != 0
    } else {
        true
    };

    let mut points = Vec::with_capacity(num_points);
    for p_idx in 0..num_points {
        let p_start = p_idx * point_step;
        if p_start + point_step > point_data.len() {
            break;
        }
        let p_bytes = &point_data[p_start..p_start + point_step];

        let mut x = 0.0;
        let mut y = 0.0;
        let mut z = 0.0;
        let mut intensity = None;
        let mut color = None;
        let mut classification = None;

        for f in &fields {
            if f.offset >= point_step {
                continue;
            }
            let val = read_field_val(&p_bytes[f.offset..], f.datatype, is_bigendian)?;
            match f.name.as_str() {
                "x" => x = val,
                "y" => y = val,
                "z" => z = val,
                "intensity" => intensity = Some(val as f32),
                "rgb" | "rgba" => {
                    let packed = val as u32;
                    let r = ((packed >> 16) & 0xff) as u16 * 257;
                    let g = ((packed >> 8) & 0xff) as u16 * 257;
                    let b = (packed & 0xff) as u16 * 257;
                    color = Some(Color::new(r, g, b));
                }
                "classification" => classification = Some(val as u8),
                _ => {}
            }
        }

        let mut point = Point::new(x, y, z);
        point.intensity = intensity;
        point.color = color;
        point.classification = classification;
        point.gps_time = Some(gps_time);
        points.push(point);
    }

    Ok(points)
}

struct CdrSerializer {
    buf: Vec<u8>,
}

impl CdrSerializer {
    fn new() -> Self {
        let mut s = Self { buf: Vec::new() };
        s.buf.extend_from_slice(&[0x00, 0x01, 0x00, 0x00]); // CDR Header
        s
    }

    fn align(&mut self, size: usize) {
        let offset = self.buf.len() - 4;
        let padding = (size - (offset % size)) % size;
        for _ in 0..padding {
            self.buf.push(0x00);
        }
    }

    fn write_u8(&mut self, val: u8) {
        self.buf.push(val);
    }

    fn write_bool(&mut self, val: bool) {
        self.buf.push(if val { 1 } else { 0 });
    }

    fn write_u32(&mut self, val: u32) {
        self.align(4);
        self.buf.extend_from_slice(&val.to_le_bytes());
    }

    fn write_i32(&mut self, val: i32) {
        self.align(4);
        self.buf.extend_from_slice(&val.to_le_bytes());
    }

    fn write_f32(&mut self, val: f32) {
        self.align(4);
        self.buf.extend_from_slice(&val.to_le_bytes());
    }

    fn write_string(&mut self, val: &str) {
        let len_with_null = val.len() as u32 + 1;
        self.write_u32(len_with_null);
        self.buf.extend_from_slice(val.as_bytes());
        self.buf.push(0x00);
    }
}

fn serialize_pointcloud2_cdr(cloud: &PointCloud) -> Vec<u8> {
    let mut s = CdrSerializer::new();

    // 1. Header stamp & frame_id
    let stamp_sec = cloud.points.first().and_then(|p| p.gps_time).unwrap_or(0.0) as i32;
    let stamp_nsec = ((cloud
        .points
        .first()
        .and_then(|p| p.gps_time)
        .unwrap_or(0.0)
        .fract())
        * 1e9) as u32;
    s.write_i32(stamp_sec);
    s.write_u32(stamp_nsec);
    s.write_string("map");

    // 2. height & width
    s.write_u32(1);
    s.write_u32(cloud.points.len() as u32);

    // 3. fields
    let has_intensity = cloud.has_intensity();
    let has_color = cloud.has_color();
    let has_class = cloud.has_classification();

    let mut fields: Vec<(&str, u32, u8, u32)> =
        vec![("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1)];
    let mut point_step = 12;
    if has_intensity {
        fields.push(("intensity", 12, 7, 1));
        point_step += 4;
    }
    if has_color {
        fields.push(("rgb", point_step, 6, 1));
        point_step += 4;
    }
    if has_class {
        fields.push(("classification", point_step, 2, 1));
        point_step += 4;
    }

    s.write_u32(fields.len() as u32);
    for &(name, offset, datatype, count) in &fields {
        s.write_string(name);
        s.write_u32(offset);
        s.write_u8(datatype);
        s.write_u32(count);
    }

    s.write_bool(false); // little endian
    s.write_u32(point_step);
    let row_step = point_step as usize * cloud.points.len();
    s.write_u32(row_step as u32);

    // Data sequence
    s.write_u32(row_step as u32);
    for p in &cloud.points {
        let x = p.position.x as f32;
        let y = p.position.y as f32;
        let z = p.position.z as f32;
        s.write_f32(x);
        s.write_f32(y);
        s.write_f32(z);

        if has_intensity {
            let val = p.intensity.unwrap_or(0.0);
            s.write_f32(val);
        }
        if has_color {
            let color = p.color.unwrap_or(Color::new(0, 0, 0));
            let r = (color.red >> 8) as u32;
            let g = (color.green >> 8) as u32;
            let b = (color.blue >> 8) as u32;
            let packed = (r << 16) | (g << 8) | b;
            s.write_u32(packed);
        }
        if has_class {
            let class = p.classification.unwrap_or(0);
            s.write_u8(class);
            s.write_u8(0);
            s.write_u8(0);
            s.write_u8(0);
        }
    }

    s.write_bool(true); // is_dense

    s.buf
}

struct CdrDeserializer<'a> {
    buf: &'a [u8],
    offset: usize,
}

impl<'a> CdrDeserializer<'a> {
    fn new(buf: &'a [u8]) -> Result<Self> {
        if buf.len() < 4 {
            return Err(Error::invalid("CDR buffer too short"));
        }
        Ok(Self { buf, offset: 4 })
    }

    fn align(&mut self, size: usize) {
        let payload_offset = self.offset - 4;
        let padding = (size - (payload_offset % size)) % size;
        self.offset += padding;
    }

    fn read_u8(&mut self) -> Result<u8> {
        if self.offset >= self.buf.len() {
            return Err(Error::invalid("CDR read EOF"));
        }
        let val = self.buf[self.offset];
        self.offset += 1;
        Ok(val)
    }

    fn read_bool(&mut self) -> Result<bool> {
        Ok(self.read_u8()? != 0)
    }

    fn read_u32(&mut self) -> Result<u32> {
        self.align(4);
        if self.offset + 4 > self.buf.len() {
            return Err(Error::invalid("CDR read EOF"));
        }
        let val = u32::from_le_bytes(self.buf[self.offset..self.offset + 4].try_into().unwrap());
        self.offset += 4;
        Ok(val)
    }

    fn read_i32(&mut self) -> Result<i32> {
        self.align(4);
        if self.offset + 4 > self.buf.len() {
            return Err(Error::invalid("CDR read EOF"));
        }
        let val = i32::from_le_bytes(self.buf[self.offset..self.offset + 4].try_into().unwrap());
        self.offset += 4;
        Ok(val)
    }

    fn read_string(&mut self) -> Result<String> {
        let len_with_null = self.read_u32()? as usize;
        if len_with_null == 0 {
            return Ok(String::new());
        }
        if self.offset + len_with_null > self.buf.len() {
            return Err(Error::invalid("CDR read EOF"));
        }
        let s = String::from_utf8_lossy(&self.buf[self.offset..self.offset + len_with_null - 1])
            .into_owned();
        self.offset += len_with_null;
        Ok(s)
    }
}

fn deserialize_pointcloud2_cdr(buf: &[u8]) -> Result<Vec<Point>> {
    let mut d = CdrDeserializer::new(buf)?;

    // 1. Header stamp & frame_id
    let sec = d.read_i32()?;
    let nsec = d.read_u32()?;
    let gps_time = sec as f64 + nsec as f64 * 1e-9;
    let _frame_id = d.read_string()?;

    // 2. height & width
    let height = d.read_u32()?;
    let width = d.read_u32()?;
    let num_points = (height * width) as usize;

    // 3. fields
    let fields_len = d.read_u32()? as usize;
    struct ParsedField {
        name: String,
        offset: usize,
        datatype: u8,
        _count: u32,
    }
    let mut fields = Vec::new();
    for _ in 0..fields_len {
        let name = d.read_string()?;
        let offset = d.read_u32()? as usize;
        let datatype = d.read_u8()?;
        let count = d.read_u32()?;
        fields.push(ParsedField {
            name,
            offset,
            datatype,
            _count: count,
        });
    }

    // 4. Config
    let is_bigendian = d.read_bool()?;
    let point_step = d.read_u32()? as usize;
    let _row_step = d.read_u32()?;

    // 5. Data payload
    let data_len = d.read_u32()? as usize;
    d.align(1);
    if d.offset + data_len > d.buf.len() {
        return Err(Error::invalid("CDR data payload EOF"));
    }
    let point_data = &d.buf[d.offset..d.offset + data_len];
    d.offset += data_len;

    // 6. is_dense
    let _is_dense = d.read_bool()?;

    let mut points = Vec::with_capacity(num_points);
    for p_idx in 0..num_points {
        let p_start = p_idx * point_step;
        if p_start + point_step > point_data.len() {
            break;
        }
        let p_bytes = &point_data[p_start..p_start + point_step];

        let mut x = 0.0;
        let mut y = 0.0;
        let mut z = 0.0;
        let mut intensity = None;
        let mut color = None;
        let mut classification = None;

        for f in &fields {
            if f.offset >= point_step {
                continue;
            }
            let val = read_field_val(&p_bytes[f.offset..], f.datatype, is_bigendian)?;
            match f.name.as_str() {
                "x" => x = val,
                "y" => y = val,
                "z" => z = val,
                "intensity" => intensity = Some(val as f32),
                "rgb" | "rgba" => {
                    let packed = val as u32;
                    let r = ((packed >> 16) & 0xff) as u16 * 257;
                    let g = ((packed >> 8) & 0xff) as u16 * 257;
                    let b = (packed & 0xff) as u16 * 257;
                    color = Some(Color::new(r, g, b));
                }
                "classification" => classification = Some(val as u8),
                _ => {}
            }
        }

        let mut point = Point::new(x, y, z);
        point.intensity = intensity;
        point.color = color;
        point.classification = classification;
        point.gps_time = Some(gps_time);
        points.push(point);
    }

    Ok(points)
}

fn read_field_val(bytes: &[u8], datatype: u8, is_bigendian: bool) -> Result<f64> {
    if bytes.is_empty() {
        return Ok(0.0);
    }
    match datatype {
        1 => Ok(bytes[0] as i8 as f64),
        2 => Ok(bytes[0] as f64),
        3 => {
            if bytes.len() < 2 {
                return Ok(0.0);
            }
            let v = if is_bigendian {
                i16::from_be_bytes(bytes[0..2].try_into().unwrap())
            } else {
                i16::from_le_bytes(bytes[0..2].try_into().unwrap())
            };
            Ok(v as f64)
        }
        4 => {
            if bytes.len() < 2 {
                return Ok(0.0);
            }
            let v = if is_bigendian {
                u16::from_be_bytes(bytes[0..2].try_into().unwrap())
            } else {
                u16::from_le_bytes(bytes[0..2].try_into().unwrap())
            };
            Ok(v as f64)
        }
        5 => {
            if bytes.len() < 4 {
                return Ok(0.0);
            }
            let v = if is_bigendian {
                i32::from_be_bytes(bytes[0..4].try_into().unwrap())
            } else {
                i32::from_le_bytes(bytes[0..4].try_into().unwrap())
            };
            Ok(v as f64)
        }
        6 => {
            if bytes.len() < 4 {
                return Ok(0.0);
            }
            let v = if is_bigendian {
                u32::from_be_bytes(bytes[0..4].try_into().unwrap())
            } else {
                u32::from_le_bytes(bytes[0..4].try_into().unwrap())
            };
            Ok(v as f64)
        }
        7 => {
            if bytes.len() < 4 {
                return Ok(0.0);
            }
            let v = if is_bigendian {
                f32::from_be_bytes(bytes[0..4].try_into().unwrap())
            } else {
                f32::from_le_bytes(bytes[0..4].try_into().unwrap())
            };
            Ok(v as f64)
        }
        8 => {
            if bytes.len() < 8 {
                return Ok(0.0);
            }
            let v = if is_bigendian {
                f64::from_be_bytes(bytes[0..8].try_into().unwrap())
            } else {
                f64::from_le_bytes(bytes[0..8].try_into().unwrap())
            };
            Ok(v)
        }
        _ => Ok(0.0),
    }
}
