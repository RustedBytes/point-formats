use crate::error::{Error, Result};
use crate::types::{Color, Geometry, Point, PointCloud, Vec3};
use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::Path;

/// Reads a point cloud from a VendorRaw binary file.
pub fn read_vendorraw(path: impl AsRef<Path>) -> Result<Geometry> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);

    let mut magic = [0_u8; 10];
    reader.read_exact(&mut magic)?;
    if &magic != b"VENDORRAW\0" {
        return Err(Error::invalid("Invalid VendorRaw file magic"));
    }

    let mut count_bytes = [0_u8; 8];
    reader.read_exact(&mut count_bytes)?;
    let count = u64::from_le_bytes(count_bytes) as usize;

    let mut flags_byte = [0_u8; 1];
    reader.read_exact(&mut flags_byte)?;
    let flags = flags_byte[0];

    let has_intensity = (flags & 1) != 0;
    let has_color = (flags & 2) != 0;
    let has_class = (flags & 4) != 0;
    let has_gps = (flags & 8) != 0;
    let has_normal = (flags & 16) != 0;

    let mut points = Vec::with_capacity(count);

    for _ in 0..count {
        let mut x_bytes = [0_u8; 8];
        let mut y_bytes = [0_u8; 8];
        let mut z_bytes = [0_u8; 8];
        reader.read_exact(&mut x_bytes)?;
        reader.read_exact(&mut y_bytes)?;
        reader.read_exact(&mut z_bytes)?;

        let x = f64::from_le_bytes(x_bytes);
        let y = f64::from_le_bytes(y_bytes);
        let z = f64::from_le_bytes(z_bytes);

        let mut p = Point::new(x, y, z);

        if has_intensity {
            let mut val_bytes = [0_u8; 4];
            reader.read_exact(&mut val_bytes)?;
            p.intensity = Some(f32::from_le_bytes(val_bytes));
        }
        if has_color {
            let mut rgb = [0_u8; 3];
            reader.read_exact(&mut rgb)?;
            let r = rgb[0] as u16 * 257;
            let g = rgb[1] as u16 * 257;
            let b = rgb[2] as u16 * 257;
            p.color = Some(Color::new(r, g, b));
        }
        if has_class {
            let mut class_byte = [0_u8; 1];
            reader.read_exact(&mut class_byte)?;
            p.classification = Some(class_byte[0]);
        }
        if has_gps {
            let mut gps_bytes = [0_u8; 8];
            reader.read_exact(&mut gps_bytes)?;
            p.gps_time = Some(f64::from_le_bytes(gps_bytes));
        }
        if has_normal {
            let mut nx_bytes = [0_u8; 4];
            let mut ny_bytes = [0_u8; 4];
            let mut nz_bytes = [0_u8; 4];
            reader.read_exact(&mut nx_bytes)?;
            reader.read_exact(&mut ny_bytes)?;
            reader.read_exact(&mut nz_bytes)?;
            let nx = f32::from_le_bytes(nx_bytes) as f64;
            let ny = f32::from_le_bytes(ny_bytes) as f64;
            let nz = f32::from_le_bytes(nz_bytes) as f64;
            p.normal = Some(Vec3::new(nx, ny, nz));
        }

        points.push(p);
    }

    Ok(Geometry::PointCloud(PointCloud::new(points)))
}

/// Writes a point cloud to a VendorRaw binary file.
pub fn write_vendorraw(path: impl AsRef<Path>, cloud: &PointCloud) -> Result<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    writer.write_all(b"VENDORRAW\0")?;
    writer.write_all(&(cloud.points.len() as u64).to_le_bytes())?;

    let has_intensity = cloud.has_intensity();
    let has_color = cloud.has_color();
    let has_class = cloud.has_classification();
    let has_gps = cloud.points.iter().any(|p| p.gps_time.is_some());
    let has_normal = cloud.points.iter().any(|p| p.normal.is_some());

    let mut flags = 0_u8;
    if has_intensity { flags |= 1; }
    if has_color { flags |= 2; }
    if has_class { flags |= 4; }
    if has_gps { flags |= 8; }
    if has_normal { flags |= 16; }

    writer.write_all(&[flags])?;

    for p in &cloud.points {
        writer.write_all(&p.position.x.to_le_bytes())?;
        writer.write_all(&p.position.y.to_le_bytes())?;
        writer.write_all(&p.position.z.to_le_bytes())?;

        if has_intensity {
            writer.write_all(&p.intensity.unwrap_or(0.0).to_le_bytes())?;
        }
        if has_color {
            let color = p.color.unwrap_or(Color::new(0, 0, 0));
            let r = (color.red >> 8) as u8;
            let g = (color.green >> 8) as u8;
            let b = (color.blue >> 8) as u8;
            writer.write_all(&[r, g, b])?;
        }
        if has_class {
            writer.write_all(&[p.classification.unwrap_or(0)])?;
        }
        if has_gps {
            writer.write_all(&p.gps_time.unwrap_or(0.0).to_le_bytes())?;
        }
        if has_normal {
            let n = p.normal.unwrap_or(Vec3::new(0.0, 0.0, 1.0));
            writer.write_all(&(n.x as f32).to_le_bytes())?;
            writer.write_all(&(n.y as f32).to_le_bytes())?;
            writer.write_all(&(n.z as f32).to_le_bytes())?;
        }
    }

    Ok(())
}

fn serialize_packet_payload(points: &[Point]) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.write_all(b"SNSR").unwrap();
    payload.write_all(&(points.len() as u32).to_le_bytes()).unwrap();

    for p in points {
        payload.write_all(&(p.position.x as f32).to_le_bytes()).unwrap();
        payload.write_all(&(p.position.y as f32).to_le_bytes()).unwrap();
        payload.write_all(&(p.position.z as f32).to_le_bytes()).unwrap();
        payload.write_all(&p.intensity.unwrap_or(0.0).to_le_bytes()).unwrap();

        let color = p.color.unwrap_or(Color::new(0, 0, 0));
        let r = (color.red >> 8) as u8;
        let g = (color.green >> 8) as u8;
        let b = (color.blue >> 8) as u8;
        payload.write_all(&[r, g, b]).unwrap();

        payload.write_all(&[p.classification.unwrap_or(0)]).unwrap();
        payload.write_all(&p.gps_time.unwrap_or(0.0).to_le_bytes()).unwrap();
    }
    payload
}

fn deserialize_packet_payload(payload: &[u8], points: &mut Vec<Point>) -> Result<()> {
    if payload.len() < 8 {
        return Err(Error::invalid("UDP packet too short"));
    }
    if &payload[0..4] != b"SNSR" {
        return Err(Error::invalid("Invalid UDP packet magic"));
    }
    let count = u32::from_le_bytes(payload[4..8].try_into().unwrap()) as usize;
    let expected_len = 8 + count * 28;
    if payload.len() < expected_len {
        return Err(Error::invalid("UDP packet payload too short for point count"));
    }

    let mut offset = 8;
    for _ in 0..count {
        let x = f32::from_le_bytes(payload[offset..offset + 4].try_into().unwrap()) as f64;
        let y = f32::from_le_bytes(payload[offset + 4..offset + 8].try_into().unwrap()) as f64;
        let z = f32::from_le_bytes(payload[offset + 8..offset + 12].try_into().unwrap()) as f64;
        let intensity = f32::from_le_bytes(payload[offset + 12..offset + 16].try_into().unwrap());
        let r = payload[offset + 16];
        let g = payload[offset + 17];
        let b = payload[offset + 18];
        let class = payload[offset + 19];
        let gps_time = f64::from_le_bytes(payload[offset + 20..offset + 28].try_into().unwrap());
        offset += 28;

        let mut p = Point::new(x, y, z);
        p.intensity = Some(intensity);
        p.color = Some(Color::new(r as u16 * 257, g as u16 * 257, b as u16 * 257));
        p.classification = Some(class);
        p.gps_time = Some(gps_time);
        points.push(p);
    }
    Ok(())
}

/// Reads a point cloud from a length-prefixed UDP packet file.
pub fn read_udppackets(path: impl AsRef<Path>) -> Result<Geometry> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);

    let mut points = Vec::new();
    loop {
        let mut len_bytes = [0_u8; 4];
        if reader.read_exact(&mut len_bytes).is_err() {
            break; // EOF
        }
        let len = u32::from_le_bytes(len_bytes) as usize;
        let mut payload = vec![0_u8; len];
        reader.read_exact(&mut payload)?;

        deserialize_packet_payload(&payload, &mut points)?;
    }

    Ok(Geometry::PointCloud(PointCloud::new(points)))
}

/// Writes a point cloud to a length-prefixed UDP packet file.
pub fn write_udppackets(path: impl AsRef<Path>, cloud: &PointCloud) -> Result<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    for chunk in cloud.points.chunks(48) {
        let payload = serialize_packet_payload(chunk);
        writer.write_all(&(payload.len() as u32).to_le_bytes())?;
        writer.write_all(&payload)?;
    }

    Ok(())
}

fn write_pcap_global_header<W: Write>(writer: &mut W) -> std::io::Result<()> {
    writer.write_all(&0xa1b2c3d4_u32.to_le_bytes())?; // magic
    writer.write_all(&2_u16.to_le_bytes())?;         // major
    writer.write_all(&4_u16.to_le_bytes())?;         // minor
    writer.write_all(&0_i32.to_le_bytes())?;         // timezone
    writer.write_all(&0_u32.to_le_bytes())?;         // sigfigs
    writer.write_all(&65535_u32.to_le_bytes())?;     // snaplen
    writer.write_all(&1_u32.to_le_bytes())?;         // network (Ethernet)
    Ok(())
}

fn write_pcap_packet<W: Write>(
    writer: &mut W,
    payload: &[u8],
    ts_sec: u32,
    ts_usec: u32,
) -> std::io::Result<()> {
    let packet_data_len = 42 + payload.len();

    // 1. Write PCAP Packet Header
    writer.write_all(&ts_sec.to_le_bytes())?;
    writer.write_all(&ts_usec.to_le_bytes())?;
    writer.write_all(&(packet_data_len as u32).to_le_bytes())?; // incl_len
    writer.write_all(&(packet_data_len as u32).to_le_bytes())?; // orig_len

    // 2. Ethernet Header
    writer.write_all(&[0xff; 6])?; // Dest MAC
    writer.write_all(&[0x00; 6])?; // Src MAC
    writer.write_all(&[0x08, 0x00])?; // EtherType IPv4

    // 3. IPv4 Header
    writer.write_all(&[0x45, 0x00])?;
    let ip_len = (20 + 8 + payload.len()) as u16;
    writer.write_all(&ip_len.to_be_bytes())?;
    writer.write_all(&[0x00, 0x00, 0x40, 0x00, 64, 17, 0x00, 0x00])?;
    writer.write_all(&[192, 168, 1, 100])?;
    writer.write_all(&[192, 168, 1, 255])?;

    // 4. UDP Header
    writer.write_all(&2368_u16.to_be_bytes())?; // Src port
    writer.write_all(&2368_u16.to_be_bytes())?; // Dest port
    let udp_len = (8 + payload.len()) as u16;
    writer.write_all(&udp_len.to_be_bytes())?;
    writer.write_all(&[0x00, 0x00])?;

    // 5. Payload
    writer.write_all(payload)?;

    Ok(())
}

/// Reads a point cloud from a PCAP packet capture file.
pub fn read_pcap(path: impl AsRef<Path>) -> Result<Geometry> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);

    let mut global_header = [0_u8; 24];
    reader.read_exact(&mut global_header)?;

    let magic = u32::from_le_bytes(global_header[0..4].try_into().unwrap());
    let swap_endian = match magic {
        0xa1b2c3d4 => false,
        0xd4c3b2a1 => true,
        _ => return Err(Error::invalid("Invalid PCAP magic number")),
    };

    let read_u32_pcap = |bytes: [u8; 4]| -> u32 {
        if swap_endian {
            u32::from_be_bytes(bytes)
        } else {
            u32::from_le_bytes(bytes)
        }
    };

    let network = read_u32_pcap(global_header[20..24].try_into().unwrap());
    if network != 1 && network != 101 {
        return Err(Error::invalid(format!("Unsupported PCAP LinkType: {}", network)));
    }

    let mut points = Vec::new();

    loop {
        let mut pkt_header = [0_u8; 16];
        if reader.read_exact(&mut pkt_header).is_err() {
            break; // EOF
        }

        let incl_len = read_u32_pcap(pkt_header[8..12].try_into().unwrap()) as usize;
        let _orig_len = read_u32_pcap(pkt_header[12..16].try_into().unwrap()) as usize;

        let mut pkt_data = vec![0_u8; incl_len];
        reader.read_exact(&mut pkt_data)?;

        let payload = match network {
            1 => {
                if pkt_data.len() < 14 {
                    continue;
                }
                let mut ip_header_start = 14;
                let mut ether_type = u16::from_be_bytes(pkt_data[12..14].try_into().unwrap());
                if ether_type == 0x8100 {
                    if pkt_data.len() < 18 {
                        continue;
                    }
                    ether_type = u16::from_be_bytes(pkt_data[16..18].try_into().unwrap());
                    ip_header_start = 18;
                }
                if ether_type != 0x0800 {
                    continue;
                }
                if pkt_data.len() < ip_header_start + 20 {
                    continue;
                }
                let protocol = pkt_data[ip_header_start + 9];
                if protocol != 17 {
                    continue;
                }
                let ip_header_len = (pkt_data[ip_header_start] & 0x0f) as usize * 4;
                let udp_header_start = ip_header_start + ip_header_len;
                if pkt_data.len() < udp_header_start + 8 {
                    continue;
                }
                let udp_len = u16::from_be_bytes(pkt_data[udp_header_start + 4..udp_header_start + 6].try_into().unwrap()) as usize;
                let payload_start = udp_header_start + 8;
                if pkt_data.len() < payload_start {
                    continue;
                }
                let payload_len = (udp_len.saturating_sub(8)).min(pkt_data.len() - payload_start);
                &pkt_data[payload_start..payload_start + payload_len]
            }
            101 => {
                if pkt_data.is_empty() {
                    continue;
                }
                let protocol = pkt_data[9];
                if protocol != 17 {
                    continue;
                }
                let ip_header_len = (pkt_data[0] & 0x0f) as usize * 4;
                let udp_header_start = ip_header_len;
                if pkt_data.len() < udp_header_start + 8 {
                    continue;
                }
                let udp_len = u16::from_be_bytes(pkt_data[udp_header_start + 4..udp_header_start + 6].try_into().unwrap()) as usize;
                let payload_start = udp_header_start + 8;
                if pkt_data.len() < payload_start {
                    continue;
                }
                let payload_len = (udp_len.saturating_sub(8)).min(pkt_data.len() - payload_start);
                &pkt_data[payload_start..payload_start + payload_len]
            }
            _ => continue,
        };

        if payload.len() >= 8 && &payload[0..4] == b"SNSR" {
            let _ = deserialize_packet_payload(payload, &mut points);
        }
    }

    Ok(Geometry::PointCloud(PointCloud::new(points)))
}

/// Writes a point cloud to a PCAP packet capture file.
pub fn write_pcap(path: impl AsRef<Path>, cloud: &PointCloud) -> Result<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    write_pcap_global_header(&mut writer)?;

    for chunk in cloud.points.chunks(48) {
        let payload = serialize_packet_payload(chunk);
        let time_val = chunk.first().and_then(|p| p.gps_time).unwrap_or(0.0);
        let ts_sec = time_val as u32;
        let ts_usec_val = ((time_val.fract() * 1e6) as u32).min(999999);

        write_pcap_packet(&mut writer, &payload, ts_sec, ts_usec_val)?;
    }

    Ok(())
}
