use crate::error::{Error, Result};
use crate::types::{Geometry, Point, PointCloud};
use std::io::{BufRead, BufReader, Read, Write};

/// Reads a point cloud from an Esri ASCII Grid stream.
pub fn read<R: Read>(reader: R) -> Result<Geometry> {
    let mut reader = BufReader::new(reader);
    let mut line = String::new();

    let mut ncols = 0;
    let mut nrows = 0;
    let mut xllcorner = None;
    let mut yllcorner = None;
    let mut xllcenter = None;
    let mut yllcenter = None;
    let mut cellsize = 0.0;
    let mut nodata_value = None;

    // Read header lines (usually 5 or 6 lines)
    loop {
        line.clear();
        let bytes_read = reader.read_line(&mut line)?;
        if bytes_read == 0 {
            break;
        }

        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }

        let parts: Vec<&str> = trimmed.split_whitespace().collect();
        if parts.len() != 2 {
            // End of header, start of data
            break;
        }

        let key = parts[0].to_lowercase();
        let val = parts[1];

        match key.as_str() {
            "ncols" => {
                ncols = val
                    .parse::<usize>()
                    .map_err(|e| Error::invalid(format!("Invalid ncols: {}", e)))?
            }
            "nrows" => {
                nrows = val
                    .parse::<usize>()
                    .map_err(|e| Error::invalid(format!("Invalid nrows: {}", e)))?
            }
            "xllcorner" => {
                xllcorner = Some(
                    val.parse::<f64>()
                        .map_err(|e| Error::invalid(format!("Invalid xllcorner: {}", e)))?,
                )
            }
            "yllcorner" => {
                yllcorner = Some(
                    val.parse::<f64>()
                        .map_err(|e| Error::invalid(format!("Invalid yllcorner: {}", e)))?,
                )
            }
            "xllcenter" => {
                xllcenter = Some(
                    val.parse::<f64>()
                        .map_err(|e| Error::invalid(format!("Invalid xllcenter: {}", e)))?,
                )
            }
            "yllcenter" => {
                yllcenter = Some(
                    val.parse::<f64>()
                        .map_err(|e| Error::invalid(format!("Invalid yllcenter: {}", e)))?,
                )
            }
            "cellsize" => {
                cellsize = val
                    .parse::<f64>()
                    .map_err(|e| Error::invalid(format!("Invalid cellsize: {}", e)))?
            }
            "nodata_value" => {
                nodata_value = Some(
                    val.parse::<f64>()
                        .map_err(|e| Error::invalid(format!("Invalid nodata_value: {}", e)))?,
                )
            }
            _ => {
                break;
            }
        }
    }

    if ncols == 0 || nrows == 0 || cellsize <= 0.0 {
        return Err(Error::invalid(
            "Esri ASCII Grid header missing ncols, nrows, or cellsize",
        ));
    }

    let min_x = match (xllcorner, xllcenter) {
        (Some(c), _) => c,
        (None, Some(c)) => c - cellsize / 2.0,
        (None, None) => {
            return Err(Error::invalid(
                "Esri ASCII Grid header missing xllcorner/xllcenter",
            ))
        }
    };

    let min_y = match (yllcorner, yllcenter) {
        (Some(c), _) => c,
        (None, Some(c)) => c - cellsize / 2.0,
        (None, None) => {
            return Err(Error::invalid(
                "Esri ASCII Grid header missing yllcorner/yllcenter",
            ))
        }
    };

    let nodata = nodata_value.unwrap_or(-9999.0);

    // Read remaining body values
    let mut points = Vec::new();
    let mut row = 0;
    let mut col = 0;

    let mut body = String::new();
    let trimmed = line.trim();
    if !trimmed.is_empty() {
        let parts: Vec<&str> = trimmed.split_whitespace().collect();
        let key = parts.first().unwrap_or(&"").to_lowercase();
        if ![
            "ncols",
            "nrows",
            "xllcorner",
            "yllcorner",
            "xllcenter",
            "yllcenter",
            "cellsize",
            "nodata_value",
        ]
        .contains(&key.as_str())
        {
            body.push_str(trimmed);
            body.push(' ');
        }
    }

    reader.read_to_string(&mut body)?;

    for token in body.split_whitespace() {
        let z = token
            .parse::<f64>()
            .map_err(|e| Error::invalid(format!("Failed to parse float: {}", e)))?;
        if (z - nodata).abs() > 1e-9 {
            let x = min_x + col as f64 * cellsize;
            let y = min_y + (nrows - 1 - row) as f64 * cellsize;
            points.push(Point::new(x, y, z));
        }

        col += 1;
        if col >= ncols {
            col = 0;
            row += 1;
            if row >= nrows {
                break;
            }
        }
    }

    Ok(Geometry::PointCloud(PointCloud::new(points)))
}

/// Writes a point cloud to an Esri ASCII Grid stream.
pub fn write<W: Write>(mut writer: W, cloud: &PointCloud) -> Result<()> {
    if cloud.points.is_empty() {
        return Err(Error::invalid(
            "Cannot write empty point cloud to Esri ASCII Grid",
        ));
    }

    let mut min_x = f64::MAX;
    let mut max_x = f64::MIN;
    let mut min_y = f64::MAX;
    let mut max_y = f64::MIN;
    for p in &cloud.points {
        let x = p.position.x;
        let y = p.position.y;
        if x < min_x {
            min_x = x;
        }
        if x > max_x {
            max_x = x;
        }
        if y < min_y {
            min_y = y;
        }
        if y > max_y {
            max_y = y;
        }
    }

    if min_x >= max_x {
        max_x = min_x + 1.0;
    }
    if min_y >= max_y {
        max_y = min_y + 1.0;
    }

    let grid_size = (cloud.points.len() as f64).sqrt().round() as usize;
    let grid_size = grid_size.clamp(16, 512);

    let width = grid_size;
    let height = grid_size;

    let cellsize_x = (max_x - min_x) / width as f64;
    let cellsize_y = (max_y - min_y) / height as f64;
    let cellsize = cellsize_x.max(cellsize_y);

    // Write ASCII header
    writeln!(writer, "ncols         {}", width)?;
    writeln!(writer, "nrows         {}", height)?;
    writeln!(writer, "xllcorner     {}", min_x)?;
    writeln!(writer, "yllcorner     {}", min_y)?;
    writeln!(writer, "cellsize      {}", cellsize)?;
    writeln!(writer, "NODATA_value  -9999.0")?;

    let mut sum_z = vec![0.0; width * height];
    let mut count = vec![0; width * height];

    for p in &cloud.points {
        let px = p.position.x;
        let py = p.position.y;

        let c = ((px - min_x) / cellsize).floor() as usize;
        let r = ((max_y - py) / cellsize).floor() as usize;

        let c = c.min(width - 1);
        let r = r.min(height - 1);

        let idx = r * width + c;
        sum_z[idx] += p.position.z;
        count[idx] += 1;
    }

    for r in 0..height {
        let mut row_vals = Vec::with_capacity(width);
        for c in 0..width {
            let idx = r * width + c;
            if count[idx] > 0 {
                row_vals.push(format!("{:.6}", sum_z[idx] / count[idx] as f64));
            } else {
                row_vals.push("-9999.000000".to_string());
            }
        }
        writeln!(writer, "{}", row_vals.join(" "))?;
    }

    Ok(())
}
