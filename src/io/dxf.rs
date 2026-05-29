use crate::error::{Error, Result};
use crate::types::{Face, Geometry, Mesh, Point, PointCloud, Vec3, Vertex};
use dxf::entities::{EntityType, ModelPoint, Face3D};
use dxf::Drawing;
use std::io::{Read, Write};

/// Reads a point cloud or mesh from a DXF stream.
pub fn read<R: Read>(reader: R) -> Result<Geometry> {
    let mut buf_reader = std::io::BufReader::new(reader);
    let drawing = Drawing::load(&mut buf_reader)
        .map_err(|e| Error::invalid(format!("Failed to parse DXF: {}", e)))?;

    let mut points = Vec::new();
    let mut vertices = Vec::new();
    let mut faces = Vec::new();

    for entity in drawing.entities() {
        match &entity.specific {
            EntityType::ModelPoint(pt) => {
                points.push(Point::new(pt.location.x, pt.location.y, pt.location.z));
            }
            EntityType::Face3D(face) => {
                let c1 = Vec3::new(face.first_corner.x, face.first_corner.y, face.first_corner.z);
                let c2 = Vec3::new(face.second_corner.x, face.second_corner.y, face.second_corner.z);
                let c3 = Vec3::new(face.third_corner.x, face.third_corner.y, face.third_corner.z);
                let c4 = Vec3::new(face.fourth_corner.x, face.fourth_corner.y, face.fourth_corner.z);

                let idx1 = add_vertex(&mut vertices, c1);
                let idx2 = add_vertex(&mut vertices, c2);
                let idx3 = add_vertex(&mut vertices, c3);

                if c3 == c4 {
                    // Triangle
                    faces.push(Face::new(idx1, idx2, idx3));
                } else {
                    // Quad - triangulate
                    let idx4 = add_vertex(&mut vertices, c4);
                    faces.push(Face::new(idx1, idx2, idx3));
                    faces.push(Face::new(idx1, idx3, idx4));
                }
            }
            _ => {}
        }
    }

    if !faces.is_empty() {
        Ok(Geometry::Mesh(Mesh::new(vertices, faces)))
    } else {
        Ok(Geometry::PointCloud(PointCloud::new(points)))
    }
}

/// Writes a point cloud or mesh to a DXF stream.
pub fn write<W: Write>(writer: W, geometry: &Geometry) -> Result<()> {
    let mut drawing = Drawing::new();

    match geometry {
        Geometry::PointCloud(cloud) => {
            for p in &cloud.points {
                let location = dxf::Point::new(p.position.x, p.position.y, p.position.z);
                let pt = ModelPoint::new(location);
                let mut ent = dxf::entities::Entity::new(EntityType::ModelPoint(pt));
                ent.common.layer = "0".to_string();
                drawing.add_entity(ent);
            }
        }
        Geometry::Mesh(mesh) => {
            for face in &mesh.faces {
                let v1 = mesh.vertices[face.indices[0]].position;
                let v2 = mesh.vertices[face.indices[1]].position;
                let v3 = mesh.vertices[face.indices[2]].position;

                let c1 = dxf::Point::new(v1.x, v1.y, v1.z);
                let c2 = dxf::Point::new(v2.x, v2.y, v2.z);
                let c3 = dxf::Point::new(v3.x, v3.y, v3.z);
                let c4 = c3.clone();

                let face3d = Face3D::new(c1, c2, c3, c4);
                let mut ent = dxf::entities::Entity::new(EntityType::Face3D(face3d));
                ent.common.layer = "0".to_string();
                drawing.add_entity(ent);
            }
        }
    }

    let mut buf_writer = std::io::BufWriter::new(writer);
    drawing.save(&mut buf_writer)
        .map_err(|e| Error::invalid(format!("Failed to write DXF: {}", e)))?;
    Ok(())
}

fn add_vertex(vertices: &mut Vec<Vertex>, pos: Vec3) -> usize {
    if let Some(pos_idx) = vertices.iter().position(|v| v.position == pos) {
        pos_idx
    } else {
        let idx = vertices.len();
        vertices.push(Vertex::new(pos));
        idx
    }
}
