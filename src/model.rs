use anyhow::Error;
use macroquad::{
    camera::{set_camera, Camera, Camera3D},
    file::load_file,
    math::{Mat4, Vec2, Vec3, Vec4},
    models::{Mesh, Vertex},
    texture::RenderPass,
};
use std::{
    collections::{hash_map::Entry, HashMap},
    io::BufReader,
};
use tobj::{load_obj_buf, LoadOptions};

pub async fn load_model(path: &str) -> Result<Mesh, Error> {
    let data = load_file(path).await?;
    let mesh = load_obj_buf(
        &mut BufReader::new(&data[..]),
        &LoadOptions {
            ignore_lines: true,
            triangulate: true,
            ..Default::default()
        },
        |_| unimplemented!(),
    )?
    .0
    .remove(0)
    .mesh;

    let mut vertex_map = HashMap::<(u32, u32, u32), u16>::new();
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    assert!(mesh.face_arities.iter().all(|&n| n == 3));
    for ((v, t), n) in (mesh.indices.into_iter())
        .zip(mesh.texcoord_indices)
        .zip(mesh.normal_indices)
    {
        match vertex_map.entry((v, t, n)) {
            Entry::Occupied(e) => indices.push(*e.get()),
            Entry::Vacant(e) => {
                let i = vertices.len() as u16;
                indices.push(i);
                e.insert(i);
                let (v, t, n) = (3 * v as usize, 2 * t as usize, 3 * n as usize);
                vertices.push(Vertex {
                    position: Vec3::from_slice(&mesh.positions[v..(v + 3)]),
                    uv: Vec2::from_slice(&mesh.texcoords[t..(t + 2)]),
                    normal: Vec4::from((Vec3::from_slice(&mesh.normals[n..(n + 3)]), 1.0)),
                    color: [255; 4],
                });
            }
        }
    }
    Ok(Mesh {
        vertices,
        indices,
        texture: None,
    })
}

pub trait TransformStack: Camera + Sized {
    fn push<T: Into<Mat4>>(&mut self, transform: T) -> Transformed<'_, Self> {
        let this = Transformed {
            base: self,
            transform: transform.into(),
        };
        set_camera(&this);
        this
    }
}

impl TransformStack for Camera3D {}

pub struct Transformed<'a, T: TransformStack> {
    base: &'a mut T,
    transform: Mat4,
}

impl<'a, T: TransformStack> Camera for Transformed<'a, T> {
    fn matrix(&self) -> Mat4 {
        self.base.matrix().mul_mat4(&self.transform)
    }
    fn depth_enabled(&self) -> bool {
        self.base.depth_enabled()
    }
    fn render_pass(&self) -> Option<RenderPass> {
        self.base.render_pass()
    }
    fn viewport(&self) -> Option<(i32, i32, i32, i32)> {
        self.base.viewport()
    }
}

impl<'a, T: TransformStack> TransformStack for Transformed<'a, T> {}

impl<'a, T: TransformStack> Drop for Transformed<'a, T> {
    fn drop(&mut self) {
        set_camera(self.base);
    }
}
