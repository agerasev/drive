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

pub enum TransformStack<'a> {
    Camera(&'a Camera3D),
    Transform(&'a Self, Mat4),
}

impl<'a> Camera for TransformStack<'a> {
    fn matrix(&self) -> Mat4 {
        match self {
            Self::Camera(camera) => camera.matrix(),
            Self::Transform(base, transform) => base.matrix().mul_mat4(transform),
        }
    }
    fn depth_enabled(&self) -> bool {
        match self {
            Self::Camera(camera) => camera.depth_enabled(),
            Self::Transform(base, _) => base.depth_enabled(),
        }
    }
    fn render_pass(&self) -> Option<RenderPass> {
        match self {
            Self::Camera(camera) => camera.render_pass(),
            Self::Transform(base, _) => base.render_pass(),
        }
    }
    fn viewport(&self) -> Option<(i32, i32, i32, i32)> {
        match self {
            Self::Camera(camera) => camera.viewport(),
            Self::Transform(base, _) => base.viewport(),
        }
    }
}

impl<'a> TransformStack<'a> {
    pub fn new(camera: &'a Camera3D) -> Self {
        let this = Self::Camera(camera);
        set_camera(&this);
        this
    }
    pub fn push<T: Into<Mat4>>(&self, transform: T) -> TransformStack {
        let this = TransformStack::Transform(self, transform.into());
        set_camera(&this);
        this
    }
}

impl<'a> Drop for TransformStack<'a> {
    fn drop(&mut self) {
        if let Self::Transform(base, _) = self {
            set_camera(*base);
        }
    }
}
