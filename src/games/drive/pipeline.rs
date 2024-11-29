use anyhow::Result;
use macroquad::{
    material::{load_material, MaterialParams},
    prelude::{
        gl_use_default_material, gl_use_material, Comparison, Material, PipelineParams,
        ShaderSource,
    },
    texture::Texture2D,
};

pub struct Pipeline {
    material: Material,
}

pub struct PipelineGuard<'a> {
    _owner: &'a Pipeline,
}

impl Pipeline {
    pub fn new() -> Result<Self> {
        Ok(Self {
            material: load_material(
                ShaderSource::Glsl {
                    vertex: VERTEX_SHADER,
                    fragment: FRAGMENT_SHADER,
                },
                MaterialParams {
                    pipeline_params: PipelineParams {
                        depth_test: Comparison::LessOrEqual,
                        depth_write: true,
                        ..Default::default()
                    },
                    textures: vec!["Color".to_string(), "Normal".to_string()],
                    ..Default::default()
                },
            )?,
        })
    }

    pub fn activate(&self, color: &Texture2D, normal: &Texture2D) -> PipelineGuard {
        self.material.set_texture("Color", color.clone());
        self.material.set_texture("Normal", normal.clone());
        gl_use_material(&self.material);
        PipelineGuard { _owner: self }
    }
}

impl<'a> Drop for PipelineGuard<'a> {
    fn drop(&mut self) {
        gl_use_default_material();
    }
}

const VERTEX_SHADER: &'static str = r#"#version 100
attribute vec3 position;
attribute vec2 texcoord;
attribute vec4 normal;

varying lowp vec2 uv;
varying lowp vec3 norm;

uniform mat4 Model;
uniform mat4 Projection;

void main() {
    uv = texcoord;
    gl_Position = Projection * Model * vec4(position, 1);
}
"#;

const FRAGMENT_SHADER: &'static str = r#"#version 100
precision lowp float;

varying vec2 uv;
varying vec3 norm;

uniform sampler2D Color;
uniform sampler2D Normal;

void main() {
    gl_FragColor = texture2D(Color, uv);
}
"#;
