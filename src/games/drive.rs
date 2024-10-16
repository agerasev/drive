use crate::model::load_model;
use anyhow::Error;
use macroquad::{
    camera::{set_camera, set_default_camera, Camera2D},
    color,
    input::{is_key_down, KeyCode},
    math::{Rect, Vec2},
    models::{draw_mesh, Mesh},
    texture::{load_texture, set_default_filter_mode, FilterMode},
    window::{clear_background, next_frame},
};
use std::{future::Future, pin::Pin};

pub async fn main() -> Result<(), Error> {
    set_default_filter_mode(FilterMode::Linear);
    let model = Mesh {
        texture: Some(load_texture("l200.png").await?),
        ..load_model("l200.obj").await?
    };

    while !is_key_down(KeyCode::Escape) {
        {
            let camera = Camera2D {
                zoom: Vec2::splat(0.2),
                ..Default::default()
            };
            set_camera(&camera);

            clear_background(color::BLACK);

            draw_mesh(&model);

            set_default_camera();
        }

        next_frame().await
    }

    Ok(())
}

pub struct Game {}

impl Game {
    pub async fn new() -> Result<Self, Error> {
        Ok(Self {})
    }
}

impl crate::Game for Game {
    fn name(&self) -> String {
        "Машина".to_owned()
    }

    fn draw_preview(&self, _rect: Rect) {}

    fn launch(&self) -> Pin<Box<dyn Future<Output = Result<(), Error>>>> {
        Box::pin(main())
    }
}
