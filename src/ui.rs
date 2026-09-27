//! Controls update appearance only; simulation runs once per content frame.
use crate::appearance::{Appearance, DEFAULT_PAINT, VEHICLE_NAMES};
use std::cell::Cell;
use wgame_egui::{Canvas, egui};

pub fn layout(ui: &mut egui::Ui, canvas: &Canvas, state: &Cell<Appearance>) -> egui::Response {
    let mut appearance = state.get();
    egui::Panel::top("vehicle-controls").show(ui, |ui| {
        ui.horizontal_wrapped(|ui| {
            ui.strong("Drive");
            ui.separator();
            egui::ComboBox::from_id_salt("vehicle")
                .selected_text(VEHICLE_NAMES[appearance.model])
                .show_ui(ui, |ui| {
                    for (model, name) in VEHICLE_NAMES.into_iter().enumerate() {
                        ui.selectable_value(&mut appearance.model, model, name);
                    }
                });
            ui.label("Paint");
            ui.color_edit_button_srgb(&mut appearance.colors[appearance.model]);
            if ui.small_button("Default color").clicked() {
                appearance.colors[appearance.model] = DEFAULT_PAINT[appearance.model];
            }
            ui.separator();
            ui.label("WASD: drive · Space: brake · Drag: orbit · Tab: capture/release");
        });
    });
    state.set(appearance);
    egui::CentralPanel::default()
        .frame(egui::Frame::NONE)
        .show(ui, |ui| canvas_keys(canvas.show(ui)))
        .inner
}

/// Driving keys belong to the focused canvas, rather than egui focus navigation.
fn canvas_keys(response: egui::Response) -> egui::Response {
    response.ctx.memory_mut(|memory| {
        memory.set_focus_lock_filter(
            response.id,
            egui::EventFilter {
                tab: true,
                horizontal_arrows: true,
                vertical_arrows: true,
                escape: true,
            },
        )
    });
    response
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn focused_canvas_retains_driving_and_capture_keys() {
        let ctx = egui::Context::default();
        let id = egui::Id::new("test-driving-canvas");
        let input = |events| egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(400.0, 300.0),
            )),
            focused: true,
            events,
            ..Default::default()
        };
        for frame in 0..2 {
            ctx.run_ui(input(vec![]), |ui| {
                let response =
                    canvas_keys(ui.interact(ui.max_rect(), id, egui::Sense::click_and_drag()));
                if frame == 0 {
                    response.request_focus();
                }
            })
            .drop_without_applying_deltas();
        }
        for key in [
            egui::Key::Tab,
            egui::Key::ArrowLeft,
            egui::Key::ArrowRight,
            egui::Key::ArrowUp,
            egui::Key::ArrowDown,
            egui::Key::Escape,
        ] {
            let output = ctx.run_ui(
                input(vec![egui::Event::Key {
                    key,
                    physical_key: None,
                    pressed: true,
                    repeat: false,
                    modifiers: Default::default(),
                }]),
                |ui| {
                    let response =
                        canvas_keys(ui.interact(ui.max_rect(), id, egui::Sense::click_and_drag()));
                    assert!(response.has_focus(), "{key:?} must reach driving input");
                },
            );
            output.drop_without_applying_deltas();
        }
    }
}
