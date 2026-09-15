#[macroquad::main("Drive")]
async fn main() -> Result<(), anyhow::Error> {
    macroquad::file::set_pc_assets_folder(concat!(env!("CARGO_MANIFEST_DIR"), "/assets"));
    macroquad::miniquad::window::set_window_size(1280, 720);
    drive::games::drive::main().await
}
