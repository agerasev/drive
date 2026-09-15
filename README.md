# Drive

A small driving playground using [wgame](../wgame) for desktop/WebGL2 rendering
and [phy](../phy) for rigid-body integration. The Logan and L200 models, terrain,
and suspension/friction model come from the original game. Its filtered Git
history is preserved; see [provenance](history/README.md).

## Run

Initialize the sibling-repository submodules, then start the game:

```sh
git submodule update --init --recursive
cargo run --locked --release
```

Assets are embedded, so the executable can run from any working directory.

- WASD / arrows: accelerate, reverse and steer.
- Space, or forward and reverse together: brake.
- Mouse: orbit while captured; wheel: zoom.
- Tab: toggle capture. When released, hold the left button to orbit.
- 1 / 2: switch to Logan / L200; R: respawn.
- P: pause; T: toggle half-speed simulation.
- Escape: quit. Falling off the terrain triggers automatic respawn.

The simulation pauses when the window loses focus and releases mouse capture.
Click the canvas to focus it in a browser; use Tab to request pointer lock.

## Web

Install the `wasm32-unknown-unknown` Rust target and Trunk, then build:

```sh
./scripts/build-web.sh
python3 -m http.server --directory dist 8080
```

Open `http://localhost:8080`. For a subdirectory deployment, pass its URL prefix
as the build script's argument, for example `./scripts/build-web.sh /drive/`.

## Simulation and rendering

Physics uses phy's RK4 solver at 240 steps per second, independent of rendering.
Catch-up is capped at 100 ms per frame. Suspension only pushes away from terrain;
wheel contact and spin are recomputed at each RK4 stage. The experimental
fixed wheel-speed drivetrain is retained: engine power/torque limits and wheel
inertia in the asset configs are not yet simulated. There is no chassis collision.

Models use wgame's shared textured meshes, scene batching, camera and depth
attachment. Spheres and cylinders come from its optional `wgame-gfx-3d` crate.
Ambient and directional light shade the imported car normals and terrain normals.
Blinn–Phong highlights follow the camera. Wheels use the existing tangent-space
normal map; car bodies and terrain use their geometry normals. Color textures
are decoded from sRGB for lighting, while normal-map RGB remains unconverted data.
The sun and material settings live in `src/render.rs`. Cast shadows are not simulated.

## Checks

```sh
cargo test --locked
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked --release -- --smoke
cargo test --locked --bin drive -- --ignored
./scripts/build-web.sh
```

`--smoke` draws twelve frames and exits without capturing the pointer. The CPU
tests cover contact geometry, suspension, driving/braking and fixed-step timing.
The ignored GPU test requires an adapter (Mesa lavapipe works), checks both car
assets and depth across render passes, and can save PPM images to an existing
directory set in `DRIVE_RENDER_OUTPUT`. Web compilation still needs a browser
check for rendering and input.
