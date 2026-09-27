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
Use the toolbar to select Logan or L200 and open the paint color picker. Each
vehicle remembers its color for the current session; **Original color** restores
its source paint. Switching vehicles respawns the car. The pointer starts released
so the controls are accessible; click the canvas to drive or press Tab to capture it.

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
wheel contact and spin are recomputed at each RK4 stage. Throttle requests engine
effort, with torque-limited launches and power-limited acceleration at speed.
Traction control limits driving force to the grip left after cornering. Opposite
direction input brakes before reversing; reverse has its own propulsion speed
limit. Rolling resistance and aerodynamic drag act independently of throttle.
See the [drivetrain contract](src/vehicle.rs) and [tuning fields](src/config.rs).
Wheel inertia and chassis collision are not simulated.

Models use wgame's shared textured meshes, scene batching, camera and depth
attachment. Spheres and cylinders come from its optional `wgame-gfx-3d` crate.
Ambient and directional light shade the imported car normals and terrain normals.
Blinn–Phong highlights follow the camera. Both car bodies use object-space normal maps;
wheels use tangent-space maps. Separate windshields and source mirrors use
their explicit mesh normals. Terrain uses its geometry normals. Color textures are decoded from
sRGB for lighting, while normal-map RGB remains unconverted data. Both vehicles
use opaque windows and omit their interiors. A separate paint mask restricts
recoloring to painted panels and detail parts, keeping glass, lamps, trim and wheels
unchanged. The mask is packed into normal-map alpha during loading, independently
of opacity. Picker values are converted from sRGB to linear RGB before shading.
The sun and material settings live in `src/render.rs`. Cast shadows are not simulated.
See [vehicle asset conventions](assets/README.md) for source models, baked maps,
and export requirements.

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
The ignored GPU tests require an adapter (Mesa lavapipe works), check both car
assets from four angles, body and cargo-bed close-ups, normal mapping and depth
across render passes. They can save PPM images to an existing directory set in
`DRIVE_RENDER_OUTPUT`. Web compilation still needs a browser
check for rendering and input.
