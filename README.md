# Drive

Standalone extraction of the driving game from `agerasev/games`'s `logan`
branch (`dd03f65`), preserving its filtered history. See
[history provenance](history/README.md) for the original-to-filtered commit map.

This is the original Macroquad implementation, ready for migration to wgame
and phy. The vehicle equations, terrain, camera controls, and experimental
material are unchanged. Both Logan and L200 assets are retained; the game
starts with Logan.

```sh
cargo run --locked --release
```

- WASD / arrows: accelerate, reverse and steer.
- Space, or forward and reverse together: brake.
- Mouse: orbit the camera; wheel: change camera distance.
- Tab: toggle mouse capture. When released, hold the left button to orbit.
- Escape: quit. Falling below the terrain triggers automatic respawn.

The desktop launcher finds assets in this checkout. Historical filtered commits
contain only selected files; the standalone manifest and launcher are added in
the extraction commit.
