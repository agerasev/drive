# Vehicle assets

Each vehicle body has a triangulated `model.obj`, an sRGB `color.png` and a
raw RGB `normal.png`. Both bodies use object-space normal maps and 4096×4096
atlases with matching color/normal UVs. Independent runtime wheels use
tangent-space normals. Wheels remain separate so steering, spin and suspension
remain independent of the body mesh. Each vehicle also has `details.obj` and
`details.png` for independently shaded geometry: source mirrors, a fitted
windshield and its surround. L200 details also include a plain opaque underbody.
These details use explicit mesh normals without the body normal map. The L200
body OBJ contains independent `l200_cab` and `l200_bed` objects with disjoint
atlas regions.

The Logan body is rebuilt from the edited detailed meshes in `logan-low3.blend`;
the old low-poly proxy and its textures are not used. Its shader colors supply
the gray paint and lamp tints. The interior is omitted and windows are opaque
black. The front grille bars use the body paint color. Lamp colors retain their
authored hues through transparent source covers in the opaque texture; emission is represented as color rather than simulated light.
The L200 body uses `L200-OBJ.zip` with its supplied silver texture and bump map.
Its windows remain opaque.
The Logan body and forward L200 cab use guided quad boxes fitted to source
cross-sections. Every wheel opening follows the corresponding source aperture,
including its asymmetric shoulders; no circular cut is substituted. A small
clearance allowance avoids clipping the source lip.
The L200 rear cab uses a newly sampled surface mesh with constrained panel
boundaries. Its bed is built from explicit broad side panels, rim, tailgate,
bumper and liner, with simple inner wheel boxes and a flat central load floor.
Handles, corrugations and shallow recesses are baked detail, not geometry guides.
The bed side panels are triangulated within their authored contours so strips
cannot cross at the steep wheel-opening shoulders.
L200 handle areas use smooth door sheets, with recesses represented by baked
normal detail. A fixed longitudinal column follows each roof/pillar
corner so the silhouette cannot alternate between adjacent grid columns. Logan
keeps that alignment through its rear pillars and blends it gradually into the
flatter boot and bonnet sections.
The generated front and rear use continuous outer envelopes
across openings. Independent
ray hits through grille slots or panel gaps must not pull cage vertices into
internal geometry. Large connected UV patches cover the body panels; projection
along cage normals supplies albedo and object-space normals. The bed side
charts project laterally so the front closing wall cannot deflect their rays
away from the outer skin. The L200
preserves its open cargo bed, liner and wheel housings. Its front end grid starts
beyond the wheel arch to retain the complete bumper. A recessed black floor and
inner wheel-house walls close the underside without suspension or differential
geometry. The cab rear follows the source wall and sloping pillar silhouette with
new topology; shared boundary vertices close its join to the forward shell.
Cab and bed are baked independently from their respective source panels into separate
regions of the shared atlas; neither bake may project onto the other part.
The bed liner has a plain dark finish. Its walls and wheel boxes use the new
mesh normals; source rib normals are retained on the matching flat load floor.
The liner is isolated from exterior baking to prevent unrelated source surfaces
from leaking into the cargo cavity.
There is no connecting band or mesh spanning their separation. Both objects use
the same vehicle transform and physics. The bed and black underbody must remain
within the rear bumper silhouette. The cab/bed boundary follows the bed's front
wall beneath
the rear window, then its sloping front side edges; the cab's rear pillars remain
part of the cab. This physical boundary is distinct from the reconstruction
seam joining the fitted cab rear to its generated door surface.

Mirrors retain their authored housings, stems, panes and split normals, with
independent color sampling. Each windshield has an explicit continuous outline
and smooth curvature fitted to its source. A narrow surround stitches that
outline into an opening cut in the body cage. The old projected glass boundary
is removed with the opening; triangle-dependent projection cannot distort the
new outline. Glass remains opaque.
Source copies, Blender scenes, scripts and previews are kept locally in the
git-ignored `vehicle-work/` directory.

## Export contract

- Positions use meters, X across the car, +Y forward and +Z up. Match wheel
  arches to the axle positions in each vehicle's `config.json`, accounting for
  suspension compression at rest. Do not include the source wheels in the body.
- Triangulate before baking and preserve those triangles during export. Both
  bodies have smooth mesh normals and a normal bake in the mesh's XYZ coordinates.
  Use `NormalSpace::Object`: RGB decodes to `2 * RGB - 1` and transforms with the
  instance's inverse transpose. Their normals do not depend on a triangle's UV
  tangent frame, and `NormalY` does not apply. Mesh deformation requires updating
  the object-space map.
- Orient source normals toward the visible target surface. Some imported Logan
  panels have inward normals that a two-sided source render can conceal.
  The CPU asset test compares the decoded object normals with triangle winding.
- Export explicit normals and UVs for every triangle corner. UVs use top-left
  image coordinates: export Blender UV `(u, v)` as `(u, 1 - v)`.
- Wheel tangent maps retain Blender's +Y convention and use `NormalY::Positive`.
  Normal RGB is raw data and must never undergo sRGB conversion. Albedo is sRGB;
  encode linear source colors before writing the color PNG.
- Keep UV islands disjoint and pad their baked pixels for linear filtering.
  Color and normal maps must be fully opaque, including their padding.
- Detail meshes use their own UVs and color texture. Do not apply the body
  normal map to them. Preserve source mirror split normals and keep windshield
  normals continuous across shared positions; its frame may have a separate
  normal at the same position. `opaque_underbody` uses black albedo and zero
  specular reflection; it must block sight lines through the floor and arches.

For visual verification, run the ignored GPU tests with an existing output
directory, then inspect both vehicles from the saved front/rear views and
close-ups of the windshields, mirrors, doors, wheel arches, cargo bed and low
front/rear bumpers:

```sh
mkdir -p vehicle-work/game-checks
DRIVE_RENDER_OUTPUT="$PWD/vehicle-work/game-checks" \
  cargo test --locked --bin drive -- --ignored
```
