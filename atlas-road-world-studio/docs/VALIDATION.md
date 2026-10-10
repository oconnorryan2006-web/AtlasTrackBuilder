# Validation and handover

## Implemented

- Metric 50 m buffer of the full projected selected route; exact ordinary-road clipping.
- Complete roundabouts assembled from split OSM ways; interior islands retained.
- A unified at-grade road collision surface, upward triangle winding and area-weighted normals. No physical one-way restriction.
- One selected/network ROAD material; lane markings are separate visual geometry following mapped lanes.
- Shared continuous elevation interpolation and terrain boundaries cut against actual road collision triangles. Geometry is checked at KN5 float32 precision; clipping slivers are rejected after welding.
- Building ways, multipolygon relation courtyards, complex outlines and building parts; parent volumes are removed under parts. Height/levels data and documented defaults; procedural facades and flat roofs.
- Textured UK-style road controls and street-name plates, roadside poles, approaching-traffic facing and reversed-oneway handling. Unsupported sign codes are reported.
- Export generation report and OSM attribution; rendered track maps include the complete drivable network.
- Windows packaging includes the new modules and Shapely; tests precede the build. File-lock handling no longer mistakes a post-build EXE-copy error for PyInstaller failure.

## Automated checks

Run `python -m unittest discover -s tests -v` from the project root.

The suite covers 20 m / 45 m road inclusion and 51 m exclusion; sparse routes and strict clipping; T/cross junction connectivity and non-overlapping triangle areas; split roundabout assembly, complete loops, island holes, and identical collision geometry for different one-way tags; malformed/incomplete data; continuous sparse-sample elevation; terrain/asphalt separation and float32 clipping slivers; building courtyards/complex footprints/ground placement/parts; mapped and inferred signs, explicit speed data, reversed one-way sign ends, UK roundabout artwork; sparse OSM query coverage and partial-download errors; recorded Limavady geometry; full export pipeline; and binary KN5 parsing, indices, materials, textures and spawn nodes.

Synthetic elevation fixtures are explicitly flat or artificial. They do not validate geographic heights. `tests/fixtures/limavady.json` is real OSM geometry; its source attribution and summary are included. `tests/fixtures/limavady-route.json` selects the Greystone approach to the roundabout.

## Required Windows / in-game checks (not performed on Linux)

1. Extract the updated source into a fresh folder; close ATLAS; double-click `build_windows.bat`. Verify it reports successful tests and a new `dist/ATLAS World Studio.exe`.
2. Double-click that new EXE. Check the map loads and the backend remains running.
3. Back up any existing track of the same name before installing. Extract the supplied test track ZIP into the Assetto Corsa root so its folder is `content/tracks/limavady_greystone_broad`.
4. Load the track in Content Manager/Assetto Corsa **Practice or Hotlap**, with a normal road car. Race AI lines are not supplied.
5. Verify the car spawns above asphalt and settles normally. Drive the Greystone approach, enter the roundabout, complete two full loops clockwise, turn around, and complete two loops anticlockwise. Exit via every generated approach and return.
6. Drive the nearby side roads, crossing each junction at low and moderate speeds. Check for collision steps, invisible barriers, tyre sinking, holes, overlapping-surface vibration and road/terrain gaps.
7. Check buildings and sign faces appear. Compare the layout with the real location. Inferred sign placements and procedural buildings must not be treated as a verified roadside inventory.
8. Regenerate the same route using the updated EXE and repeat the drive, ensuring the packaged modules work rather than only the supplied test ZIP.
9. Record any failed location with coordinates, a screenshot and the application log. Actual in-game drivability remains awaiting this procedure.

## Research performed by the subagent

The subagent read these upstream projects through GitHub. Their approaches informed the design; software/data/model licences are distinct.

- [OSM2World roads and junctions](https://github.com/tordanik/OSM2World/blob/master/core/src/main/java/org/osm2world/world/modules/RoadModule.java): physical junction surfaces precede lane logic.
- [OSM2World buildings](https://github.com/tordanik/OSM2World/blob/master/core/src/main/java/org/osm2world/world/modules/building/BuildingModule.java): multipolygon relations and duplicate suppression.
- [Arnis building handling](https://github.com/louis-e/arnis/blob/main/src/element_processing/buildings.rs): outer/inner rings, building parts and mapped roof metadata.
- [Arnis elevation](https://github.com/louis-e/arnis/blob/main/src/elevation/mod.rs): caching, provider fallback and terrain artefact handling.
- [Blosm](https://github.com/vvoovv/blosm): optional external Blender add-on for scene inspection/detailing. No assistant plugin or model pack was required for the functional fixes.
- [Overpass bbox implementation](https://github.com/drolbr/Overpass-API/blob/master/src/overpass_api/data/bbox_filter.cc): checks crossing way segments, not just nodes.

For more geographic accuracy, supply licensed Limavady elevation/road surveys and photographs of signs, kerbs, junction widths and distinctive buildings. Northern Ireland LiDAR coverage was not verified; England datasets should not be assumed to cover Limavady. Optional later assets: regional facades, kerbs, lamp posts and guard rails in metre-scaled FBX/GLB/OBJ with textures and reuse rights.

## Current live Limavady export

Greystone Road/Broad Road: 256.9 m selected route, 19 ordinary road sections, one complete roundabout, 341 buildings, 145 trees and 26 inferred signs (zero mapped sign nodes). Public OSM and terrain/elevation services succeeded. The KN5 was independently parsed, and physical triangle winding/nondegeneracy was checked after float32 serialization. OSM roundabout lane tags conflict; the exported report warns that its continuous 6.5 m width is inferred. This is an engineering export check, not a road survey or in-game test.

## Thin terrain triangle validation fix

The screenshot error `Degenerate triangle: 1GRASS_NETWORK_0` was reproduced using valid float32-serialized positions whose float32 cross product cancels to zero. Validation now computes in float64 after rounding coordinates to KN5 float32 precision. A regression accepts the valid thin triangle and still rejects collapsed and inverted triangles. The exact route from the screenshot has not been supplied or reproduced; regenerate it in the new build to confirm that route.
