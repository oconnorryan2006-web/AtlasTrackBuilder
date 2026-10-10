# ATLAS World Studio — connected road generator

Select a real road in the map, then export a local driving environment. Eligible vehicle roads whose OSM centreline intersects the **50 metre route corridor** get full ROAD collision surfaces. Ordinary roads stop at the corridor boundary. Complete connected roundabouts are preserved, including their central islands and entry/exit connections.

All at-grade carriageways, including the selected route, use one polygon union and triangulated collision surface. OSM one-way information affects signage only: **it does not stop the player driving the wrong way**. Terrain is cut away underneath the asphalt and its boundary follows the road surface.

Buildings use real OSM footprints, including relation courtyards and building parts; procedural walls and roofs are approximate. Missing heights use defaults. UK-style stop, give-way, speed, one-way, no-entry, roundabout and street-name signs are generated with poles and embedded textures. Speed limits are only taken from explicit OSM data. The interface and exported `generation_report.json` distinguish mapped signs from inferred placements and display data warnings.

## Windows: build and double-click

1. Extract this project into a **new folder**, keeping your old project as a backup.
2. Install 64-bit Python 3.12 or a compatible newer Python. The builder also locates the known Python 3.14 installation.
3. Close any running ATLAS application.
4. Double-click **build_windows.bat**. It installs requirements, runs tests, and builds with PyInstaller. Internet access is needed.
5. Double-click **run_windows.bat** (or **dist/ATLAS World Studio.exe**). Do not launch an older EXE from another folder.

The builder includes the backend, frontend, network generator, building/sign modules, scientific libraries, Shapely, Pillow and Qt WebEngine. It reports the failing build step and avoids overwriting a running application. Windows packaging has **not been executed in the Linux development environment**.

The desktop launcher writes caches and exports under `%LOCALAPPDATA%\ATLAS World Studio` in a packaged build; source runs default to `output`. `ATLAS_DATA_DIR` can override the directory. Backend port: 8743. Closing a desktop window stops the backend that window started.

## Python development

Create a virtual environment, then install `requirements.txt`. Start the desktop with `python src/launch.py`, or the backend with `python src/launch.py --serve`. Run all tests with `python -m unittest discover -s tests -v`. The test suite needs no online services; recorded OSM data and explicitly synthetic elevations are used where stated.

## Accuracy and verification

The actual **Greystone Road / Broad Road roundabout in Limavady** is included as an attributed OSM regression fixture. The live export also uses online elevation and land-cover data. Global elevation tiles are not surveyed road heights and cannot reproduce measured camber, kerbs or detailed bridge clearances.

Roundabout tags in the Limavady sample disagree about lane count. The generator reports its inferred consistent carriageway width; this is not a measured width. Sign positions, roofs and facades are not a surveyed replica. Accurate local LiDAR/road surveys and licensed reference photographs would improve these details. No additional model pack is required for basic functionality.

Grade-separated roads inside the corridor currently stop export with a clear error because the generator lacks a trustworthy independent bridge/tunnel deck-height profile. They are never welded into false level intersections. Incomplete roundabout data or partial OSM downloads also stop export rather than silently exporting gaps or missing buildings. Successful OSM tiles are cached for retry.

See [docs/VALIDATION.md](docs/VALIDATION.md) for automated checks, research references and the exact in-game test procedure. A successful export or geometry test does **not** verify driving in Assetto Corsa.

## Reusable UK sign models

`tools/export_sign_models.py` generates self-contained GLB models with embedded faces, metal posts and 3 mm plate backs/edges. Run `python tools/export_sign_models.py --output sign-models`. Diagram 609 is a mini-roundabout sign; full-size approaches use warning diagram 510. Artwork, dimensions, fonts and inferred positions are approximate; see `docs/UK_SIGN_REFERENCES.md`. No Tripo or remote desktop connection is required.

## Folder layout

| Location | Contents |
| --- | --- |
| `build_windows.bat` | Install, test and build the Windows app |
| `run_windows.bat` | Launch the current built app |
| `src/` | Backend, desktop launcher, geometry and frontend |
| `tests/` | Tests and recorded OSM fixtures |
| `docs/` | Validation procedure and sign research |
| `tools/` | Optional reusable sign-model exporter |
| `dist/` | Built EXE, created by the builder |
| `build/` | Temporary build files and generated spec |
| `output/` | Source-run caches and generated tracks |

The source ZIP excludes build products, caches and old executables. Extract into a fresh folder instead of merging into the old messy folder; keep the old folder until the new app is verified. Packaged app caches and exports stay in `%LOCALAPPDATA%\ATLAS World Studio`.
