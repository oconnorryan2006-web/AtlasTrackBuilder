# ATLAS downloads

Download a ZIP by opening its file on GitHub and clicking the download raw file button.

- **ATLAS_World_Studio_rebuild.zip**: updated source. Extract it, open atlas-road-world-studio, and run build_windows.bat on Windows, then run_windows.bat. Source is organised into src, tests, docs and tools.
- **limavady_greystone_broad.zip**: Greystone Road / Broad Road, Limavady test track. Extract into the Assetto Corsa root, preserving content/tracks paths. Back up an existing track with that name first.
- **UK_sign_models.zip**: 13 procedural UK GLB sign models, embedded textures and approximate dimensions.

28 generator tests passed. The live Limavady KN5 export was parsed and its physical triangles checked at float32 precision. Windows EXE packaging and actual Assetto Corsa driving remain unverified. Road widths, buildings and inferred sign placements are approximations; see the included validation and research documents. No Tripo API was used.

SHA256SUMS.txt contains archive checksums.

Updated terrain validation avoids false float32 cross-product cancellation on valid thin triangles; real collapsed/inverted triangles remain rejected. Extract the new source ZIP into a fresh folder rather than merging into an old builder folder.

If the builder reports it cannot find build/src/server.py, download the corrected build_windows.bat here and replace the batch file in your project root, then rerun it. The source ZIP also includes the fix.

The latest source ZIP includes the redesigned charcoal/lime studio UI and new ATLAS logo/icon. Download the entire source ZIP and rebuild to get both. ATLAS-interface-preview.png shows the UI with recorded route/API fixture responses and real Esri imagery; it is not an in-game screenshot. ATLAS-logo.png and ATLAS-icon.ico are also downloadable separately.

Hedge collision update: physical WALL barriers now avoid all generated roads; the KN5 generator and GLB exporter share the new sign models. 30 offline tests pass, including binary hedge export and sign geometry regression checks. Rebuild Windows and regenerate tracks; existing exports do not update automatically.

Continuous hedge update: both backroad verges and normal turnoff roads receive edge-following hedges even without mapped OSM hedges. Houses retain lower 0.85 m hedges; driveways stay open without side hedges. Turnoff openings follow the asphalt edge, with 35 cm verge clearance. Opaque foliage eliminates tree-texture holes. 37 offline tests pass. Download the rebuild ZIP, rebuild the app and regenerate tracks.
