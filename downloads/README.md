# ATLAS downloads

Download a ZIP by opening its file on GitHub and clicking the download raw file button.

- **ATLAS_World_Studio_rebuild.zip**: updated source. Extract it, open atlas-road-world-studio, and run build_windows.bat on Windows, then run_windows.bat. Source is organised into src, tests, docs and tools.
- **limavady_greystone_broad.zip**: Greystone Road / Broad Road, Limavady test track. Extract into the Assetto Corsa root, preserving content/tracks paths. Back up an existing track with that name first.
- **UK_sign_models.zip**: 13 procedural UK GLB sign models, embedded textures and approximate dimensions.

28 generator tests passed. The live Limavady KN5 export was parsed and its physical triangles checked at float32 precision. Windows EXE packaging and actual Assetto Corsa driving remain unverified. Road widths, buildings and inferred sign placements are approximations; see the included validation and research documents. No Tripo API was used.

SHA256SUMS.txt contains archive checksums.

Updated terrain validation avoids false float32 cross-product cancellation on valid thin triangles; real collapsed/inverted triangles remain rejected. Extract the new source ZIP into a fresh folder rather than merging into an old builder folder.
