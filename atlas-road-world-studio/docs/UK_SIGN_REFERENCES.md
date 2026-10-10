# UK sign models: references and validation

Prepared for ATLAS World Studio. No application code changed by this research task.

## Evidence and scope

The official Department for Transport working-drawing index is:
https://www.gov.uk/government/publications/traffic-signs-working-drawings-tsrgd-2016

Related references:
- Traffic Signs Manual, Chapter 3: https://www.gov.uk/government/publications/traffic-signs-manual
- TSRGD 2016: https://www.legislation.gov.uk/uksi/2016/362/contents/made
- For the project's Northern Ireland location, also check the applicable Traffic Signs Regulations (Northern Ireland), rather than representing Great Britain TSRGD as the sole local authority.

The official drawing index and publishing-service request returned proxy HTTP 403 in this session. Exact working-drawing dimensions, colour specifications, current licensing exceptions, and mounting requirements have therefore **not** been verified from those drawings. Diagram identification and visual descriptions below are established technical knowledge, not a claim that official PDFs were downloaded. Obtain the actual drawing for an exact replica.

## Required sign faces

| Diagram | Face | Shape and orientation | Model guidance |
|---|---|---|---|
| 601.1 | STOP | Regular octagon, horizontal top/bottom edges; red field, white border and white STOP | Use a genuine octagonal mesh, not a rectangular textured plane. Fit lettering from the official drawing. |
| 602 | Give way | Equilateral triangle pointing DOWN; red border and white interior | Use a downward triangle with unobstructed inner white field; no invented STOP lettering. |
| 670 | Speed limit | Circle; red ring, white interior, black numerals | A UK numeric limit denotes mph. Do not convert explicit mph values. Do not use an invented numeric limit or conflate national-speed-limit signage with a numeric 60 sign. |
| 616 | No entry | Circle; red field and horizontal white bar | Preserve the full white bar and circular silhouette. Place toward the prohibited entry. |
| 652 | One-way traffic | Upright blue rectangular panel and white upward arrow | Keep rectangular proportions and arrow geometry from the actual diagram; align its placement with allowed traffic direction. |
| 609 | Mini-roundabout | Blue circle with white circulating arrows, indicating clockwise circulation | **This is a mini-roundabout sign.** Do not present it as the default sign for every full-sized roundabout. It has no red prohibition border. Full roundabout approaches may use different warning/direction/chevron signage. |

The legal sign face does not make the simulated road physically one-way. Players should remain able to travel both ways.

## Concrete issues in the current procedural implementation

1. `sign_texture('roundabout')` draws a red outer circle and then a smaller blue circle. Diagram 609 needs a blue/white treatment, not a red prohibition rim. Its meaning is specifically a mini-roundabout.
2. Circular meshes use radius 0.38 m while UV mapping uses division by 0.9. The face samples only texture radius approximately 0.422; the artwork's outer edge extends to approximately 0.469. This crops the outer sign ring. Use UVs derived from each actual silhouette's drawing bounds.
3. The STOP octagon has geometric flat top/bottom edges and the give-way face points down, which are appropriate baseline silhouettes. Font fallback is not a verified Transport reproduction.
4. `oneway=-1` currently reverses facing but leaves entry/no-entry candidates at the wrong polyline ends. Swap candidate locations as well as facing.
5. Sign faces are coplanar front/back triangles. There is no physical plate edge or separate grey rear panel. Double-sided triangles can show identical regulatory artwork on the back.
6. Pole top is 2.1 m but the sign centre is 2.05 m. This is not a 2.1 m clearance to the sign's lowest edge: the lowest circle edge is about 1.67 m. Set ground-to-bottom clearance independently of sign size.
7. Street-name face geometry is portrait (0.6 by 0.8 m), while its texture is a landscape board. Use a landscape board if that sign type is exported.

## Metric asset contract

Use GLB/glTF 2.0 with embedded or packaged PBR textures. Define **1 unit = 1 metre**. Preserve the format's Y-up convention and record explicit road-engine conversion; do not silently swap axes on import. Place the origin at the pole's ground contact. Define the front-facing axis in metadata and export identity scale.

Practical initial model parameters (art/design defaults, **not certified drawing dimensions**):
- Circular diameter: 0.60 or 0.75 m, configurable by context.
- Octagon across flats: 0.60 or 0.75 m; do not confuse across-flats width with vertex-to-vertex diameter.
- Triangle nominal side: 0.60 or 0.75 m, configurable.
- Aluminium plate thickness: approximately 2–3 mm, with a restrained edge bevel.
- Galvanised steel post outside diameter: approximately 60–76 mm. Visible rear clamps/brackets are optional detail.
- Initial lowest-edge mounting height: 2.1 m for a pedestrian-clearance example; approximately 1.5 m may suit some non-pedestrian rural settings. These are approximate modelling choices requiring site/current manual verification, especially above cycle facilities.
- Reflective front material should be principally nonmetallic, with a controlled retroreflective visual approximation. Avoid making the sign glow as an emissive billboard. Metal post/back plate can use metalness near 1 with plausible roughness. Exact reflection depends on renderer support.
- Front texture: deterministic, sharp vector/raster artwork. Use 1024 px for an individual sign face and suitable mipmaps/alpha handling. Avoid image-generator-invented text.

For Assetto Corsa, treat GLB as a reusable authoring asset, not proof of game compatibility. KN5 conversion must preserve silhouette, UVs, materials, backface appearance and placement. Keep signs out of the drivable collision mesh unless their collision behaviour is deliberately configured.

## Tripo3D workflow

There is no callable Tripo3D capability or installable recommended Tripo plugin in this session. These prompts can be used if the user supplies Tripo-generated assets or a supported Tripo connection becomes available. No plugin was installed and no external account action was taken.

Preferred reference-image workflow:
1. Obtain the official sign drawing or a user-authorised photograph; make a clean orthographic front reference with the correct silhouette.
2. Generate the structural plate, grey rear panel, pole and brackets. Apply the exact regulatory front face as a deterministic decal/texture afterward. Inspect against the reference; generated text/arrows are not authoritative.
3. Export GLB with textures. Record dimensions, orientation, asset licence and any Tripo output-use conditions. Test its authoring and KN5 import independently.

Prompt template:

> Photorealistic UK roadside [DIAGRAM / SIGN TYPE] asset based strictly on the attached authorised reference. Single sign, isolated. Exact [circular / downward equilateral triangular / flat-top octagonal / rectangular] plate silhouette. Thin approximately 3 mm aluminium plate, neutral grey rear, galvanised cylindrical steel post approximately 76 mm diameter, realistic rear fastening brackets. Metric scale, sign [specified dimension] metres, ground-contact pivot. Lowest sign edge [specified clearance] metres above ground. Clean low-poly game-ready topology, restrained wear, no extra objects, no invented lettering or arrows. Preserve the reference's front graphics and UV alignment, provide PBR textures and embedded-texture GLB. Front graphics will be replaced with verified deterministic artwork after generation.

For stop/speed faces, specify the exact text/number but still replace/validate lettering afterward. Prefer the DfT drawing's genuine glyph outlines when their permitted use is established. Do not claim DejaVu/Arial is Transport. A commercial font is not automatically licensed for asset redistribution.

## Acceptance checks

- Bounding box matches specified metres; pole contact is Y=0; intended sign clearance measures to the LOWEST plate edge.
- Front silhouette matches the diagram; no square backing corners around circles/triangles/octagons.
- Face is visible from the intended approach; back is a separate grey material; edge has thickness.
- Speed values are source-derived mph; no-entry and one-way orientations are validated for both OSM `oneway=yes` and `oneway=-1`.
- No red rim on diagram 609; identify inferred versus actually mapped signage.
- No text distortion, UV cropping, mirrored lettering, missing textures, invalid triangles or unintended collision walls.
- Sign/post positions are outside every road surface at junctions and both vehicle travel directions remain physically possible.
- Inspect daylight and headlights in the target renderer. Game acceptance remains separate from asset-file validation.

## Asset rights

Check each official drawing's copyright/OGL statement and third-party exceptions before redistribution. Do not assume a government page grants rights to every embedded font or asset. Check Tripo's applicable plan and output terms; no current Tripo terms were verified here. User photos/model files should have explicit permission for derivative creation and redistribution. Preserve OSM attribution and applicable ODbL obligations for source map data separately from model/font/texture licensing. Avoid importing Google 3D city geometry or third-party mod assets without an explicit permitted-use basis.

## Verified Tripo API integration route

The official SDK README was successfully read from:
https://github.com/VAST-AI-Research/tripo-python-sdk/blob/master/README.md

It identifies the SDK as the official Tripo client, installs as `tripo3d`, reads `TRIPO_API_KEY`, supports text/image/multiview generation and exposes `get_balance()` with available/frozen balances.

Verified implementation:
https://github.com/VAST-AI-Research/tripo-python-sdk/blob/master/tripo3d/client.py

- Standard API base: `https://api.tripo3d.ai/v2/openapi`
- Create task: `POST /task`
- Query task: `GET /task/{task_id}`
- Balance: `GET /user/balance`
- Upload capability: `POST /upload/sts/token`, followed by an authenticated upload to the returned storage host. The SDK also supports a legacy upload path.
- Some newer task types use `https://openapi.tripo3d.ai` and `/v3/tasks/{task_id}`. Do not assume v2 responses cover every model/edit operation.
- API reference: https://github.com/VAST-AI-Research/tripo-python-sdk/blob/master/docs/API.md

The API-platform page `https://platform.tripo3d.ai/` could not be loaded here (proxy 403); exact dashboard labels, account setup requirements, current prices and plan/output rights were not verified. A practical user route is to sign in to the Tripo API platform, obtain API access and create a key there, then store it **securely in cloud environment settings under `TRIPO_API_KEY`**, never in chat or committed code. Website/UI subscription access should not be assumed to include API credits. Verify the account balance before submitting generation tasks and establish a batch limit; each task can consume account credit. No generation task was submitted during this research.

Add the API domain and the actual returned upload/model-download hosts to the network allowlist when required. Do not replace an unknown existing allowlist or invent broad storage permissions.

Important implementation constraint: the inspected SDK's `_download_with_ssl_retry` automatically retries certificate failures with verification disabled. Do **not** adopt that behaviour. Keep TLS verification enabled, fix trust configuration if necessary, or download returned model URLs through a verified-TLS HTTP client. The SDK's API reference and actual current implementation also differ on the shape of upload results, so pin/review the installed SDK version before integrating.

A remote desktop controller is a separate capability from this API. Giving this environment a Tripo key does not provide access to the user's Windows desktop. Do not claim that a local Tripo login or installed Windows application creates assistant desktop access.
