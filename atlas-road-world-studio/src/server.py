#!/usr/bin/env python3
"""
ATLAS Road World Studio — Backend server
OSM road data → elevation → FBX with AC markers → Assetto Corsa track zip
"""

import json, math, os, shutil, struct, time, zipfile, io
import threading, queue
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs, urlencode
from urllib.request import urlopen, Request
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from scipy.interpolate import splprep, splev, interp1d
from scipy.ndimage import uniform_filter1d, gaussian_filter1d, median_filter
from pyproj import Transformer

class ElevationUnavailable(Exception):
    """Elevation or land-cover data could not be obtained.

    Raised rather than silently substituting flat ground: a flat track looks
    plausible but is wrong, which is worse than a clear failure.
    """


EXPORT_PROGRESS = {"stage": "idle", "done": 0, "total": 0, "failed": 0}

PORT = 8743
OUTPUT_DIR = os.environ.get("ATLAS_DATA_DIR", os.path.join(os.path.dirname(os.path.dirname(__file__)), "output"))
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ─── OSM Query ────────────────────────────────────────────────────────────────

def search_osm_roads(query: str) -> dict:
    nom_url = (
        f"https://nominatim.openstreetmap.org/search"
        f"?q={query.replace(' ', '+')}&format=json&limit=8&addressdetails=1"
    )
    try:
        req = Request(nom_url, headers={"User-Agent": "ATLAS Road World Studio/1.0"})
        with urlopen(req, timeout=10) as r:
            results = json.loads(r.read())
    except Exception as e:
        return {"error": str(e), "results": []}

    out = []
    for item in results[:8]:
        out.append({
            "display_name": item.get("display_name", ""),
            "osm_type":     item.get("osm_type", ""),
            "osm_id":       item.get("osm_id", ""),
            "lat":          float(item.get("lat", 0)),
            "lon":          float(item.get("lon", 0)),
            "boundingbox":  item.get("boundingbox", []),
            "type":         item.get("type", "road"),
        })
    return {"results": out}


OVERPASS_ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://lz4.overpass-api.de/api/interpreter",
    "https://z.overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
]


_overpass_good = None   # last mirror that answered — always tried first


def _overpass(q: str, timeout: int = 40):
    """
    Run an Overpass query with HEDGED parallel requests. The last-known-good
    mirror fires immediately; another mirror joins the race every few seconds
    (or the moment one fails), and the first success wins. Sequential
    mirror-walking meant a run of dead mirrors cost their full read timeouts
    stacked end to end — minutes per call — and the old rotation actively
    moved the working mirror away from the front.

    Overpass has a second overload mode that is easy to miss: HTTP 200 with a
    TRUNCATED result and a "remark" saying the query timed out server-side.
    Accepting that silently produces exports with mysteriously missing
    barriers/vegetation, so a remark is treated exactly like a busy mirror.
    """
    global _overpass_good
    order = list(OVERPASS_ENDPOINTS)
    if _overpass_good in order:
        order.remove(_overpass_good)
        order.insert(0, _overpass_good)

    def one(url):
        encoded = urlencode({"data": q})
        req = Request(
            url + "?" + encoded if len(encoded) < 6500 else url,
            data=None if len(encoded) < 6500 else encoded.encode(),
            headers={"User-Agent": "ATLAS Road World Studio/1.0",
                     "Content-Type": "application/x-www-form-urlencoded"})
        with urlopen(req, timeout=timeout) as r:
            data = json.loads(r.read())
        remark = str(data.get("remark", "")).lower()
        if remark and any(w in remark for w in
                          ("timed out", "timeout", "error",
                           "out of memory", "load")):
            raise IOError(f"partial result: {remark[:70]}")
        return data

    STAGGER = 5.0           # seconds before the next mirror joins the race
    waits = (3.0, 8.0)
    last = None
    for attempt in range(len(waits) + 1):
        results = queue.Queue()

        def worker(url, _rq=results):
            try:
                _rq.put((url, one(url), None))
            except Exception as e:
                _rq.put((url, None, e))

        started = finished = 0
        next_start = time.monotonic()
        while finished < len(order):
            now = time.monotonic()
            if started < len(order) and now >= next_start:
                threading.Thread(target=worker, args=(order[started],),
                                 daemon=True).start()
                started += 1
                next_start = now + STAGGER
            try:
                url, data, err = results.get(timeout=0.25)
            except queue.Empty:
                continue
            if err is None:
                _overpass_good = url          # pin the winner for next time
                return data
            finished += 1
            last = err
            busy = any(c in str(err) for c in
                       ("429", "504", "503", "partial result"))
            print(f"  [overpass] {url.split('/')[2]} "
                  f"{'busy' if busy else 'failed'} ({err}) — racing next")
            next_start = time.monotonic()     # a failure frees the next slot
        if attempt < len(waits):
            print(f"  [overpass] all mirrors busy — waiting "
                  f"{waits[attempt]:.0f}s and retrying "
                  f"({attempt + 1}/{len(waits)})")
            time.sleep(waits[attempt])
    raise IOError(f"all Overpass mirrors failed (last: {last})")


def _stitch_segments(segments: list, seed_idx: int = None,
                     tol_m: float = 40.0, far_tol_m: float = 300.0) -> tuple:
    """
    Chain road segments (lists of [lat,lon]) into one continuous route by
    matching endpoints. Greedy: start from the seed (or longest) segment and
    repeatedly attach the nearest-endpoint segment to either end, reversing
    as needed.

    Two acceptance bands, because stopping at the first gap silently truncates
    the road — a single roundabout or offset junction node used to drop
    everything beyond it:
      • within tol_m  — treat as a genuine topological join, accept as-is.
      • up to far_tol_m — a junction-sized gap; accept ONLY if the candidate
        carries on in roughly the same direction. That guard is what stops the
        chain hopping onto the opposite carriageway of a divided road and
        zig-zagging back down it.
    Returns (coords, used_count).
    """
    if not segments:
        return [], 0
    segs = [list(s) for s in segments if len(s) >= 2]
    if not segs:
        return [], 0

    def seg_len(s):
        return sum(_haversine(s[i][0], s[i][1], s[i+1][0], s[i+1][1])
                   for i in range(len(s)-1))

    if seed_idx is None or seed_idx >= len(segs):
        seed_idx = max(range(len(segs)), key=lambda i: seg_len(segs[i]))

    chain = segs.pop(seed_idx)
    used = 1

    def heading(pts, at_end):
        """Unit direction pointing OUT of the chain/segment at one end."""
        if len(pts) < 2:
            return (0.0, 0.0)
        if at_end:
            a, b = pts[max(0, len(pts) - 4)], pts[-1]
        else:
            a, b = pts[min(len(pts) - 1, 3)], pts[0]
        dy, dx = b[0] - a[0], (b[1] - a[1]) * math.cos(math.radians(b[0]))
        n = math.hypot(dx, dy) or 1.0
        return (dx / n, dy / n)

    def continues(chain_dir, cand_dir):
        # reject a candidate that doubles back (opposite carriageway)
        return (chain_dir[0] * cand_dir[0] + chain_dir[1] * cand_dir[1]) > -0.34

    bridged = 0
    while segs:
        head, tail = chain[0], chain[-1]
        best = None   # (dist, seg_i, attach_at, reverse)
        for i, s in enumerate(segs):
            for attach_at, cpt in (("tail", tail), ("head", head)):
                for rev, spt in ((False, s[0]), (True, s[-1])) if attach_at == "tail" \
                                 else ((False, s[-1]), (True, s[0])):
                    d = _haversine(cpt[0], cpt[1], spt[0], spt[1])
                    if best is None or d < best[0]:
                        best = (d, i, attach_at, rev)
        if best is None or best[0] > far_tol_m:
            break

        _, i, attach_at, rev = best
        s = segs[i]
        cand = list(reversed(s)) if rev else s

        if best[0] > tol_m:
            # junction-sized gap: only bridge it if the road carries on
            c_dir = heading(chain, attach_at == "tail")
            n_dir = heading(cand, attach_at != "tail")
            if attach_at == "tail":
                n_dir = heading(cand, False)
                n_dir = (-n_dir[0], -n_dir[1])
            if not continues(c_dir, n_dir):
                # not a continuation — drop it so the search can move on
                segs.pop(i)
                continue
            bridged += 1

        segs.pop(i)
        s = cand
        if attach_at == "tail":
            # drop duplicated joint point if coincident
            if _haversine(chain[-1][0], chain[-1][1], s[0][0], s[0][1]) < 1.0:
                s = s[1:]
            chain.extend(s)
        else:
            if _haversine(chain[0][0], chain[0][1], s[-1][0], s[-1][1]) < 1.0:
                s = s[:-1]
            chain = s + chain
        used += 1

    if bridged:
        print(f"  [geometry] bridged {bridged} junction-sized gap(s)")
    return chain, used


def _struct_flag(tags: dict) -> int:
    """1 if a way is carried on a structure the DEM cannot see.

    Elevation data measures the GROUND SURFACE, not the road. Bridges and
    tunnels are the obvious cases, but embankments and cuttings lie the same
    way: an embankment carries the road level across a gully the DEM dives
    into, and a cutting carries it below a ridge the DEM climbs over. All are
    levelled with a straight grade between the ground heights at their ends.
    """
    for key in ("bridge", "tunnel", "embankment", "cutting"):
        v = str(tags.get(key, "no")).strip().lower()
        if v and v not in ("no", "false", "0"):
            return 1
    if str(tags.get("covered", "no")).lower() in ("yes", "true"):
        return 1
    return 0


_GEOM_CACHE_DIR = os.path.join(OUTPUT_DIR, "geometry")
_GEOM_CACHE_VER = 2      # bump when _struct_flag or stitching logic changes
_geom_mem = {}


def _geom_cached(osm_type, osm_id):
    key = f"{osm_type}_{osm_id}_v{_GEOM_CACHE_VER}"
    if key in _geom_mem:
        return _geom_mem[key]
    path = os.path.join(_GEOM_CACHE_DIR, f"{key}.json")
    if os.path.exists(path):
        try:
            with open(path) as f:
                data = json.load(f)
            _geom_mem[key] = data
            return data
        except Exception:
            pass
    return None


def _geom_store(osm_type, osm_id, data):
    key = f"{osm_type}_{osm_id}_v{_GEOM_CACHE_VER}"
    _geom_mem[key] = data
    try:
        os.makedirs(_GEOM_CACHE_DIR, exist_ok=True)
        with open(os.path.join(_GEOM_CACHE_DIR, f"{key}.json"), 'w') as f:
            json.dump(data, f)
    except Exception:
        pass


def fetch_road_geometry(osm_type: str, osm_id: str) -> dict:
    """
    Fetch the FULL road, not just the selected OSM way. Roads are split
    into many ways in OSM; we read the selected way's name/ref, gather all
    same-named ways in the surrounding area, and stitch them into one
    continuous route.
    """
    hit = _geom_cached(osm_type, osm_id)
    if hit is not None:
        print(f"  [geometry] cached: {hit.get('count', 0)} points "
              f"({hit.get('segments_used')}/{hit.get('segments_total')} segments)")
        return hit

    try:
        if osm_type == "way":
            data = _overpass(f"[out:json];way({osm_id});out geom;", timeout=20)
        elif osm_type == "relation":
            data = _overpass(f"[out:json];relation({osm_id});way(r);out geom;", timeout=20)
        else:
            return {"error": "Unsupported OSM type", "coords": []}
    except Exception as e:
        return {"error": str(e), "coords": []}

    elements = [el for el in data.get("elements", [])
                if el.get("type") == "way" and "geometry" in el]
    if not elements:
        return {"error": "No geometry found", "coords": []}

    segments = [[[g["lat"], g["lon"], _struct_flag(el.get("tags", {}))]
                 for g in el["geometry"]]
                for el in elements]
    seed_idx = 0
    # The clicked way's classification — drives the road surface texture
    seed_tags = elements[0].get("tags", {}) or {}

    # ── Name expansion (single way only — relations already give members)
    if osm_type == "way":
        tags = elements[0].get("tags", {})
        name = tags.get("name") or tags.get("ref")
        key  = "name" if tags.get("name") else "ref"
        if name:
            # bbox of the selected way, padded ~0.25° (~25 km)
            lats = [g["lat"] for g in elements[0]["geometry"]]
            lons = [g["lon"] for g in elements[0]["geometry"]]
            pad = 0.25
            bbox = (min(lats)-pad, min(lons)-pad, max(lats)+pad, max(lons)+pad)
            esc = name.replace('\\', '\\\\').replace('"', '\\"')
            q = (f"[out:json][timeout:30];"
                 f"way[\"{key}\"=\"{esc}\"][\"highway\"]"
                 f"({bbox[0]:.4f},{bbox[1]:.4f},{bbox[2]:.4f},{bbox[3]:.4f});"
                 f"out geom;")
            try:
                more = _overpass(q, timeout=35)
                more_els = [el for el in more.get("elements", [])
                            if el.get("type") == "way" and "geometry" in el]
                if len(more_els) > len(elements):
                    seed_geom = segments[0]
                    segments = [[[g["lat"], g["lon"],
                                  _struct_flag(el.get("tags", {}))]
                                 for g in el["geometry"]]
                                for el in more_els]
                    # seed = the originally selected way (match by first point)
                    seed_idx = 0
                    for i, s in enumerate(segments):
                        if s[0] == seed_geom[0] and s[-1] == seed_geom[-1]:
                            seed_idx = i
                            break
                    print(f"  [geometry] '{name}': expanded from 1 to "
                          f"{len(segments)} way segments")
            except Exception as e:
                print(f"  [geometry] name expansion failed ({e}) — "
                      f"using selected way only")

    coords, used = _stitch_segments(segments, seed_idx)
    if len(coords) < 2:
        return {"error": "No geometry found", "coords": []}
    print(f"  [geometry] stitched {used}/{len(segments)} segments, "
          f"{len(coords)} points")
    out = {"coords": coords, "count": len(coords),
           "segments_used": used, "segments_total": len(segments),
           "road_type": seed_tags.get("highway"),
           "road_surface": seed_tags.get("surface")}
    _geom_store(osm_type, osm_id, out)
    return out


def _haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin((lat2-lat1)*math.pi/360)**2 +
         math.cos(phi1)*math.cos(phi2)*math.sin((lon2-lon1)*math.pi/360)**2)
    return 2 * R * math.asin(math.sqrt(a))


# ─── Surroundings (buildings + trees from OSM, free via Overpass) ─────────────

def fetch_surroundings(coords: list, radius_m: float = 175.0) -> dict:
    """
    Fetch buildings and trees within radius_m of the route from Overpass.
    Returns {"buildings": [{"outline": [[lat,lon],...], "height": m}],
             "trees": [[lat, lon], ...]}  (empty lists on failure).
    """
    out = {"buildings": [], "trees": [], "forests": [], "roads": [],
           "waterways": [], "barriers": [], "signs": []}
    if len(coords) < 2:
        return out

    # ATLAS V8: bounded, individually cached OSM requests.  A single 20 km
    # around-polyline query times out on shared Overpass instances.
    import hashlib
    cache_dir = os.path.join(OUTPUT_DIR, "surroundings", "tiles_v10")
    os.makedirs(cache_dir, exist_ok=True)
    complete = True
    seen_query_elements = {}
    # Walk the route at <= 180 m intervals; pad each bounding box so that
    # roads/buildings next to the selected line are included.
    sample = [coords[0][:2]]
    for a, b in zip(coords, coords[1:]):
        steps = max(1, math.ceil(_haversine(a[0], a[1], b[0], b[1]) / 180.0))
        for j in range(1, steps + 1):
            t = j / steps
            sample.append([a[0] + (b[0]-a[0])*t, a[1] + (b[1]-a[1])*t])
    # Deduplicate overlapping query areas using rounded geographic tiles.
    boxes = []
    for lat, lon in sample:
        dlat = (radius_m + 50) / 111320.0
        dlon = (radius_m + 50) / (111320.0 * max(0.2, math.cos(math.radians(lat))))
        boxes.append((lat-dlat, lon-dlon, lat+dlat, lon+dlon))
    # Small bounding boxes outperform large around(polyline) queries.
    failed = 0
    EXPORT_PROGRESS.update(stage="Downloading OSM scenery", done=0, total=len(boxes), failed=0)
    for idx, (south, west, north, east) in enumerate(boxes, 1):
        bb = f"{south:.5f},{west:.5f},{north:.5f},{east:.5f}"
        key = hashlib.sha256(("atlas-v10:"+bb).encode()).hexdigest()[:24]
        tile_file = os.path.join(cache_dir, key + ".json")
        tile_data = None
        if os.path.isfile(tile_file):
            try:
                with open(tile_file, encoding="utf8") as f:
                    tile_data = json.load(f)
            except (ValueError, OSError):
                pass
        if tile_data is None:
            q = (f'[out:json][timeout:35];('
                 f'way["building"]({bb});'
                 f'way["building:part"]({bb});'
                 f'relation["building"]({bb});'
                 f'way["highway"]({bb});'
                 f'way["waterway"]({bb});'
                 f'way["barrier"]({bb});'
                 f'node["natural"="tree"]({bb});'
                 f'node["highway"~"stop|give_way|traffic_signals"]({bb});'
                 f'node["traffic_sign"]({bb});'
                 f'way["natural"~"wood|scrub|tree_row"]({bb});'
                 f'way["landuse"~"forest|orchard"]({bb});'
                 f');out geom;')
            try:
                tile_data = _overpass(q, timeout=48)
                if not isinstance(tile_data.get("elements"), list):
                    raise ValueError("Missing Overpass elements")
                temp_file = tile_file + ".tmp"
                with open(temp_file, "w", encoding="utf8") as f:
                    json.dump(tile_data, f)
                os.replace(temp_file, tile_file)
                time.sleep(0.3)  # shared API; avoid flooding it
            except Exception as exc:
                complete = False
                failed += 1
                EXPORT_PROGRESS.update(done=idx, failed=failed)
                print(f"  [surroundings] tile {idx}/{len(boxes)} failed: {exc}")
                continue
        for element in tile_data.get("elements", []):
            eid = (element.get("type"), element.get("id"))
            if eid not in seen_query_elements:
                seen_query_elements[eid] = element
        EXPORT_PROGRESS.update(done=idx)
        print(f"  [surroundings] tile {idx}/{len(boxes)}: {len(seen_query_elements)} unique OSM elements", flush=True)
    if not seen_query_elements:
        raise IOError("No scenery tiles could be downloaded. Try again when Overpass is available; successfully fetched tiles are cached for the next attempt.")
    data = {"elements": list(seen_query_elements.values())}
    # Cache individual tiles rather than caching partial whole-route responses.
    cache_path = None

    def veg_density(tags):
        """1.0 = dense woodland, lower = sparser scatter."""
        if tags.get("natural") in ("wood",) or tags.get("landuse") == "forest":
            return 1.0
        if tags.get("natural") in ("scrub", "heath") or tags.get("landuse") == "orchard":
            return 0.5
        if tags.get("leisure") == "nature_reserve" or "boundary" in tags:
            return 0.35     # parks/reserves: sparse bush scatter
        return 0.0

    seen = set()

    def parse_elements(elements):
        for el in elements:
            eid = (el.get("type"), el.get("id"))
            if eid in seen:
                continue
            seen.add(eid)
            tags = el.get("tags", {})
            if el.get("type") == "way" and "geometry" in el:
                outline = [[g["lat"], g["lon"]] for g in el["geometry"] if g]
                if ("building" in tags or "building:part" in tags) and tags.get("building", tags.get("building:part")) != "no":
                    # height: explicit metres > levels×3 > default 5m
                    h = 5.0
                    try:
                        if "height" in tags:
                            h = float(str(tags["height"]).replace("m", "").strip())
                        elif "building:levels" in tags:
                            h = float(tags["building:levels"]) * 3.0
                    except ValueError:
                        pass
                    h = max(2.5, min(h, 60.0))
                    if len(outline) >= 3:
                        out["buildings"].append({"outline": outline, "height": h,
                                                 "kind": tags.get("building", tags.get("building:part")), "osm_id": el.get("id"), "part": "building:part" in tags, "tags": tags})
                elif "highway" in tags:
                    # Adjacent roads — the arnis approach: render every
                    # highway the query returns, draped on the terrain.
                    # Foot-scale ways are skipped; they read as noise at
                    # driving speed.
                    htype = tags["highway"]
                    if htype in ("footway", "path", "steps", "pedestrian",
                                 "cycleway", "bridleway", "corridor"):
                        pass
                    elif len(outline) >= 2:
                        out["roads"].append({"pts": outline, "type": htype, "tags": tags, "junction": tags.get("junction"), "osm_id": el.get("id")})
                elif "waterway" in tags:
                    if len(outline) >= 2:
                        out["waterways"].append({"pts": outline,
                                                 "type": tags["waterway"]})
                elif "barrier" in tags:
                    if len(outline) >= 2:
                        out["barriers"].append({"pts": outline,
                                                "type": tags["barrier"]})
                elif tags.get("natural") == "tree_row":
                    # plant a tree every ~8m along the row
                    for j in range(len(outline) - 1):
                        a, b = outline[j], outline[j+1]
                        d = _haversine(a[0], a[1], b[0], b[1])
                        n = max(1, int(d / 8))
                        for k in range(n):
                            t = k / n
                            out["trees"].append([a[0] + (b[0]-a[0])*t,
                                                 a[1] + (b[1]-a[1])*t])
                else:
                    dns = veg_density(tags)
                    if dns > 0 and len(outline) >= 3:
                        out["forests"].append({"outline": outline,
                                               "density": dns})
            elif el.get("type") == "relation" and "members" in el:
                if "building" in tags and tags.get("building") != "no":
                    from shapely.geometry import LineString, Polygon
                    from shapely.ops import linemerge
                    def rings(role):
                        lines = [LineString([(g["lat"], g["lon"]) for g in m["geometry"] if g])
                                 for m in el["members"] if m.get("type") == "way"
                                 and m.get("role", "outer") == role and len(m.get("geometry", [])) >= 2]
                        if not lines:
                            return []
                        merged = linemerge(lines)
                        pieces = [merged] if merged.geom_type == "LineString" else list(merged.geoms)
                        return [list(line.coords) for line in pieces if line.is_ring]
                    inner = rings("inner")
                    for outer in rings("outer"):
                        holes = [ring for ring in inner if Polygon(outer).covers(Polygon(ring))]
                        try:
                            height = float(str(tags["height"]).replace("m", "").strip()) if "height" in tags else float(tags.get("building:levels", 2)) * 3.0
                        except (TypeError, ValueError):
                            height = 6.0
                        out["buildings"].append({"outline": outer, "holes": holes, "height": height,
                                                 "kind": tags.get("building"), "osm_id": el.get("id")})
                    # Member outlines belong to the relation, not standalone blocks.
                    member_ids = {m.get("ref") for m in el["members"] if m.get("type") == "way"}
                    out["buildings"] = [b for b in out["buildings"] if b.get("osm_id") not in member_ids]
                # multipolygon vegetation (national parks, big forests) —
                # each outer member ring becomes a scatter polygon
                dns = veg_density(tags)
                if dns > 0:
                    for m in el["members"]:
                        if m.get("type") == "way" and "geometry" in m \
                           and m.get("role") in ("outer", ""):
                            outline = [[g["lat"], g["lon"]]
                                       for g in m["geometry"] if g]
                            if len(outline) >= 3:
                                out["forests"].append({"outline": outline,
                                                       "density": dns})
            elif el.get("type") == "node" and "lat" in el:
                if tags.get("natural") == "tree":
                    out["trees"].append([el["lat"], el["lon"]])
                if tags.get("highway") in ("stop", "give_way", "traffic_signals") or tags.get("traffic_sign"):
                    out["signs"].append({"pt": [el["lat"], el["lon"]], "tags": tags})

    parse_elements(data.get("elements", []))

    # Vegetation is fetched by bounded tile queries above.

    # Road and building output is chunked by the KN5 writer, not silently capped.
    out["trees"] = out["trees"][:8000]
    out["forests"] = out["forests"][:300]
    out["waterways"] = out["waterways"][:200]
    out["barriers"] = out["barriers"][:300]
    dens = [f["density"] for f in out["forests"]]
    print(f"  [surroundings] {len(out['buildings'])} buildings, "
          f"{len(out['trees'])} tree nodes, {len(out['roads'])} nearby roads, "
          f"{len(out['waterways'])} waterways, {len(out['barriers'])} barriers, "
          f"{len(out['forests'])} vegetation "
          f"areas (max density {max(dens) if dens else 0})")
    if not out["roads"]:
        # The drivable road itself lies inside the query radius, so a valid
        # result can never contain zero roads — a regional or overloaded
        # mirror answered with an empty 200. Fail loudly, never cache.
        raise IOError("surroundings response was empty (bad Overpass "
                      "mirror?) — try the export again")
    if failed:
        raise IOError(f"{failed}/{len(boxes)} OSM tiles unavailable; refusing an incomplete road/building export. Retry to reuse successful cached tiles.")
    return out


def _gable_frame(pts: list):
    """
    Oriented frame for a roughly rectangular footprint: centre, unit axes
    (u along the long side), and half-extents. Returns None when the
    footprint isn't rectangular enough for a clean gable (polygon area under
    82% of its oriented bounding box — an L-shape with a modest wing sits
    around 75%) — those keep a flat roof.
    """
    P = np.array(pts, dtype=float)
    n = len(P)
    e = P[(np.arange(n) + 1) % n] - P
    L = np.hypot(e[:, 0], e[:, 1])
    k = int(np.argmax(L))
    if L[k] < 1e-6:
        return None
    ux, uz = e[k] / L[k]
    area = abs(sum(P[i, 0] * P[(i+1) % n, 1] - P[(i+1) % n, 0] * P[i, 1]
                   for i in range(n))) / 2.0
    a = P @ np.array([ux, uz])
    b = P @ np.array([-uz, ux])
    hl = (a.max() - a.min()) / 2.0
    hw = (b.max() - b.min()) / 2.0
    if hl < 1e-6 or hw < 1e-6 or area < 0.82 * (4.0 * hl * hw):
        return None
    if hw > hl:                            # ridge always along the long axis
        ux, uz, hl, hw = -uz, ux, hw, hl
        a, b = P @ np.array([ux, uz]), P @ np.array([-uz, ux])
        hl = (a.max() - a.min()) / 2.0
        hw = (b.max() - b.min()) / 2.0
    cx = (a.max() + a.min()) / 2.0 * ux + (b.max() + b.min()) / 2.0 * -uz
    cz = (a.max() + a.min()) / 2.0 * uz + (b.max() + b.min()) / 2.0 * ux
    return {"cx": cx, "cz": cz, "ux": ux, "uz": uz, "hl": hl, "hw": hw}


def _ear_clip(pts: list) -> list:
    """
    Triangulate a simple polygon [(x, z), ...] by ear clipping. A triangle
    fan — the previous approach — folds over itself on any concave footprint
    (L-shaped houses, courtyards), which made most roofs look broken.
    Returns [(i, j, k), ...] index triples; falls back to a fan if the
    outline is degenerate/self-intersecting.
    """
    n = len(pts)
    if n < 3:
        return []

    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

    area = sum(pts[i][0]*pts[(i+1) % n][1] - pts[(i+1) % n][0]*pts[i][1]
               for i in range(n))
    idx = list(range(n))
    if area < 0:
        idx.reverse()

    def in_tri(p, a, b, c):
        return (cross(a, b, p) >= -1e-9 and cross(b, c, p) >= -1e-9
                and cross(c, a, p) >= -1e-9)

    tris = []
    guard = 0
    while len(idx) > 3 and guard < 10 * n:
        guard += 1
        clipped = False
        for ii in range(len(idx)):
            i0, i1, i2 = idx[ii-1], idx[ii], idx[(ii+1) % len(idx)]
            a, b, c = pts[i0], pts[i1], pts[i2]
            if cross(a, b, c) <= 1e-12:
                continue                       # reflex or collinear corner
            if any(in_tri(pts[j], a, b, c)
                   for j in idx if j not in (i0, i1, i2)):
                continue                       # another vertex inside: not an ear
            tris.append((i0, i1, i2))
            idx.pop(ii)
            clipped = True
            break
        if not clipped:
            break
    if len(idx) == 3:
        tris.append((idx[0], idx[1], idx[2]))
    elif not tris:
        tris = [(0, i, i+1) for i in range(1, n-1)]   # degenerate: fan fallback
    return tris



SIDE_ROAD_W = {"motorway": 11.0, "trunk": 10.0, "primary": 8.0,
               "secondary": 7.0, "tertiary": 6.5, "residential": 5.5,
               "unclassified": 5.5, "living_street": 5.0, "service": 4.0,
               "track": 3.0}
WATER_W   = {"river": 8.0, "canal": 6.0, "stream": 2.5,
             "drain": 1.5, "ditch": 1.2}


def build_environment_meshes(surroundings: dict, mesh: dict,
                             ground_pts: list = None) -> dict:
    """
    Convert OSM buildings/trees into KN5 mesh tuples in the track's local
    coordinate space. Returns [(name, kn5_verts, indices, material_key)].
    Barriers use physical WALL meshes; scenery remains visual only.
    """
    if not surroundings:
        return {"meshes": [], "n_buildings": 0, "n_trees": 0}
    proj_p = mesh["proj"]
    proj = Transformer.from_crs(
        "EPSG:4326",
        f"+proj=tmerc +lat_0={proj_p['mid_lat']} +lon_0={proj_p['mid_lon']} +units=m",
        always_xy=True)
    ox, oz = proj_p["ox"], proj_p["oz"]

    # Ground height lookup. With real terrain: nearest terrain/road vertex.
    # Without: nearest centerline point + synthetic lateral drop.
    from scipy.spatial import cKDTree
    cl = mesh["centerline"]
    hw = mesh["stats"]["road_width"] / 2.0
    kd_road = cKDTree(np.array([[p[0], p[2]] for p in cl]))
    if ground_pts:
        g_xz = np.array([[p[0], p[2]] for p in ground_pts])
        g_y  = np.array([p[1] for p in ground_pts])
        kd = cKDTree(g_xz)

        def ground_y(x, z):
            _, i = kd.query([x, z])
            return float(g_y[i])

        def ground_y_smooth(x, z):
            # Inverse-distance blend of the 4 nearest ground vertices: the
            # terrain grid is 12m, so nearest-vertex heights step visibly
            # along a draped strip; blending removes the stair-stepping.
            d, i = kd.query([x, z], k=4)
            w = 1.0 / (np.asarray(d) + 0.5)
            return float(np.sum(g_y[np.asarray(i)] * w) / np.sum(w))

        # True rendered-surface height. The terrain mesh is a Delaunay
        # triangulation of these same points, so linear interpolation over
        # their Delaunay reproduces the drawn surface almost exactly — unlike
        # the vertex-bound lookups above, which over/undershoot by the local
        # relief on slopes (max-bounds left draped strips floating a metre
        # above hillsides; min-bounds still missed cliff-edge triangles).
        from scipy.interpolate import LinearNDInterpolator
        try:
            _lin = LinearNDInterpolator(g_xz, g_y)
        except Exception:
            _lin = None

        def ground_surf(x, z):
            if _lin is not None:
                v = _lin(x, z)
                if v == v:                      # inside the hull (not NaN)
                    return float(v)
            return ground_y_smooth(x, z)
    else:
        cl_xz = np.array([[p[0], p[2]] for p in cl])
        cl_y  = np.array([p[1] for p in cl])
        kd = cKDTree(cl_xz)

        def ground_y(x, z):
            d, i = kd.query([x, z])
            return float(cl_y[i]) - ground_drop(float(d) - hw)

        ground_y_smooth = ground_y
        ground_surf = ground_y

    def to_local(lat, lon):
        X, Z = proj.transform(lon, lat)
        return X - ox, -Z - oz     # -Z: same handedness flip as the road

    # ── Scatter trees inside vegetation polygons ──
    # OSM forests are areas, not tree nodes — walk the road, throw random
    # points into the corridor, keep ones that land inside vegetation,
    # accepting with the area's density (woods dense, reserves sparse).
    tree_pts = [to_local(la, lo) for la, lo in surroundings.get("trees", [])]

    forests = []
    for f in surroundings.get("forests", []):
        outline = f["outline"] if isinstance(f, dict) else f
        density = f.get("density", 1.0) if isinstance(f, dict) else 1.0
        poly = [to_local(la, lo) for la, lo in outline[::max(1, len(outline)//200)]]
        if len(poly) >= 3:
            pxs = [p[0] for p in poly]; pzs = [p[1] for p in poly]
            forests.append((poly, (min(pxs), min(pzs), max(pxs), max(pzs)),
                            density))

    def in_poly(x, z, poly):
        inside = False
        j = len(poly) - 1
        for i in range(len(poly)):
            xi, zi = poly[i]; xj, zj = poly[j]
            if (zi > z) != (zj > z) and \
               x < (xj - xi) * (z - zi) / ((zj - zi) or 1e-12) + xi:
                inside = not inside
            j = i
        return inside

    lc_fn = mesh.get("land_cover")
    lc_latlon = mesh.get("to_latlon")
    if lc_latlon is None:
        lc_fn = None

    # Cap scales with track length — a fixed cap left the far end of long
    # tracks bare once it filled up near the start. Stations are also
    # visited in SHUFFLED order, so if the cap is ever hit the thinning is
    # uniform along the whole road rather than front-loaded.
    # These are NOT engine limits — the KN5 65535-verts-per-mesh cap is
    # already handled by chunking. They bound file size and build time
    # (~17 verts/tree incl. shadow → 30k trees ≈ 510k verts ≈ ~22MB).
    tree_cap = min(30000, max(6000,
                   int(mesh["stats"].get("length_km", 5.0) * 1600)))
    if (forests or lc_fn) and len(tree_pts) < tree_cap:
        rng = np.random.default_rng(99)
        min_lat = hw + 4.0          # keep off road and verge
        max_lat = TERRAIN_MAX_DIST - 5.0   # plant right out to the terrain edge
        step = 6                     # try planting every 6m of road
        for ci in rng.permutation(np.arange(0, len(cl), step)):
            if len(tree_pts) >= tree_cap:
                break
            p = cl[ci]
            pn = cl[min(ci + 1, len(cl) - 1)]
            dx, dz = pn[0] - p[0], pn[2] - p[2]
            L = math.hypot(dx, dz) or 1.0
            px_, pz_ = -dz / L, dx / L     # perpendicular
            for side in (+1, -1):
                for _ in range(4):
                    lat_d = rng.uniform(min_lat, max_lat)
                    along = rng.uniform(-6, 6)
                    x = p[0] + side * px_ * lat_d + dx / L * along
                    z = p[2] + side * pz_ * lat_d + dz / L * along
                    placed = False
                    for poly, bb, density in forests:
                        if bb[0] <= x <= bb[2] and bb[1] <= z <= bb[3] \
                           and rng.random() < density \
                           and in_poly(x, z, poly):
                            placed = True
                            break
                    # No OSM polygon here — ask WorldCover what the ground
                    # actually is. This is what fills in rural roads that
                    # nobody has mapped vegetation for.
                    if not placed and lc_fn is not None:
                        la, lo = lc_latlon(x, z)
                        dns = LC_TREE_DENSITY.get(lc_fn(la, lo), 0.0)
                        placed = dns > 0 and rng.random() < dns
                    if placed:
                        tree_pts.append((x, z))
                        # Clumping: uniform scatter reads as "sprinkled";
                        # real woodland grows in clusters. Satellites spread
                        # only AWAY from the road; the clearance pass below
                        # still backstops everything.
                        if lat_d > min_lat + 8.0 and rng.random() < 0.55:
                            for _s in range(int(rng.integers(1, 4))):
                                s_lat = rng.uniform(0.0, 6.0)
                                s_alg = rng.uniform(-7.0, 7.0)
                                tree_pts.append((
                                    x + side * px_ * s_lat + dx / L * s_alg,
                                    z + side * pz_ * s_lat + dz / L * s_alg))

    # ── Road clearance ──
    # Nothing may stand on the carriageway: OSM tree nodes are often mapped
    # a few metres off, road smoothing shifts the road from its GPS line, and
    # the scatter's along-road jitter can curl into the road on bends. Every
    # tree is checked against its true distance to the centreline.
    if tree_pts:
        d_road, _ = kd_road.query(np.array(tree_pts))
        before = len(tree_pts)
        if mesh.get("network_surface") is not None:
            from shapely.geometry import Point
            clearance = mesh["network_surface"].buffer(2.0)
            tree_pts = [t for t in tree_pts if not clearance.covers(Point(t))]
        else:
            tree_pts = [t for t, dd in zip(tree_pts, d_road) if dd > hw + 2.0]
        if len(tree_pts) < before:
            print(f"  [surroundings] removed {before - len(tree_pts)} tree(s) "
                  f"standing on/beside the carriageway")

    meshes = []

    # ── Draped linear features: adjacent roads, waterways, barriers ──
    # (the arnis approach: render every linear feature the query returns.)
    # Barriers are physical WALL meshes, clipped clear of all carriageways.
    BARRIER_H = {"fence": 1.2, "wall": 1.8, "hedge": 1.5,
                 "guard_rail": 0.75, "retaining_wall": 2.0}
    BARRIER_MAT = {"hedge": "tree", "guard_rail": "asphalt"}  # others: building

    # Ground-to-top, two-sided physical barriers follow terrain and are
    # clipped against the complete road surface, including side roads.
    barr = {}          # mat -> [verts, idx, part]

    def flush_barrier(mat):
        verts_b, idx_b, part_b = barr[mat]
        if verts_b:
            meshes.append((f"1WALL_BARR_{mat.upper()}_{part_b}", verts_b, idx_b,
                           mat))
            barr[mat] = [[], [], part_b + 1]

    def _barrier_runs(pts_ll):
        from shapely.geometry import LineString
        loc = [to_local(la, lo) for la, lo in pts_ll]
        if len(loc) < 2:
            return
        line = LineString(loc)
        surface = mesh.get("network_surface")
        if surface is None:
            surface = LineString([(p[0], p[2]) for p in cl]).buffer(hw)
        clear = line.difference(surface.buffer(0.15))
        runs = [clear] if clear.geom_type == "LineString" else getattr(clear, "geoms", [])
        for run in runs:
            if run.geom_type != "LineString" or run.length < 0.05:
                continue
            # Retain every original bend, then subdivide each segment.
            points = []
            coords = list(run.coords)
            for a, b in zip(coords, coords[1:]):
                count = max(1, int(np.ceil(np.linalg.norm(np.subtract(b, a)) / 3.0)))
                points.extend(np.asarray(a) + (np.asarray(b)-a)*k/count for k in range(count))
            points.append(np.asarray(coords[-1]))
            points = np.asarray(points)
            distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
            yield points[:,0], points[:,1], distances, None, None

    n_barr = 0
    for br in surroundings.get("barriers", []):
        btype = br.get("type")
        bh = BARRIER_H.get(btype, 1.2)
        mat = BARRIER_MAT.get(btype, "building")
        rail = (btype == "guard_rail")
        if mat not in barr:
            barr[mat] = [[], [], 0]
        for rx_, rz_, rt_, _p, _n in _barrier_runs(br["pts"]):
            verts_b, idx_b, _ = barr[mat]
            if len(verts_b) + 6 * len(rx_) > 60000:
                flush_barrier(mat)
                verts_b, idx_b, _ = barr[mat]
            dxs_ = np.gradient(rx_); dzs_ = np.gradient(rz_)
            L = np.sqrt(dxs_**2 + dzs_**2) + 1e-9
            px_, pz_ = -dzs_ / L, dxs_ / L
            ux_, uz_ = dxs_ / L, dzs_ / L
            gys = [ground_y_smooth(float(rx_[k]), float(rz_[k]))
                   for k in range(len(rx_))]
            # A solid ground-to-top ribbon reads as a wall. A real armco is a
            # floating W-beam band on posts, so guard rails get exactly that:
            # band 0.45-0.80m above ground, plus a post at every sample (3m
            # spacing) reaching into the ground.
            lo = 0.45 if rail else -0.2
            hi = 0.80 if rail else bh
            v0 = len(verts_b)
            for k in range(len(rx_)):
                x_, z_ = float(rx_[k]), float(rz_[k])
                gy = gys[k]
                u = float(rt_[k]) * 0.25
                nx_, nz_ = float(px_[k]), float(pz_[k])
                verts_b.append(((x_, gy + lo, z_), (nx_, 0, nz_), (u, 0), (1, 0, 0)))
                verts_b.append(((x_, gy + hi, z_), (nx_, 0, nz_), (u, 1), (1, 0, 0)))
            for k in range(len(rx_) - 1):
                a = v0 + k*2
                idx_b.extend((a, a+2, a+3,  a, a+3, a+1))
                idx_b.extend((a, a+3, a+2,  a, a+1, a+3))
            if rail:
                for k in range(len(rx_)):
                    x_, z_ = float(rx_[k]), float(rz_[k])
                    gy = gys[k]
                    ax_, az_ = float(ux_[k]) * 0.07, float(uz_[k]) * 0.07
                    nx_, nz_ = float(px_[k]), float(pz_[k])
                    p0 = len(verts_b)
                    verts_b.append(((x_ - ax_, gy - 0.10, z_ - az_), (nx_, 0, nz_), (0, 0), (1, 0, 0)))
                    verts_b.append(((x_ + ax_, gy - 0.10, z_ + az_), (nx_, 0, nz_), (1, 0), (1, 0, 0)))
                    verts_b.append(((x_ + ax_, gy + 0.72, z_ + az_), (nx_, 0, nz_), (1, 1), (1, 0, 0)))
                    verts_b.append(((x_ - ax_, gy + 0.72, z_ - az_), (nx_, 0, nz_), (0, 1), (1, 0, 0)))
                    idx_b.extend((p0, p0+1, p0+2,  p0, p0+2, p0+3))
                    idx_b.extend((p0, p0+2, p0+1,  p0, p0+3, p0+2))
            n_barr += 1
    for mat in list(barr):
        flush_barrier(mat)

    if n_barr:
        print(f"  [surroundings] {n_barr} barrier strip(s) "
              f"(side roads/waterways are painted into the terrain)")

    # ── Trees: alpha-cutout cross-plane impostors ──
    # Three quads at 60° sampling one column of a 4-variant silhouette atlas
    # (repeating a single silhouette is what read as fake). Each tree also
    # lays a soft alpha-blended shadow blob on the ground — real-time shadows
    # fade with distance, and the baked blob is what keeps trees anchored.
    cans = {"tree": [[], [], 0], "tree2": [[], [], 0]}
    shadow = [[], [], 0]

    def flush_tree_meshes():
        for mat in cans:
            cv, ci, cp = cans[mat]
            if cv:
                suffix = "" if mat == "tree" else "2"
                meshes.append((f"ENV_TREE{suffix}_{cp}", cv, ci, mat))
                cans[mat] = [[], [], cp + 1]
        sv, si, sp = shadow
        if sv:
            meshes.append((f"ENV_SHADOW_{sp}", sv, si, "treeshadow"))
            shadow[0], shadow[1], shadow[2] = [], [], sp + 1

    rng_t = np.random.default_rng(7)
    for x, z in tree_pts:
        gy = ground_surf(x, z)             # true rendered-terrain height
        gy_base = gy - 0.5                 # quad foot: sunk into the surface
        var = int(rng_t.integers(0, 4))    # atlas column
        if rng_t.random() < 0.45:          # conifer
            h = rng_t.uniform(8.0, 14.0)
            w = h * TREE2_ASPECT[var]
            mat = "tree2"
        else:                              # broadleaf (incl. gum variant)
            h = rng_t.uniform(6.5, 11.5)
            w = h * TREE_ASPECT[var]
            mat = "tree"
        u0, u1 = var * 0.25, var * 0.25 + 0.25
        yaw = rng_t.uniform(0.0, math.pi)
        cv, ci, _ = cans[mat]
        if len(cv) + 12 > 60000 or len(shadow[0]) + 5 > 60000:
            flush_tree_meshes()
            cv, ci, _ = cans[mat]
        top = gy + h
        # Up normals: cross-planes lit like the ground, so no plane goes
        # black when the sun is behind it.
        for pi in range(3):
            a = yaw + pi * math.pi / 3.0
            dx_ = math.cos(a) * w * 0.5
            dz_ = math.sin(a) * w * 0.5
            v0 = len(cv)
            # DirectX v=0 is the TOP of the image (= tree top), so the base
            # vertices carry v=1 and the top vertices v=0.
            cv.append(((x - dx_, gy_base, z - dz_), (0, 1, 0), (u0, 1), (1, 0, 0)))
            cv.append(((x + dx_, gy_base, z + dz_), (0, 1, 0), (u1, 1), (1, 0, 0)))
            cv.append(((x + dx_, top,     z + dz_), (0, 1, 0), (u1, 0), (1, 0, 0)))
            cv.append(((x - dx_, top,     z - dz_), (0, 1, 0), (u0, 0), (1, 0, 0)))
            ci.extend((v0, v0+1, v0+2,  v0, v0+2, v0+3))
            ci.extend((v0, v0+2, v0+1,  v0, v0+3, v0+2))
        # Shadow blob: centre + 4 corners, each conformed to the terrain so
        # the blob hugs slopes instead of clipping into them.
        sr = w * 0.42
        s_yaw = rng_t.uniform(0.0, math.pi)
        elong = rng_t.uniform(1.0, 1.3)
        sv, si, _ = shadow
        c0 = len(sv)
        sv.append(((x, gy + 0.07, z), (0, 1, 0), (0.5, 0.5), (1, 0, 0)))
        for k4, (ux, uz) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1))):
            ox = (ux * math.cos(s_yaw) - uz * math.sin(s_yaw)) * sr * elong
            oz = (ux * math.sin(s_yaw) + uz * math.cos(s_yaw)) * sr
            sx, sz = x + ox, z + oz
            sv.append(((sx, ground_surf(sx, sz) + 0.07, sz), (0, 1, 0),
                       ((ux + 1) / 2.0, (uz + 1) / 2.0), (1, 0, 0)))
        for k4 in range(4):
            a_, b_ = c0 + 1 + k4, c0 + 1 + (k4 + 1) % 4
            si.extend((c0, a_, b_,  c0, b_, a_))   # double-sided fan
    flush_tree_meshes()

    return {"meshes": meshes, "n_trees": len(tree_pts),
            "n_buildings": 0}


# ─── Elevation ────────────────────────────────────────────────────────────────

# ─── Terrarium raster tiles (AWS Open Data — no key, no rate limit) ───────────
# The point-query elevation APIs are the bottleneck in this tool: opentopodata
# allows 100 points/call at 1 call/sec, so dense sampling is impossible. AWS
# Terrain Tiles serve DEM as ordinary PNG raster tiles instead — one 256×256
# tile carries 65,536 samples in a single request, cached on disk, after which
# sampling any point is free and local. Terrarium encodes height in the pixel:
#     elevation_m = R*256 + G + B/256 - 32768
# Tiles are standard slippy-map (Web Mercator) tiles.
# Dataset: https://registry.opendata.aws/terrain-tiles/  (approach per the
# Apache-2.0 project github.com/louis-e/arnis, implemented here from the spec).

TERRARIUM_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
TILE_CACHE_DIR = os.path.join(OUTPUT_DIR, "tiles")
TERRARIUM_MIN_ZOOM = 10
TERRARIUM_MAX_ZOOM = 15      # ~4.8 m/px — same max zoom as arnis; the finer
                             # sampling grid interpolates slopes more smoothly
                             # even where the source DEM is ~30 m
TERRARIUM_MAX_TILES = 256    # per export; zoom drops until the budget is met.
                             # Tiles are ~30 KB, so a generous budget keeps
                             # long roads at max zoom instead of degrading to
                             # coarse DEM (which smears valleys into the road).

_tile_mem = {}               # (z,x,y) -> 256×256 float array, or None if absent


def _tile_xy(lat, lon, zoom):
    """Fractional GLOBAL pixel coords (256px tiles) for a lat/lon."""
    n = 2.0 ** zoom
    fx = (lon + 180.0) / 360.0 * n * 256.0
    lat_r = math.radians(max(-85.05, min(85.05, lat)))
    fy = (1.0 - math.asinh(math.tan(lat_r)) / math.pi) / 2.0 * n * 256.0
    return fx, fy


def _pick_zoom(coords):
    """
    Highest zoom whose tile count fits the budget. Counts only the tiles the
    route actually touches, not its bounding box — a long diagonal road covers
    a huge bbox but a thin corridor, and counting the bbox would drop the zoom
    (and the terrain's accuracy) for no reason.
    """
    step = max(1, len(coords) // 400)
    sample = coords[::step] + [coords[-1]]
    for z in range(TERRARIUM_MAX_ZOOM, TERRARIUM_MIN_ZOOM - 1, -1):
        tiles = set()
        for c in sample:
            fx, fy = _tile_xy(c[0], c[1], z)
            tx, ty = int(fx // 256), int(fy // 256)
            # neighbours too: bilinear reads one pixel past a tile edge
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    tiles.add((tx + dx, ty + dy))
        if len(tiles) <= TERRARIUM_MAX_TILES:
            return z
    return TERRARIUM_MIN_ZOOM


def _load_tile(z, x, y):
    """Decoded 256×256 elevation array for a tile, or None. Disk + memory cached."""
    key = (z, x, y)
    if key in _tile_mem:
        return _tile_mem[key]

    try:
        from PIL import Image
    except ImportError:
        raise ElevationUnavailable(
            "Pillow is not installed — run: pip install pillow")

    os.makedirs(TILE_CACHE_DIR, exist_ok=True)
    path = os.path.join(TILE_CACHE_DIR, f"z{z}_x{x}_y{y}.png")

    if not os.path.exists(path):
        url = TERRARIUM_URL.format(z=z, x=x, y=y)
        data = None
        for attempt in (1, 2, 3):
            try:
                req = Request(url, headers={"User-Agent": "ATLAS Road World Studio/1.0"})
                with urlopen(req, timeout=20) as r:
                    data = r.read()
                break
            except Exception as e:
                if attempt == 3:
                    raise ElevationUnavailable(
                        f"could not download elevation tile {z}/{x}/{y} from "
                        f"AWS Terrain Tiles: {e}")
                time.sleep(0.5 * attempt)
        try:
            with open(path, 'wb') as f:
                f.write(data)
        except Exception:
            pass

    try:
        with Image.open(path) as im:
            arr = np.asarray(im.convert("RGB"), dtype=np.float64)
    except Exception as e:
        # corrupt tile: drop it so the next run refetches
        try:
            os.remove(path)
        except Exception:
            pass
        raise ElevationUnavailable(
            f"elevation tile {z}/{x}/{y} was corrupt ({e}) — it has been "
            f"deleted from the cache, try the export again")

    elev = arr[:, :, 0] * 256.0 + arr[:, :, 1] + arr[:, :, 2] / 256.0 - 32768.0
    _tile_mem[key] = elev
    return elev


def _tile_pixel(z, tx, ty, px, py):
    """One pixel, following tile boundary crossover. None if the tile is missing."""
    if px < 0:
        tx, px = tx - 1, px + 256
    elif px >= 256:
        tx, px = tx + 1, px - 256
    if py < 0:
        ty, py = ty - 1, py + 256
    elif py >= 256:
        ty, py = ty + 1, py - 256
    return float(_load_tile(z, tx, ty)[py, px])


def _prefetch_tiles(coords, z, workers=8):
    """Download every tile the sample points need, in parallel."""
    need = set()
    for c in coords:
        fx, fy = _tile_xy(c[0], c[1], z)
        tx, ty = int(fx // 256), int(fy // 256)
        for dx in (0, 1, -1):
            for dy in (0, 1, -1):
                need.add((tx + dx, ty + dy))
    todo = [t for t in need if (z,) + t not in _tile_mem]
    if not todo:
        return
    errors = []

    def grab(t):
        try:
            _load_tile(z, t[0], t[1])
        except ElevationUnavailable as e:
            errors.append(e)

    with ThreadPoolExecutor(max_workers=workers) as ex:
        list(ex.map(grab, todo))
    if errors:
        raise errors[0]


def _fetch_terrarium(coords):
    """
    Elevations from AWS Terrain Tiles, bilinearly interpolated.
    Raises ElevationUnavailable if the data cannot be obtained — there is no
    silent degradation to a worse source.
    """
    if not coords:
        return []
    z = _pick_zoom(coords)
    _prefetch_tiles(coords, z)
    out = []
    for c in coords:
        lat, lon = c[0], c[1]
        fx, fy = _tile_xy(lat, lon, z)
        tx, ty = int(fx // 256), int(fy // 256)
        px, py = fx - tx * 256.0, fy - ty * 256.0
        x0, y0 = int(math.floor(px)), int(math.floor(py))
        dx, dy = px - x0, py - y0
        v00 = _tile_pixel(z, tx, ty, x0,     y0)
        v10 = _tile_pixel(z, tx, ty, x0 + 1, y0)
        v01 = _tile_pixel(z, tx, ty, x0,     y0 + 1)
        v11 = _tile_pixel(z, tx, ty, x0 + 1, y0 + 1)
        top = v00 + (v10 - v00) * dx
        bot = v01 + (v11 - v01) * dx
        out.append(top + (bot - top) * dy)
    n_tiles = sum(1 for k in _tile_mem if k[0] == z)
    print(f"  [terrarium] {len(coords)} points from {n_tiles} tiles @ z{z}")
    return out


# ─── Local high-resolution DEMs (e.g. ELVIS LiDAR GeoTIFFs) ──────────────────
#
# Drop GeoTIFF DEM files (.tif) into the ./dem folder next to this script and
# they are preferred over the online tiles wherever they cover. Intended for
# Geoscience Australia ELVIS LiDAR DEMs (1m bare-earth, elevation.fsdf.org.au)
# but any georeferenced elevation GeoTIFF works — CRS is read from the file.
# Requires rasterio (pip install rasterio). Points outside local coverage fall
# back to the online tiles, datum-aligned so the seam is minimal.

LOCAL_DEM_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dem")
_local_dems = None
_local_dem_sig = None


def _load_local_dems(refresh: bool = False) -> list:
    """Cached list of loaded local DEMs, finest resolution first."""
    global _local_dems, _local_dem_sig
    if not os.path.isdir(LOCAL_DEM_DIR):
        _local_dems = []
        return _local_dems
    files = sorted(f for f in os.listdir(LOCAL_DEM_DIR)
                   if f.lower().endswith((".tif", ".tiff")))
    sig = tuple((f, os.path.getmtime(os.path.join(LOCAL_DEM_DIR, f)))
                for f in files)
    if _local_dems is not None and (not refresh or sig == _local_dem_sig):
        return _local_dems
    _local_dem_sig = sig
    _local_dems = []
    if not files:
        return _local_dems
    try:
        import rasterio
    except ImportError:
        print(f"  [dem] {len(files)} GeoTIFF(s) found in ./dem but rasterio "
              f"is not installed — run: pip install rasterio")
        return _local_dems
    for fn in files:
        path = os.path.join(LOCAL_DEM_DIR, fn)
        try:
            with rasterio.open(path) as ds:
                arr = ds.read(1).astype(np.float32)
                if ds.nodata is not None:
                    arr[arr == ds.nodata] = np.nan
                arr[arr < -1000] = np.nan          # common sentinel values
                tr = Transformer.from_crs("EPSG:4326", ds.crs,
                                          always_xy=True)
                res = max(abs(ds.transform.a), abs(ds.transform.e))
                # WGS84 bounds so the UI can draw the coverage on the map
                inv_tr = Transformer.from_crs(ds.crs, "EPSG:4326",
                                              always_xy=True)
                b = ds.bounds
                lons, lats = inv_tr.transform(
                    [b.left, b.right, b.right, b.left],
                    [b.top, b.top, b.bottom, b.bottom])
                bounds = [min(lats), min(lons), max(lats), max(lons)]
                _local_dems.append({"name": fn, "arr": arr, "tr": tr,
                                    "inv": ~ds.transform, "res": res,
                                    "w": ds.width, "h": ds.height,
                                    "bounds": bounds})
                print(f"  [dem] loaded {fn}: {ds.width}×{ds.height} px @ "
                      f"{res:.1f} m/px ({ds.crs})")
        except Exception as e:
            print(f"  [dem] could not load {fn}: {e}")
    _local_dems.sort(key=lambda d: d["res"])       # finest wins on overlap
    return _local_dems


def _sample_local_dem(lat: float, lon: float):
    """Bilinear sample from the finest local DEM covering the point, or None."""
    for d in _local_dems or []:
        try:
            x, y = d["tr"].transform(lon, lat)
            col, row = d["inv"] * (x, y)
        except Exception:
            continue
        c0 = int(math.floor(col - 0.5))
        r0 = int(math.floor(row - 0.5))
        if c0 < 0 or r0 < 0 or c0 + 1 >= d["w"] or r0 + 1 >= d["h"]:
            continue
        q = d["arr"][r0:r0 + 2, c0:c0 + 2]
        if np.isnan(q).any():
            continue
        fx = (col - 0.5) - c0
        fy = (row - 0.5) - r0
        return float(q[0, 0] * (1 - fx) * (1 - fy) + q[0, 1] * fx * (1 - fy)
                     + q[1, 0] * (1 - fx) * fy + q[1, 1] * fx * fy)
    return None


def _local_dem_coverage(coords: list) -> float:
    """Fraction of the given points covered by loaded local DEMs (0-1)."""
    if not _local_dems or not coords:
        return 0.0
    step = max(1, len(coords) // 400)
    pts = coords[::step]
    hit = sum(1 for c in pts if _sample_local_dem(c[0], c[1]) is not None)
    return hit / len(pts)


def dem_status() -> dict:
    """Elevation mode + loaded local DEMs, for the UI."""
    dems = _load_local_dems(refresh=True)
    return {
        "mode": "manual" if dems else "auto",
        "dems": [{"name": d["name"], "res_m": round(d["res"], 2),
                  "bounds": d["bounds"]} for d in dems],
    }


def fetch_elevations(coords: list, prefer_openmeteo: bool = False,
                     tiles_only: bool = True):
    """Elevation for a list of [lat, lon]. Prefers local GeoTIFF DEMs (./dem)
    where they cover; online tiles elsewhere, datum-aligned across the seam.
    Raises ElevationUnavailable."""
    dems = _load_local_dems(refresh=True)
    if not dems:
        return _fetch_terrarium(coords)

    vals = [_sample_local_dem(c[0], c[1]) for c in coords]
    missing = [i for i, v in enumerate(vals) if v is None]
    n_local = len(vals) - len(missing)
    if n_local == 0:
        return _fetch_terrarium(coords)
    if not missing:
        return [float(v) for v in vals]

    # Mixed coverage: fill gaps from the online tiles, shifted by the median
    # local-vs-tile difference over covered points so the vertical datums
    # (LiDAR AHD vs tile EGM96) don't create a step at the coverage boundary.
    covered = [i for i, v in enumerate(vals) if v is not None]
    probe = covered[::max(1, len(covered) // 150)]
    fill = _fetch_terrarium([coords[i] for i in missing]
                            + [coords[i] for i in probe])
    off = float(np.median([vals[i] - fill[len(missing) + k]
                           for k, i in enumerate(probe)]))
    for k, i in enumerate(missing):
        vals[i] = float(fill[k]) + off
    print(f"  [dem] local DEM covered {n_local}/{len(vals)} points; "
          f"{len(missing)} filled from online tiles "
          f"(datum offset {off:+.1f} m)")
    return vals


OPENMETEO_URL = "https://api.open-meteo.com/v1/elevation"


def fetch_elevations_copernicus(coords: list, max_pts: int = 300):
    """
    Independent second opinion on the elevation profile. Open-Meteo serves
    Copernicus GLO-90 — a radar DEM from a different satellite mission
    (TanDEM-X, ~2011-2015) than the SRTM-era data behind AWS Terrain Tiles —
    so its errors are independent: where both agree a feature is probably
    real terrain; where they disagree, at least one is an artefact.
    Returns (step, elevations) sampling coords[::step], or None on failure.
    """
    if len(coords) < 2:
        return None
    step = max(1, (len(coords) + max_pts - 1) // max_pts)
    pts = coords[::step]
    out = []
    for i in range(0, len(pts), 100):        # API takes ≤100 points per call
        chunk = pts[i:i + 100]
        lats = ",".join(f"{c[0]:.6f}" for c in chunk)
        lons = ",".join(f"{c[1]:.6f}" for c in chunk)
        req = Request(f"{OPENMETEO_URL}?latitude={lats}&longitude={lons}",
                      headers={"User-Agent": "ATLAS Road World Studio/1.0"})
        with urlopen(req, timeout=15) as r:
            data = json.loads(r.read())
        vals = data.get("elevation") or []
        if len(vals) != len(chunk):
            return None
        out.extend(float(v) for v in vals)
    return step, out


def _flatten_structures(elevs: list, flags: list) -> list:
    """
    Replace elevation across bridge and tunnel spans with a straight grade
    between the ground heights at each end. DEM measures the ground, so a
    bridge would otherwise plunge into the valley it crosses and a tunnel
    would climb over the mountain it bores through.
    """
    if not flags or max(flags) < 0.5:
        return elevs
    e = list(elevs)
    n = len(e)
    i = 0
    spans = 0
    while i < n:
        if flags[i] < 0.5:
            i += 1
            continue
        j = i
        while j < n and flags[j] >= 0.5:
            j += 1
        # anchor on the last/first solid ground either side of the span
        a, b = i - 1, j
        ya = e[a] if a >= 0 else (e[b] if b < n else e[i])
        yb = e[b] if b < n else ya
        span = (b - a) if (a >= 0 and b < n) else max(1, j - i)
        for k in range(i, j):
            t = (k - a) / span if span else 0.0
            e[k] = ya + (yb - ya) * t
        spans += 1
        i = j
    if spans:
        print(f"  [elevation] {spans} bridge/tunnel span(s) levelled")
    return e


def _despike_1d(e: list, win: int = 3, k: float = 4.0) -> list:
    """
    Median/MAD de-spike along a 1-D elevation profile. Raw DEM tiles contain
    occasional bad pixels; sampled onto the road they become phantom hills the
    car has to climb. A median is robust to isolated outliers while leaving a
    genuine steep grade (where the whole neighbourhood moves together) intact.
    """
    a = np.asarray(e, dtype=float)
    n = len(a)
    if n < 2 * win + 1:
        return [float(v) for v in a]
    out = a.copy()
    for i in range(n):
        i0, i1 = max(0, i - win), min(n, i + win + 1)
        nb = a[i0:i1]
        med = float(np.median(nb))
        mad = float(np.median(np.abs(nb - med)))
        # 2 m floor: below that we're inside DEM quantisation noise, not signal
        if abs(a[i] - med) > k * max(mad, 2.0):
            out[i] = med
    return [float(v) for v in out]


def _auto_max_grade(dists: list, elevs: list) -> float:
    """
    Estimate a grade limit from the road's own long-baseline trend.
    A ~800m rolling median erases DEM artefacts up to ~400m wide (gully
    crossings, canopy steps) while keeping genuine sustained climbs and
    broad valleys, so the trend's steepest grade is the road's real
    steepness. The limit is set comfortably above that: artefacts (whose
    walls are far steeper than the road ever is) get bridged, genuinely
    steep or dippy roads raise their own limit and keep their character.
    """
    e = np.asarray(elevs, dtype=float)
    d = np.asarray(dists, dtype=float)
    n = len(e)
    if n < 20 or d[-1] <= d[0]:
        return 0.35
    spacing = (d[-1] - d[0]) / (n - 1)
    win = int(800.0 / max(spacing, 1e-9))
    win = max(5, min(win | 1, n if n % 2 else n - 1))     # odd, ≤ n
    trend = median_filter(e, size=win, mode='nearest')
    g = np.abs(np.gradient(trend, d))
    limit = float(np.percentile(g, 95)) * 1.5 + 0.02
    return min(0.35, max(0.07, limit))


def _limit_grade(dists: list, elevs: list, max_grade: float = 0.35,
                 iters: int = 200) -> list:
    """
    Remove physically impossible road grades. DEM artefacts wider than the
    de-spike window (tile seams, void-fill blobs, valley smear) survive the
    median filter and become huge phantom rises/dips in the road. No drivable
    road sustains anywhere near a 35% grade, so samples that create one are
    treated as bad data and re-interpolated from the surrounding good samples.
    Iterates to convergence (each pass widens the bridged region): a tall
    plateau artefact — e.g. a canopy block on a flat road — needs its whole
    footprint bridged, not just its steep walls. Each pass is O(n), so cheap.
    """
    e = np.asarray(elevs, dtype=float)
    d = np.asarray(dists, dtype=float)
    if len(e) < 3:
        return [float(v) for v in e]
    fixed = 0
    for _ in range(iters):
        seg = np.maximum(np.diff(d), 1e-9)
        bad_seg = np.abs(np.diff(e)) / seg > max_grade
        if not bad_seg.any():
            break
        bad = np.zeros(len(e), dtype=bool)
        bad[:-1] |= bad_seg
        bad[1:]  |= bad_seg
        good = ~bad
        if good.sum() < 2:
            break
        e[bad] = np.interp(d[bad], d[good], e[good])
        fixed += int(bad.sum())
    if fixed:
        print(f"  [elevation] {fixed} sample(s) exceeded {max_grade:.0%} "
              f"grade — re-interpolated (DEM artefacts)")
    return [float(v) for v in e]


DEM_REGISTRATION_M = 15.0   # assumed horizontal uncertainty of the DEM (m)


def _corridor_elevations(sample_coords: list, half_width_m: float = 15.0,
                         registration_m: float = None):
    """
    Sample the DEM ACROSS the road corridor, not just at one point on the
    centreline.

    A point sample assumes the DEM knows exactly where the road is. It does
    not: ~30 m cells plus OSM geometry error put the true road anywhere
    within ~15 m of the sampled point. On flat ground that hardly matters.
    On a road cut into a hillside — bank one side, drop the other — a 15 m
    HORIZONTAL error becomes a 15 m VERTICAL error, and as the road curves
    against the contours that error swings: phantom dips and rises.

    At each station a 5-point cross-section is sampled and a quadratic is
    fitted across it, evaluated at the centreline. With symmetric offsets
    that estimate is exactly unbiased for any locally planar or parabolic
    cross-section (so a hillside or a valley floor is not skewed), while
    averaging ~half the per-pixel noise of a single sample.

    The fitted cross-slope also tells us how unreliable each station is:
    uncertainty ≈ cross-slope × registration error. That is reported rather
    than silently hidden, because on a steep sidehill no amount of processing
    can recover a narrow road bench a 30 m DEM never resolved.

    Returns (fitted, centre, cross_slope, uncertainty_m).
    """
    offsets = [-half_width_m, -half_width_m / 2, 0.0,
               half_width_m / 2, half_width_m]
    n = len(sample_coords)
    query = []
    for i, c in enumerate(sample_coords):
        i0, i1 = max(0, i - 1), min(n - 1, i + 1)
        coslat = max(1e-9, math.cos(math.radians(c[0])))
        dn = (sample_coords[i1][0] - sample_coords[i0][0]) * 111320.0
        de = (sample_coords[i1][1] - sample_coords[i0][1]) * 111320.0 * coslat
        L = math.hypot(de, dn) or 1.0
        pe, pn = -dn / L, de / L                 # perpendicular (east, north)
        for m in offsets:
            query.append([c[0] + pn * m / 111320.0,
                          c[1] + pe * m / (111320.0 * coslat)])

    vals = np.array(fetch_elevations(query), dtype=float).reshape(n, len(offsets))
    centre = vals[:, len(offsets) // 2]

    o  = np.array(offsets, dtype=float)
    S2 = float((o ** 2).sum())
    S4 = float((o ** 4).sum())
    det = S4 * len(o) - S2 * S2
    A = (vals * (o ** 2)).sum(axis=1)
    B = vals.sum(axis=1)
    fitted = (S4 * B - S2 * A) / det             # quadratic value at offset 0
    slope  = (vals * o).sum(axis=1) / S2         # cross-slope (m/m)

    cross = np.abs(slope)
    unc = cross * (registration_m if registration_m is not None
                   else DEM_REGISTRATION_M)
    steep = float((cross > 0.25).mean())
    print(f"  [elevation] corridor sampling: {steep:.0%} of route on steep "
          f"sidehill, typical uncertainty ±{float(np.median(unc)):.1f} m "
          f"(max ±{float(unc.max()):.1f} m)")
    return ([float(v) for v in fitted], [float(v) for v in centre],
            cross, unc)


def _pava(y: list) -> list:
    """Pool Adjacent Violators: least-squares non-decreasing fit. O(n)."""
    vals, wts, cnts = [], [], []
    for v in y:
        vals.append(float(v)); wts.append(1.0); cnts.append(1)
        while len(vals) > 1 and vals[-2] > vals[-1]:
            w = wts[-2] + wts[-1]
            vals[-2] = (vals[-2]*wts[-2] + vals[-1]*wts[-1]) / w
            wts[-2] = w; cnts[-2] += cnts[-1]
            vals.pop(); wts.pop(); cnts.pop()
    out = []
    for v, c in zip(vals, cnts):
        out.extend([v] * c)
    return out


def _monotonic_profile(elevs: list) -> list:
    """
    Force the profile monotonic in the direction of its net elevation change
    (isotonic regression). For a road the user knows climbs (or descends)
    steadily, any dip is by definition a DEM artefact — gully crossings on
    embankments/culverts the DEM can't see — and the least-squares monotonic
    fit removes exactly those while leaving already-monotonic sections
    untouched.
    """
    if len(elevs) < 3:
        return [float(v) for v in elevs]
    inc = elevs[-1] >= elevs[0]
    y = elevs if inc else [-v for v in elevs]
    fit = _pava(y)
    fixed = sum(1 for a, b in zip(y, fit) if abs(a - b) > 0.01)
    if fixed:
        print(f"  [elevation] steady-climb fit adjusted {fixed} sample(s)")
    return fit if inc else [-v for v in fit]


def fetch_elevation_profile(coords: list, max_grade: float = None,
                            monotonic: bool = False) -> dict:
    """
    Sample elevation at UNIFORM DISTANCE intervals along the route
    (not at the raw GPS nodes, which are unevenly spaced — dense in
    bends, sparse on straights — and would distort gradients).

    Returns {"dists": [m along route], "elevs": [m]} or None.
    """
    if len(coords) < 2:
        return None

    # Arc length along the raw polyline (haversine, metres)
    dists = [0.0]
    for i in range(1, len(coords)):
        dists.append(dists[-1] + _haversine(coords[i-1][0], coords[i-1][1],
                                            coords[i][0],   coords[i][1]))
    total = dists[-1]

    # Tile-backed elevation has no request limits, so sample densely (every
    # 10 m, up to 4000 points) for a gradient true to the real hill.
    lat_f = interp1d(dists, [c[0] for c in coords])
    lon_f = interp1d(dists, [c[1] for c in coords])
    # 3rd element (if present) flags bridge/tunnel spans
    flags = [float(c[2]) if len(c) > 2 else 0.0 for c in coords]
    flag_f = interp1d(dists, flags)

    interval = max(10.0, total / 3999.0)
    n_samples = max(int(total / interval) + 1, 4)
    sample_d = [min(i * interval, total) for i in range(n_samples)]
    sample_coords = [[float(lat_f(d)), float(lon_f(d))] for d in sample_d]
    sample_flags = [float(flag_f(d)) for d in sample_d]

    # ── Pick the pipeline for the data quality ──
    # The corrections below exist to fight coarse-DEM errors. On LiDAR they
    # can't tell a real feature from an artefact, so when a local high-res
    # DEM covers the road they step aside:
    #  - corridor narrows ±15m → ±3m (LiDAR resolves the road bench itself;
    #    a wide cross-section would average cut batters into the road)
    #  - registration uncertainty 15m → 1m (survey-grade georeferencing)
    #  - auto grade limiting off (a steep pinch in LiDAR is real)
    #  - profile smoothing 30m → 8m (keep real crests and compressions)
    # Bridge/tunnel flattening stays ON in both modes: bare-earth LiDAR
    # removes bridge decks too, so the road still needs levelling there.
    _load_local_dems(refresh=True)
    dem_cov = _local_dem_coverage(sample_coords)
    hires = dem_cov > 0.9
    dem_mode = "manual" if _local_dems else "auto"
    if hires:
        print(f"  [elevation] local DEM covers {dem_cov:.0%} of the road — "
              f"high-resolution pipeline (narrow corridor, minimal "
              f"smoothing, no auto grade limit)")

    elevs, centre, cross, unc = _corridor_elevations(
        sample_coords,
        half_width_m=3.0 if hires else 15.0,
        registration_m=1.0 if hires else DEM_REGISTRATION_M)
    raw = centre                        # naive point sample, for diagnostics
    elevs = _despike_1d(elevs)
    if max_grade is None:
        if hires:
            max_grade = 0.35            # effectively off: trust the data
        else:
            max_grade = _auto_max_grade(sample_d, elevs)
            print(f"  [elevation] auto grade limit: {max_grade:.0%}")
    elevs = _limit_grade(sample_d, elevs, max_grade=max_grade)
    elevs = _flatten_structures(elevs, sample_flags)
    if monotonic:
        elevs = _monotonic_profile(elevs)
    return {"dists": sample_d, "elevs": elevs, "raw_elevs": raw,
            "coords": sample_coords,
            "uncert": [float(v) for v in unc],
            "sidehill_frac": float((cross > 0.25).mean()),
            "uncert_med": float(np.median(unc)),
            "dem_mode": dem_mode,
            "dem_coverage": dem_cov,
            "smooth_sigma": 8.0 if hires else 30.0}


def fetch_elevation(coords: list) -> list:
    """Legacy per-coordinate elevation (kept for API compatibility)."""
    prof = fetch_elevation_profile(coords)
    if prof is None:
        return [0.0] * len(coords)
    dists = [0.0]
    for i in range(1, len(coords)):
        dists.append(dists[-1] + _haversine(coords[i-1][0], coords[i-1][1],
                                            coords[i][0],   coords[i][1]))
    f = interp1d(prof["dists"], prof["elevs"], kind='linear',
                 fill_value='extrapolate')
    return [float(f(d)) for d in dists]


# ─── ESA WorldCover 2021 (global 10 m land cover, free on AWS S3) ────────────
# OSM only knows what a mapper drew, so vegetation is patchy outside towns.
# WorldCover classifies every 10 m of ground on Earth, so trees and surface
# type work even where nobody has mapped a forest polygon.
# Files are Cloud-Optimised GeoTIFFs covering 3°×3° — far too big to download,
# so we parse the TIFF header and pull only the internal tiles we need via HTTP
# Range requests. Dataset: https://esa-worldcover.s3.eu-central-1.amazonaws.com
# (CC-BY 4.0, © ESA WorldCover project 2021 / Contains modified Copernicus data)

ESA_BASE_URL = ("https://esa-worldcover.s3.eu-central-1.amazonaws.com"
                "/v200/2021/map")
ESA_TILE_DEG = 3
LC_CACHE_DIR = os.path.join(OUTPUT_DIR, "landcover")

# WorldCover class codes
LC_TREE, LC_SHRUB, LC_GRASS, LC_CROP, LC_BUILT = 10, 20, 30, 40, 50
LC_BARE, LC_SNOW, LC_WATER, LC_WETLAND, LC_MANGROVE, LC_MOSS = 60, 70, 80, 90, 95, 100

_cog_hdr_cache = {}     # url -> parsed header
_cog_tile_cache = {}    # (url, tile_index) -> uint8 array


def _esa_tile_url(lat, lon):
    """WorldCover file covering a point (tiles named by their SW corner)."""
    tlat = int(math.floor(lat / ESA_TILE_DEG) * ESA_TILE_DEG)
    tlon = int(math.floor(lon / ESA_TILE_DEG) * ESA_TILE_DEG)
    ns = "N" if tlat >= 0 else "S"
    ew = "E" if tlon >= 0 else "W"
    name = (f"ESA_WorldCover_10m_2021_v200_"
            f"{ns}{abs(tlat):02d}{ew}{abs(tlon):03d}_Map.tif")
    return f"{ESA_BASE_URL}/{name}"


def _http_range(url, start, length):
    """Fetch a byte range. Returns bytes or None."""
    req = Request(url, headers={"User-Agent": "ATLAS Road World Studio/1.0",
                                "Range": f"bytes={start}-{start + length - 1}"})
    with urlopen(req, timeout=25) as r:
        return r.read()


def _parse_cog_header(url):
    """
    Minimal TIFF/COG header parse: enough to locate internal tiles.
    Returns dict with geotransform, tile layout and per-tile byte ranges.
    """
    if url in _cog_hdr_cache:
        return _cog_hdr_cache[url]

    head = _http_range(url, 0, 131072)
    if not head or len(head) < 8:
        raise IOError("empty TIFF header")
    bo = "<" if head[:2] == b"II" else ">"
    magic = struct.unpack(bo + "H", head[2:4])[0]
    if magic != 42:
        raise IOError(f"not a classic TIFF (magic {magic})")
    ifd_off = struct.unpack(bo + "I", head[4:8])[0]

    def u(fmt, off):
        return struct.unpack_from(bo + fmt, head, off)

    if ifd_off + 2 > len(head):
        raise IOError("IFD beyond fetched header")
    n_entries = u("H", ifd_off)[0]
    tags = {}
    TYPE_SIZE = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 12: 8, 16: 8}
    for i in range(n_entries):
        off = ifd_off + 2 + i * 12
        tag, typ, cnt = u("HHI", off)
        voff = off + 8
        size = TYPE_SIZE.get(typ, 4) * cnt
        if size > 4:
            voff = u("I", off + 8)[0]
        vals = []
        if voff + size <= len(head):
            for k in range(cnt):
                p = voff + k * TYPE_SIZE.get(typ, 4)
                if typ == 3:
                    vals.append(u("H", p)[0])
                elif typ == 4:
                    vals.append(u("I", p)[0])
                elif typ == 12:
                    vals.append(u("d", p)[0])
                elif typ == 1:
                    vals.append(head[p])
                else:
                    vals.append(u("I", p)[0])
        tags[tag] = vals

    def one(tag, default=None):
        v = tags.get(tag)
        return v[0] if v else default

    hdr = {
        "bo": bo,
        "width":  one(256), "height": one(257),
        "tile_w": one(322), "tile_h": one(323),
        "offsets": tags.get(324, []), "counts": tags.get(325, []),
        "compression": one(259, 1), "predictor": one(317, 1),
        "scale": tags.get(33550, []), "tiepoint": tags.get(33922, []),
    }
    if not hdr["tile_w"] or not hdr["offsets"]:
        raise IOError("not a tiled COG")
    if not hdr["scale"] or not hdr["tiepoint"]:
        raise IOError("missing geotransform")
    _cog_hdr_cache[url] = hdr
    return hdr


def _cog_tile(url, hdr, ti):
    """Decode one internal COG tile (uint8 band) via a Range request."""
    key = (url, ti)
    if key in _cog_tile_cache:
        return _cog_tile_cache[key]
    import zlib as _z
    raw = _http_range(url, hdr["offsets"][ti], hdr["counts"][ti])
    comp = hdr["compression"]
    if comp in (8, 32946):
        data = _z.decompress(raw)
    elif comp == 1:
        data = raw
    else:
        raise IOError(f"unsupported TIFF compression {comp}")
    tw, th = hdr["tile_w"], hdr["tile_h"]
    arr = np.frombuffer(data[:tw * th], dtype=np.uint8).reshape(th, tw).copy()
    if hdr["predictor"] == 2:
        np.cumsum(arr, axis=1, dtype=np.uint8, out=arr)
    _cog_tile_cache[key] = arr
    return arr


def _lc_sample(url, hdr, lat, lon):
    """Land-cover class code at a point, or 0 if unavailable."""
    sx, sy = hdr["scale"][0], hdr["scale"][1]
    ox, oy = hdr["tiepoint"][3], hdr["tiepoint"][4]
    px = int((lon - ox) / sx)
    py = int((oy - lat) / sy)
    if px < 0 or py < 0 or px >= hdr["width"] or py >= hdr["height"]:
        return 0
    tw, th = hdr["tile_w"], hdr["tile_h"]
    across = (hdr["width"] + tw - 1) // tw
    ti = (py // th) * across + (px // tw)
    if ti >= len(hdr["offsets"]):
        return 0
    tile = _cog_tile(url, hdr, ti)
    return int(tile[py % th, px % tw])


def fetch_land_cover(coords: list):
    """
    Build a land-cover sampler for the area around a route. Returns a function
    (lat, lon) -> WorldCover class code, or None if the data is unavailable
    (callers then fall back to OSM-only behaviour).
    """
    if not coords:
        return None
    os.makedirs(LC_CACHE_DIR, exist_ok=True)

    urls = {}
    for c in coords[::max(1, len(coords) // 50)]:
        # coords may carry a 3rd bridge/tunnel element — index, don't unpack
        u = _esa_tile_url(c[0], c[1])
        if u not in urls:
            try:
                urls[u] = _parse_cog_header(u)
            except Exception as e:
                raise ElevationUnavailable(
                    f"could not read ESA WorldCover for this area: {e}")
    if not urls:
        return None
    print(f"  [landcover] ESA WorldCover: {len(urls)} tile file(s)")

    def sampler(lat, lon):
        u = _esa_tile_url(lat, lon)
        hdr = urls.get(u)
        if hdr is None:
            try:
                hdr = _parse_cog_header(u)
                urls[u] = hdr
            except Exception:
                return 0
        try:
            return _lc_sample(u, hdr, lat, lon)
        except Exception:
            return 0

    return sampler


# Terrain surface material per land-cover class
LC_MATERIAL = {
    LC_TREE: "forest", LC_SHRUB: "forest", LC_MANGROVE: "forest",
    LC_GRASS: "terrain", LC_CROP: "terrain", LC_WETLAND: "terrain",
    LC_MOSS: "terrain", LC_BUILT: "dirt", LC_BARE: "dirt",
    LC_SNOW: "dirt", LC_WATER: "water",
}
# Classes where trees are scattered when OSM has no vegetation polygon
LC_TREE_DENSITY = {LC_TREE: 1.0, LC_MANGROVE: 0.8, LC_SHRUB: 0.35}


# ─── Road Geometry ────────────────────────────────────────────────────────────

# Lateral terrain profile (metres from road edge → height drop below road)
GRASS_W    = 10.0   # grass verge width each side
GRASS_DROP = 0.4    # drop at grass outer edge
SKIRT_W    = 70.0   # terrain skirt beyond the grass (10m → 80m out)
SKIRT_DROP = 2.0    # total drop at skirt outer edge


def _fix_edge_folds(ex: np.ndarray, ez: np.ndarray,
                    dxs: np.ndarray, dzs: np.ndarray) -> tuple:
    """
    Repair folded offset edges. On a bend tighter than the offset distance
    the offset polyline reverses direction and crosses itself; in the physics
    mesh that becomes overlapping/inverted collision triangles — the
    invisible spikes that launch the car in AC. Points where the edge runs
    backwards relative to the centreline are re-interpolated from the
    surrounding healthy points (the edge collapses to a chord there).
    """
    vex = np.diff(ex)
    vez = np.diff(ez)
    seg_bad = (vex * dxs[:-1] + vez * dzs[:-1]) <= 1e-9
    if not seg_bad.any():
        return ex, ez
    bad = np.zeros(len(ex), dtype=bool)
    bad[:-1] |= seg_bad
    bad[1:]  |= seg_bad
    good = ~bad
    if good.sum() < 2:
        return ex, ez
    idx = np.arange(len(ex), dtype=float)
    ex = ex.copy(); ez = ez.copy()
    ex[bad] = np.interp(idx[bad], idx[good], ex[good])
    ez[bad] = np.interp(idx[bad], idx[good], ez[good])
    return ex, ez


def ground_drop(lateral_from_road_edge: float) -> float:
    """Height drop of the terrain at a lateral distance from the road EDGE."""
    d = max(0.0, lateral_from_road_edge)
    if d <= GRASS_W:
        return GRASS_DROP * (d / GRASS_W)
    if d <= GRASS_W + SKIRT_W:
        return GRASS_DROP + (SKIRT_DROP - GRASS_DROP) * ((d - GRASS_W) / SKIRT_W)
    return SKIRT_DROP


# ─── Real Terrain (world-space grid conformed to the road) ────────────────────
# Instead of offsetting the centreline into lateral rings (which self-intersect
# on tight corners and cross the road), a regular grid is sampled over a band
# around the route and CONFORMED to the road: at the road it sits at road height;
# moving outward it blends into the real DEM shape. The grid + an explicit collar
# at the grass edge are triangulated (Delaunay) in world XZ, which cannot fold, so
# hairpins and switchbacks are handled without terrain crossing the road, and the
# road-height anchoring keeps the terrain seam-free with the road/grass regardless
# of DEM noise or road smoothing. Standard "indent + surrounding" conform approach
# (cf. EasyRoads3D / GIS road-flattening).

TERRAIN_MAX_DIST  = 90.0    # how far terrain reaches from the road (m)
TERRAIN_GRID_STEP = 12.0    # grid node spacing (m)
TERRAIN_BLEND     = 12.0    # distance over which terrain blends road→DEM (m).
                            # Kept short: a wide blend flattens real cliff
                            # faces and cuttings beside the road into gentle
                            # artificial verges. 12 m hides the road/DEM seam
                            # while letting the real landform show beyond it.


def _despike_grid(g: np.ndarray, win: int = 2, k: float = 4.0) -> np.ndarray:
    """
    Replace outlier cells with the local median, using a (2*win+1)² window and
    median absolute deviation as the threshold. MAD is robust: a real ridge
    raises the whole neighbourhood's median so it survives, while a lone bad
    pixel stands far off its neighbours and gets flattened. NaNs are filled
    from the local median too. Returns a new array.
    """
    out = g.copy()
    nX, nZ = g.shape
    for i in range(nX):
        for j in range(nZ):
            c = g[i, j]
            i0, i1 = max(0, i - win), min(nX, i + win + 1)
            j0, j1 = max(0, j - win), min(nZ, j + win + 1)
            nb = g[i0:i1, j0:j1]
            nb = nb[~np.isnan(nb)]
            if nb.size < 3:
                continue
            med = float(np.median(nb))
            if np.isnan(c):
                out[i, j] = med
                continue
            mad = float(np.median(np.abs(nb - med)))
            # 1.5 m floor keeps flat ground (MAD≈0) from flagging every ripple
            if abs(c - med) > k * max(mad, 1.5):
                out[i, j] = med
    return out


def _suppress_canopy(node_grid: np.ndarray, node_ids: list, node_latlon: list,
                     lc_fn, dist: np.ndarray, near: np.ndarray,
                     ry: np.ndarray, step: float) -> np.ndarray:
    """
    Remove phantom terrain peaks caused by tree canopy.

    The DEM is a SURFACE model: over woodland it measures treetops, so a
    patch of gums stands ~10-20 m above the adjacent bare ground and becomes
    a hillock in the terrain mesh that does not exist. Gated on ESA
    WorldCover, every tree-covered node is lowered by a typical canopy
    height, with the tree mask gaussian-blurred so forest edges ramp instead
    of step. Two safety properties:

    - Bare ground is never touched (mask is zero there), so real hills and
      cliffs survive; continuous treed slopes shift down uniformly, keeping
      their shape (and landing nearer the true ground).
    - A local-minimum floor stops the subtraction carving craters where the
      DEM actually resolved ground through sparse canopy: no node may end
      below the lowest original height in its 5×5 neighbourhood.
    """
    if lc_fn is None:
        return node_grid
    if _local_dems:
        # Local DEMs (ELVIS LiDAR) are bare-earth ground models — there is no
        # canopy in the data, so subtracting one would carve real terrain.
        print("  [terrain] canopy suppression skipped: local ground DEM active")
        return node_grid
    CANOPY_TYP = 12.0
    from scipy.ndimage import gaussian_filter, minimum_filter, maximum_filter

    tree_classes = {LC_TREE, LC_MANGROVE}
    mask = np.zeros(node_grid.shape, dtype=float)
    known = np.zeros(node_grid.shape, dtype=bool)
    for (i, j), (la, lo) in zip(node_ids, node_latlon):
        known[i, j] = True
        try:
            if lc_fn(la, lo) in tree_classes:
                mask[i, j] = 1.0
        except Exception:
            pass
    if not mask.any():
        return node_grid

    # Dilate then blur: edge canopy cells must receive the FULL correction
    # (a blurred-only mask leaves a tall rim around every patch); the blur
    # spills partial correction onto adjacent bare cells, where the bare
    # floor below cancels it.
    mask_s = gaussian_filter(maximum_filter(mask, size=3), sigma=1.0)
    # Floor from BARE ground only: where the DEM resolved actual ground
    # nearby, don't carve below it. Inside continuous forest there is no
    # bare reference, and the full subtraction applies.
    bare_h = np.where(known & (mask == 0.0) & ~np.isnan(node_grid),
                      node_grid, np.inf)
    wmin = minimum_filter(bare_h, size=5) - 1.0

    out = node_grid - CANOPY_TYP * mask_s
    out = np.maximum(out, np.where(np.isfinite(wmin), wmin, -np.inf))
    out = np.where(np.isnan(node_grid), node_grid, out)

    changed = int(((node_grid - out) > 0.5).sum())
    if changed:
        drop = float(np.nanmax(node_grid - out))
        print(f"  [terrain] canopy suppression: lowered {changed} tree-covered "
              f"node(s) by up to {drop:.1f} m")
    return out


def fetch_terrain_grid(mesh: dict) -> dict:
    """
    Sample the real DEM on a world-space grid over a band around the route,
    plus along the road centreline for height anchoring. Returns everything
    build_terrain_meshes needs, or None if the elevation APIs are unavailable.
    """
    cl = mesh["centerline"]
    proj_p = mesh["proj"]

    inv = Transformer.from_crs(
        f"+proj=tmerc +lat_0={proj_p['mid_lat']} +lon_0={proj_p['mid_lon']} +units=m",
        "EPSG:4326", always_xy=True)
    ox, oz = proj_p["ox"], proj_p["oz"]

    def to_latlon(x, z):
        lon, lat = inv.transform(x + ox, -(z + oz))   # undo handedness flip
        return [lat, lon]

    # ── Road reference: resample the centreline ~every 6 m, carrying road
    #    height, and sample the DEM at each reference point so terrain can be
    #    anchored to the road's own elevation source (seam-free). ──
    cl_xz = np.array([[p[0], p[2]] for p in cl])
    seg = np.sqrt(((cl_xz[1:] - cl_xz[:-1]) ** 2).sum(axis=1))
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(arc[-1])
    n_ref = max(2, int(total / 6.0) + 1)
    ref_d = np.linspace(0, total, n_ref)
    rx = np.interp(ref_d, arc, cl_xz[:, 0])
    rz = np.interp(ref_d, arc, cl_xz[:, 1])
    ry = np.interp(ref_d, arc, np.array([p[1] for p in cl]))   # road height
    ref_latlon = [to_latlon(float(rx[i]), float(rz[i])) for i in range(n_ref)]

    # ── Grid over the route's bounding box, padded by the reach distance ──
    pad = TERRAIN_MAX_DIST + TERRAIN_GRID_STEP
    min_x = float(cl_xz[:, 0].min()) - pad
    max_x = float(cl_xz[:, 0].max()) + pad
    min_z = float(cl_xz[:, 1].min()) - pad
    max_z = float(cl_xz[:, 1].max()) + pad
    gx = np.arange(min_x, max_x + TERRAIN_GRID_STEP, TERRAIN_GRID_STEP)
    gz = np.arange(min_z, max_z + TERRAIN_GRID_STEP, TERRAIN_GRID_STEP)
    nX, nZ = len(gx), len(gz)

    # Nearest road reference per node → distance + anchor index
    from scipy.spatial import cKDTree
    kd = cKDTree(np.column_stack([rx, rz]))
    GX, GZ = np.meshgrid(gx, gz, indexing="ij")          # (nX, nZ)
    flat_xz = np.column_stack([GX.ravel(), GZ.ravel()])
    dist_flat, near_flat = kd.query(flat_xz)
    dist = dist_flat.reshape(nX, nZ)
    near = near_flat.reshape(nX, nZ).astype(int)

    # Only nodes within reach need a real DEM sample; the rest stay unused.
    within = dist <= (TERRAIN_MAX_DIST + TERRAIN_GRID_STEP)
    node_ids = [tuple(map(int, ij)) for ij in np.argwhere(within)]
    node_latlon = [to_latlon(float(GX[i, j]), float(GZ[i, j])) for i, j in node_ids]

    # ── One batched DEM fetch: road references + grid nodes ──
    query = ref_latlon + node_latlon
    print(f"  [terrain] sampling {len(query)} DEM points "
          f"({n_ref} road refs + {len(node_latlon)} grid nodes, "
          f"{nX}×{nZ} grid)…")
    elevs = fetch_elevations(query)

    dem_ref = np.array(elevs[:n_ref], dtype=float)

    # ── De-spike the sampled lattice ──
    # Raw DEM tiles carry isolated bad pixels that become vertical spikes in
    # the mesh. A 5×5 MEDIAN + MAD filter removes them while preserving real
    # ridges and canyons — unlike a Gaussian blur, which flattens genuine
    # terrain along with the noise.
    node_grid = np.full((nX, nZ), np.nan)
    for k, (i, j) in enumerate(node_ids):
        node_grid[i, j] = float(elevs[n_ref + k])
    node_grid = _despike_grid(node_grid)
    node_grid = _suppress_canopy(node_grid, node_ids, node_latlon,
                                 mesh.get("land_cover"), dist, near, ry,
                                 TERRAIN_GRID_STEP)

    dem_node = {}
    for (i, j) in node_ids:
        v = node_grid[i, j]
        if not np.isnan(v):
            dem_node[(i, j)] = float(v)

    return {
        "gx": gx, "gz": gz, "nX": nX, "nZ": nZ,
        "dist": dist, "near": near, "within": within,
        "rx": rx, "rz": rz, "ry": ry,
        "dem_ref": dem_ref, "dem_node": dem_node,
    }


def _conform_terrain_to_side_paths(pts_xz, ys, surroundings, mesh, kd_cl, hw):
    """
    Grade the terrain along side roads and waterways, TWO ways:

    1. Adjust existing grid nodes near each path toward a smooth along-path
       profile (flat across, feathered back into raw terrain laterally and
       at path ends).
    2. INJECT ribbon cross-sections (centre, ±half-width at profile height,
       ±blend-edge at raw terrain height) into the triangulation. The 12m
       grid alone almost never has a node inside a 3–4m corridor, so without
       these the "shelf" only partially existed between grid nodes and the
       draped strips still floated over residual bumps.

    Waterways are depressed slightly so water sits in a shallow channel.
    Returns (adjusted_ys, extra_pts_xz, extra_ys).
    """
    from scipy.spatial import cKDTree
    paths = []
    for rd in ([] if mesh.get("unified_network") else surroundings.get("roads", [])):
        paths.append((rd["pts"], SIDE_ROAD_W.get(rd.get("type"), 5.0) / 2.0,
                      0.0, "road"))
    for ww in surroundings.get("waterways", []):
        paths.append((ww["pts"], WATER_W.get(ww.get("type"), 3.0) / 2.0,
                      0.35, "water"))
    corridors = []          # (samples_xz, paint_half_w, kind) for painting
    drop = np.zeros(len(ys), dtype=bool)   # grid nodes inside the paint zone
    if not paths:
        return ys, np.empty((0, 2)), np.empty(0), corridors, drop

    proj_p = mesh["proj"]
    proj = Transformer.from_crs(
        "EPSG:4326",
        f"+proj=tmerc +lat_0={proj_p['mid_lat']} +lon_0={proj_p['mid_lon']} +units=m",
        always_xy=True)
    ox, oz = proj_p["ox"], proj_p["oz"]

    def to_local(lat, lon):
        X, Z = proj.transform(lon, lat)
        return X - ox, -Z - oz

    node_kd = cKDTree(pts_xz)
    BLEND = 7.0
    adj_w = np.zeros(len(ys))
    adj_t = np.zeros(len(ys))
    extra_xz, extra_y = [], []
    n_conf = 0
    for pts_ll, p_hw, depress, kind in paths:
        half_w = p_hw + 0.6           # graded flat zone: paint edge + margin
        loc = [to_local(la, lo) for la, lo in pts_ll]
        # resample every 6m along the polyline
        rs = []
        carry = 0.0
        for k in range(len(loc) - 1):
            x0, z0 = loc[k]; x1, z1 = loc[k + 1]
            seg = math.hypot(x1 - x0, z1 - z0)
            if seg < 1e-6:
                continue
            t = carry
            while t < seg:
                f = t / seg
                rs.append((x0 + (x1 - x0) * f, z0 + (z1 - z0) * f))
                t += 6.0
            carry = t - seg
        if len(rs) < 3:
            continue
        rs = np.array(rs)
        corridors.append((rs, p_hw, kind))
        # target profile: current terrain height along the path, smoothed —
        # a graded version of the hill the path climbs
        d4, i4 = node_kd.query(rs, k=4)
        w4 = 1.0 / np.maximum(d4, 1e-6)
        prof = (ys[i4] * w4).sum(axis=1) / w4.sum(axis=1)
        prof = gaussian_filter1d(prof, 4.0) - depress
        # keep clear of the main road: its own grading wins near junctions
        d_cl, _ = kd_cl.query(rs)
        keep = d_cl > hw + 6.0
        if keep.sum() < 2:
            continue
        # longitudinal end taper: without it the shelf STOPS dead where the
        # path ends (or where samples were dropped at the main road), leaving
        # a step back onto raw terrain — feather over ~4 samples (24m),
        # applied per contiguous kept run
        end_w = np.zeros(len(rs))
        k0 = None
        for k in range(len(rs) + 1):
            if k < len(rs) and keep[k]:
                if k0 is None:
                    k0 = k
            elif k0 is not None:
                for k2 in range(k0, k):
                    end_w[k2] = min(1.0, (k2 - k0 + 1) / 4.0,
                                    (k - k2) / 4.0)
                k0 = None
        s_kd = cKDTree(rs[keep])
        pk = prof[keep]
        ek = end_w[keep]
        # k=2 + inverse-distance blend ≈ linear interpolation along the path.
        # Nearest-sample-only gave nodes a piecewise-CONSTANT profile that
        # fought the linear ribbon between cross-sections (visible wobble).
        dn2, sn2 = s_kd.query(pts_xz, k=2,
                              distance_upper_bound=half_w + BLEND)
        dn = dn2[:, 0]
        m = np.isfinite(dn)
        if not m.any():
            continue
        w = np.clip((half_w + BLEND - dn[m]) / BLEND, 0.0, 1.0)
        w = w * w * (3.0 - 2.0 * w)                # smoothstep
        s0 = np.clip(sn2[m, 0], 0, len(pk) - 1)
        s1 = np.clip(sn2[m, 1], 0, len(pk) - 1)
        d0 = np.maximum(dn2[m, 0], 1e-6)
        d1 = dn2[m, 1]
        both = np.isfinite(d1)
        tgt = pk[s0].copy()
        ew_n = ek[s0].copy()
        if both.any():
            w0 = 1.0 / d0[both]
            w1 = 1.0 / np.maximum(d1[both], 1e-6)
            tgt[both] = (pk[s0[both]] * w0 + pk[s1[both]] * w1) / (w0 + w1)
            ew_n[both] = (ek[s0[both]] * w0 + ek[s1[both]] * w1) / (w0 + w1)
        w = w * ew_n
        idxs = np.nonzero(m)[0]
        # Grid nodes inside the painted corridor are REMOVED from the
        # triangulation: the bed surface comes only from ribbon points, so a
        # node that disagrees with the bed (path crossings, taper ends) can
        # no longer poke a triangle up through the road surface.
        drop[idxs[dn[m] <= p_hw + 0.4]] = True
        stronger = w > adj_w[idxs]
        adj_w[idxs[stronger]] = w[stronger]
        adj_t[idxs[stronger]] = tgt[stronger]
        # ribbon cross-sections: a real graded road bed in the mesh
        kept_i = np.nonzero(keep)[0]
        for kk in range(0, len(kept_i), 1):
            si = kept_i[kk]
            ew_ = end_w[si]
            if ew_ <= 0.01:
                continue
            i0 = kept_i[max(kk - 1, 0)]
            i1 = kept_i[min(kk + 1, len(kept_i) - 1)]
            tx_ = rs[i1][0] - rs[i0][0]
            tz_ = rs[i1][1] - rs[i0][1]
            tl = math.hypot(tx_, tz_) or 1.0
            qx, qz = -tz_ / tl, tx_ / tl            # lateral unit vector
            p_y = prof[si]
            for off in (0.0, p_hw, -p_hw, half_w, -half_w,
                        half_w + BLEND, -(half_w + BLEND)):
                ex_ = rs[si][0] + qx * off
                ez_ = rs[si][1] + qz * off
                # raw terrain height at this exact spot
                d4e, i4e = node_kd.query([ex_, ez_], k=4)
                w4e = 1.0 / np.maximum(d4e, 1e-6)
                raw_y = float((ys[i4e] * w4e).sum() / w4e.sum())
                if abs(off) > half_w + 1e-6:        # blend edge: meets terrain
                    extra_y.append(raw_y)
                else:                               # road bed: graded profile,
                    extra_y.append(p_y * ew_        # tapered at path ends
                                   + raw_y * (1.0 - ew_))
                extra_xz.append((ex_, ez_))
        n_conf += 1
    # never fight the main road corridor's own grading
    d_main, _ = kd_cl.query(pts_xz)
    adj_w[d_main < hw + 6.0] = 0.0
    if n_conf:
        print(f"  [terrain] graded road beds along {n_conf} side roads / "
              f"waterways ({len(extra_xz)} ribbon points)")
    return (ys * (1.0 - adj_w) + adj_t * adj_w,
            np.array(extra_xz) if extra_xz else np.empty((0, 2)),
            np.array(extra_y) if extra_y else np.empty(0),
            corridors, drop)


def build_terrain_meshes(mesh: dict, grid: dict,
                         surroundings: dict = None) -> list:
    """
    Triangulate the conformed grid + a grass-edge collar into physical terrain
    (1GRASS_* → AC GRASS surface, so leaving the road lands on real ground).
    At the road the surface sits at grass-edge height; it blends into the real
    DEM shape over TERRAIN_BLEND metres. Returns
    [(name, kn5_verts, indices, "terrain")].
    """
    lc = mesh.get("land_cover")
    inv_ll = mesh.get("to_latlon")
    from scipy.spatial import Delaunay, cKDTree

    hw = mesh["stats"]["road_width"] / 2.0
    grass_w = mesh["stats"].get("grass_width", GRASS_W)
    g_drop  = mesh["stats"].get("grass_drop", GRASS_DROP)
    indent  = hw + grass_w                 # corridor edge = grass outer edge

    gx = grid["gx"]; gz = grid["gz"]
    dist = grid["dist"]; near = grid["near"]
    rx = grid["rx"]; rz = grid["rz"]; ry = grid["ry"]
    dem_ref = grid["dem_ref"]; dem_node = grid["dem_node"]

    pts = []      # (x, z)
    ys  = []      # height

    # True distance to the FULL centreline (not the 6m road refs, whose
    # polyline cuts corners and underestimates closeness on curves — which
    # otherwise lets terrain spill onto the road on bends).
    cl_xz = np.array([[p[0], p[2]] for p in mesh["centerline"]])
    cl_y  = np.array([p[1] for p in mesh["centerline"]])
    kd_cl = cKDTree(cl_xz)

    # ── Exterior grid nodes (outside the road corridor), conformed height ──
    # y = grass_edge_height + (DEM_here − DEM_at_nearest_road) · blend(dist)
    for (i, j), dv in dem_node.items():
        nx0, nz0 = float(gx[i]), float(gz[j])
        d, _ = kd_cl.query([nx0, nz0])       # true distance to road
        if d < indent:
            continue                       # corridor interior: road covers it
        a = int(near[i, j])
        base = ry[a] - g_drop
        w = (d - indent) / TERRAIN_BLEND
        w = 0.0 if w < 0 else (1.0 if w > 1 else w)
        pts.append((nx0, nz0))
        ys.append(base + (dv - dem_ref[a]) * w)

    # ── Collar at the grass outer edge (both sides of the road) ──
    # A dense inner boundary at exactly the grass-edge height, so terrain meets
    # the road's grass strip watertight and no gap can open where the grid
    # lattice misses the road.
    n_ref = len(rx)
    # Collar tucks 0.3m UNDER the road edge, 5cm below its surface. The road
    # edge polyline (per-point normals + fold fixing) and this collar ring
    # (6m refs + curvature capping) are different curves — they only coincide
    # on straights, and on corners the divergence opened sliver gaps between
    # road and terrain. Overlapping in plan with the collar just below the
    # road surface makes a gap geometrically impossible; the overlap strip is
    # hidden under the carriageway.
    # DOUBLE collar ring. The outer ring sits AT the road edge just below
    # the surface — terrain's climb into the hillside starts there, never
    # inside the carriageway (a single tucked ring let uphill terrain ramp
    # up from inside the road and cover the edge lines). The inner ring sits
    # 0.5m INSIDE the road edge, deeper below the surface: the strip between
    # the rings underlaps the carriageway, so any plan divergence between
    # this ring and the true fold-fixed road edge (up to ~0.46m on an R=10m
    # hairpin with 6m refs) is covered from below — no sliver gap can open.
    OVERLAP = 0.50
    RINGS = ((OVERLAP, 0.10),   # (inset from road edge, depth below surface)
             (0.0,     0.04))   # 2cm shimmered against the road at distance
    floor = max(0.5, hw - OVERLAP)

    def _cl_height_at(px, pz, ci):
        """Road height at the point's projection onto the centreline —
        nearest-POINT height mismatches the local surface by up to
        grade × half the centreline spacing on steep roads, enough to poke
        the tucked collar up through the carriageway."""
        best = float(cl_y[ci]); bd = None
        for j0 in (ci - 1, ci):
            j1 = j0 + 1
            if j0 < 0 or j1 >= len(cl_xz):
                continue
            ax0, az0 = cl_xz[j0]; bx0, bz0 = cl_xz[j1]
            ex, ez = bx0 - ax0, bz0 - az0
            L2 = ex * ex + ez * ez
            if L2 < 1e-9:
                continue
            t = min(1.0, max(0.0, ((px - ax0) * ex + (pz - az0) * ez) / L2))
            qx, qz = ax0 + ex * t, az0 + ez * t
            d = (px - qx) ** 2 + (pz - qz) ** 2
            if bd is None or d < bd:
                bd = d
                best = float(cl_y[j0]) * (1 - t) + float(cl_y[j1]) * t
        return best
    for i in range(n_ref):
        i0 = max(i - 1, 0); i1 = min(i + 1, n_ref - 1)
        dx = rx[i1] - rx[i0]; dz = rz[i1] - rz[i0]
        L = math.hypot(dx, dz) or 1.0
        px_, pz_ = -dz / L, dx / L
        # Local radius of curvature (circumcircle of the three refs); on the
        # inside of a bend tighter than the collar offset, an offset would fold
        # across the road, so cap it to 0.8·R (floored just past the road edge).
        ax, az = rx[i0], rz[i0]; bx, bz = rx[i], rz[i]; cxr, czr = rx[i1], rz[i1]
        det = 2.0 * (ax * (bz - czr) + bx * (czr - az) + cxr * (az - bz))
        Ox = Oz = None
        if abs(det) > 1e-9:
            a2 = ax*ax + az*az; b2 = bx*bx + bz*bz; c2 = cxr*cxr + czr*czr
            Ox = (a2*(bz - czr) + b2*(czr - az) + c2*(az - bz)) / det
            Oz = (a2*(cxr - bx) + b2*(ax - cxr) + c2*(bx - ax)) / det
            Rc = math.hypot(bx - Ox, bz - Oz)
        for side in (+1, -1):
            for inset, tuck in RINGS:
                d_off = indent - inset
                if Ox is not None and Rc < 1e4:
                    if side * (px_ * (Ox - bx) + pz_ * (Oz - bz)) > 0:  # inside
                        d_off = min(d_off, 0.8 * Rc)
                d_off = max(floor, d_off)
                cxp = rx[i] + side * px_ * d_off
                czp = rz[i] + side * pz_ * d_off
                # A ref-perpendicular offset can still land inside the road
                # on a bend (nearest true-centreline point differs from the
                # ref); push it back out so it never crosses the centre.
                dcl, ci = kd_cl.query([cxp, czp])
                if dcl < floor:
                    ox0, oz0 = cl_xz[ci]
                    vx, vz = cxp - ox0, czp - oz0
                    vl = math.hypot(vx, vz) or 1.0
                    cxp = ox0 + vx / vl * floor
                    czp = oz0 + vz / vl * floor
                pts.append((cxp, czp))
                # Height from the projection onto the centreline: nearest-
                # POINT heights mismatch the local surface on steep grades,
                # enough to poke a tucked ring up through the carriageway.
                ys.append(_cl_height_at(cxp, czp, int(ci)) - g_drop - tuck)

    if len(pts) < 3:
        return []
    pts = np.array(pts, dtype=float)
    ys  = np.array(ys, dtype=float)

    # ── Level water surfaces ──
    # DEM over water is noisy, so classified water would otherwise be a bumpy
    # sheet. Each water point takes the MEDIAN of nearby water points: that
    # kills the ripple, but unlike flattening a whole body to one height it
    # still lets a river run downhill.
    if lc is not None and inv_ll is not None:
        wmask = np.zeros(len(pts), dtype=bool)
        for i, (px_, pz_) in enumerate(pts):
            la, lo = inv_ll(float(px_), float(pz_))
            if lc(la, lo) == LC_WATER:
                wmask[i] = True
        widx = np.nonzero(wmask)[0]
        if len(widx) >= 3:
            wkd = cKDTree(pts[widx])
            neigh = wkd.query_ball_point(pts[widx], r=TERRAIN_GRID_STEP * 5.0)
            levelled = ys[widx].copy()
            for a, nb in enumerate(neigh):
                if len(nb) >= 3:
                    levelled[a] = float(np.median(ys[widx][nb]))
            ys[widx] = levelled
            print(f"  [terrain] levelled {len(widx)} water points")

    # ── Side-road / waterway shelves ──
    # Grade the terrain itself along side paths, so the draped strips lie ON
    # the ground instead of hovering over its bumps.
    corridors = []
    if surroundings:
        try:
            ys, ex_xz, ex_y, corridors, drop_n = \
                _conform_terrain_to_side_paths(
                    pts, ys, surroundings, mesh, kd_cl, hw)
            if drop_n.any():                 # nodes inside painted corridors
                pts = pts[~drop_n]
                ys = ys[~drop_n]
            if len(ex_xz):
                pts = np.vstack([pts, ex_xz])
                ys = np.concatenate([ys, ex_y])
        except Exception as e:
            print(f"  [terrain] side-path grading skipped: {e}")
    pos = np.column_stack([pts[:, 0], ys, pts[:, 1]])   # (N,3) world xyz

    tri = Delaunay(pts)
    max_edge = 3.0 * TERRAIN_GRID_STEP     # drop triangles that span the road

    # Keep only triangles that don't cross the road and don't span a wide gap
    # (e.g. across a hairpin throat). Distance is to the FULL centreline, and
    # a triangle is rejected if its centroid OR any vertex sits inside the road.
    keep = []
    for s in tri.simplices:
        p = pts[s]
        e = max(math.hypot(*(p[0] - p[1])),
                math.hypot(*(p[1] - p[2])),
                math.hypot(*(p[2] - p[0])))
        if e > max_edge:
            continue
        cx, cz = p.mean(axis=0)
        dc, _ = kd_cl.query([cx, cz])
        dv, _ = kd_cl.query(p)             # distance of each vertex to road
        # thresholds sit just inside the tucked collar (hw−0.50): the hidden
        # overlap strip is kept, anything genuinely over the road is dropped
        if dc < hw - 0.55 or float(dv.min()) < hw - 0.60:
            continue
        keep.append(s)
    if not keep:
        return []

    # ── Vertex normals from accumulated face normals ──
    nrm = np.zeros((len(pts), 3))
    for s in keep:
        p0, p1, p2 = pos[s]
        n = np.cross(p1 - p0, p2 - p0)
        if n[1] < 0:
            n = -n
        nrm[s] += n
    ln = np.linalg.norm(nrm, axis=1)
    ln[ln == 0] = 1.0
    nrm = nrm / ln[:, None]

    # ── Emit triangles, grouped by land-cover surface, chunked under the KN5
    #    65k-vertex limit. Each triangle takes the material of its centroid's
    #    WorldCover class, so forest floor, bare ground and water read
    #    differently instead of one uniform green.
    out = []
    groups = {}          # material -> [verts, idx, vmap, part]

    # Corridor lookup for painting side roads / waterways ONTO the terrain.
    # The terrain already carries their graded bed; texturing those triangles
    # directly (instead of hovering a separate strip above them) makes the
    # side road flush and hill-integrated BY CONSTRUCTION — there is no
    # second surface to misalign. Painted road triangles get ROAD physics.
    cor_kd = None
    if corridors:
        cp, ct, cv, chw, ck = [], [], [], [], []
        v_run = 0.0
        for rs_, phw_, kind_ in corridors:
            tanx = np.gradient(rs_[:, 0]); tanz = np.gradient(rs_[:, 1])
            tl = np.sqrt(tanx ** 2 + tanz ** 2) + 1e-9
            cp.append(rs_)
            ct.append(np.column_stack([tanx / tl, tanz / tl]))
            cv.append(v_run + np.arange(len(rs_)) * 6.0)   # arc length (6m)
            v_run += len(rs_) * 6.0 + 64.0                 # gap between paths
            chw.append(np.full(len(rs_), phw_))
            ck.append(np.full(len(rs_), 0 if kind_ == "road" else 1))
        cor_pts = np.vstack(cp)
        cor_tan = np.vstack(ct)
        cor_arc = np.concatenate(cv)
        cor_hw = np.concatenate(chw)
        cor_kind = np.concatenate(ck)
        cor_kd = cKDTree(cor_pts)

    def corridor_mat(cx, cz):
        if cor_kd is None:
            return None
        dd, ii = cor_kd.query([cx, cz], k=3)
        for d_, i_ in zip(np.atleast_1d(dd), np.atleast_1d(ii)):
            if i_ < len(cor_hw) and d_ <= cor_hw[i_]:
                return "sideroad" if cor_kind[i_] == 0 else "water"
        return None

    def corridor_uv(x, z):
        """Path-aligned UV: U spans the corridor width (wheel tracks follow
        the road), V runs along it at one texture repeat per 8m."""
        _, i_ = cor_kd.query([x, z])
        tx_, tz_ = cor_tan[i_]
        dxp = x - cor_pts[i_][0]
        dzp = z - cor_pts[i_][1]
        lat = -tz_ * dxp + tx_ * dzp
        along = tx_ * dxp + tz_ * dzp
        u = 0.5 + lat / (2.0 * cor_hw[i_] + 0.8)
        v = (cor_arc[i_] + along) / 8.0
        return (float(u), float(v))

    def group(matkey):
        if matkey not in groups:
            groups[matkey] = [[], [], {}, 0]
        return groups[matkey]

    def flush(matkey):
        verts, idx, vmap, part = groups[matkey]
        if verts:
            if matkey == "sideroad":
                name = f"1ROAD_SIDE_{part}"      # drivable: ROAD physics
            else:
                suffix = "" if matkey == "terrain" else f"_{matkey.upper()}"
                name = f"1GRASS_TER{suffix}" if part == 0 \
                       else f"1GRASS_TER{suffix}_{part}"
            out.append((name, verts, idx, matkey))
            groups[matkey] = [[], [], {}, part + 1]

    def vid(g, k, uv=None):
        verts, idx, vmap, _ = g
        vi = vmap.get(k)
        if vi is None:
            x, y, z = pos[k]
            nx_, ny_, nz_ = nrm[k]
            vi = len(verts)
            verts.append(((float(x), float(y), float(z)),
                          (float(nx_), float(ny_), float(nz_)),
                          uv if uv is not None
                          else (float(x) / 20.0, float(z) / 20.0),
                          (1.0, 0.0, 0.0)))
            vmap[k] = vi
        return vi

    for s in keep:
        p0, p1, p2 = pos[s]
        matkey = "terrain"
        cx, cz = float((p0[0] + p1[0] + p2[0]) / 3.0), \
                 float((p0[2] + p1[2] + p2[2]) / 3.0)
        if lc is not None and inv_ll is not None:
            la, lo = inv_ll(cx, cz)
            matkey = LC_MATERIAL.get(lc(la, lo), "terrain")
        cm = corridor_mat(cx, cz)
        if cm is not None:
            matkey = cm
        g = group(matkey)
        if len(g[0]) + 3 > 60000:
            flush(matkey)
            g = group(matkey)
        gy = np.cross(p1 - p0, p2 - p0)[1]     # up-facing winding
        a, b, c = int(s[0]), int(s[1]), int(s[2])
        if matkey == "sideroad":               # path-aligned road UVs
            va = vid(g, a, corridor_uv(float(pos[a][0]), float(pos[a][2])))
            vb = vid(g, b, corridor_uv(float(pos[b][0]), float(pos[b][2])))
            vc = vid(g, c, corridor_uv(float(pos[c][0]), float(pos[c][2])))
        else:
            va, vb, vc = vid(g, a), vid(g, b), vid(g, c)
        if gy >= 0:
            g[1].extend((va, vb, vc))
        else:
            g[1].extend((va, vc, vb))
    for matkey in list(groups):
        flush(matkey)

    return out


FAR_MAX_DIST  = 500.0   # how far the visual-only backdrop terrain reaches (m)
FAR_GRID_STEP = 40.0    # backdrop grid spacing (m)


def build_far_terrain(mesh: dict, grid: dict) -> list:
    """
    Coarse VISUAL-ONLY terrain ring from the inner terrain edge out to
    FAR_MAX_DIST. Without it, a road section seen across a valley floats
    over void — the hill it sits on ends 90m from the carriageway.

    Heights use the same anchoring as the inner terrain at full blend
    (road height + DEM offset relative to the DEM at the nearest road
    point), so the two surfaces agree where they overlap; the ring is sunk
    0.6m so the one-cell overlap tucks underneath the inner mesh. Names
    carry no digit prefix → no physics.
    """
    from scipy.spatial import Delaunay, cKDTree
    rx, rz, ry = grid["rx"], grid["rz"], grid["ry"]
    dem_ref = grid["dem_ref"]
    g_drop = mesh["stats"].get("grass_drop", GRASS_DROP)
    lc = mesh.get("land_cover")
    inv_ll = mesh.get("to_latlon")
    if inv_ll is None:
        return []

    cl_xz = np.array([[p[0], p[2]] for p in mesh["centerline"]])
    kd_ref = cKDTree(np.column_stack([rx, rz]))

    step = FAR_GRID_STEP
    pad = FAR_MAX_DIST + step
    min_x = float(cl_xz[:, 0].min()) - pad
    max_x = float(cl_xz[:, 0].max()) + pad
    min_z = float(cl_xz[:, 1].min()) - pad
    max_z = float(cl_xz[:, 1].max()) + pad
    est = ((max_x - min_x) / step) * ((max_z - min_z) / step)
    if est > 45000:                        # very long roads: coarsen, not grow
        step *= math.sqrt(est / 45000.0)
    gx = np.arange(min_x, max_x + step, step)
    gz = np.arange(min_z, max_z + step, step)
    GX, GZ = np.meshgrid(gx, gz, indexing="ij")
    flat = np.column_stack([GX.ravel(), GZ.ravel()])
    d_ref, near_flat = kd_ref.query(flat)
    keep = (d_ref > TERRAIN_MAX_DIST - step) & (d_ref <= FAR_MAX_DIST)
    nodes = flat[keep]
    near = near_flat[keep].astype(int)
    d_kept = d_ref[keep]
    if len(nodes) < 3:
        return []

    latlon = [list(inv_ll(float(x), float(z))) for x, z in nodes]
    print(f"  [terrain] sampling {len(nodes)} far-terrain DEM points "
          f"(ring {TERRAIN_MAX_DIST:.0f}–{FAR_MAX_DIST:.0f}m, "
          f"{step:.0f}m grid)…")
    elevs = np.array(fetch_elevations(latlon), dtype=float)
    bad = ~np.isfinite(elevs)
    if bad.any():
        elevs[bad] = dem_ref[near[bad]]
    ys = ry[near] - g_drop + (elevs - dem_ref[near]) - 0.6

    tri = Delaunay(nodes)
    max_edge2 = (2.8 * step) ** 2
    out = []
    verts, idx, vmap, part = [], [], {}, 0

    def flush():
        nonlocal verts, idx, vmap, part
        if verts:
            out.append((f"ENV_FAR_{part}", verts, idx, "terrain"))
            part += 1
            verts, idx, vmap = [], [], {}

    def vid(k):
        vi = vmap.get(k)
        if vi is None:
            x, z = float(nodes[k][0]), float(nodes[k][1])
            vi = len(verts)
            verts.append(((x, float(ys[k]), z), (0.0, 1.0, 0.0),
                          (x / 20.0, z / 20.0), (1.0, 0.0, 0.0)))
            vmap[k] = vi
        return vi

    for s in tri.simplices:
        a, b, c = int(s[0]), int(s[1]), int(s[2])
        pa, pb, pc = nodes[a], nodes[b], nodes[c]
        if (((pa - pb) ** 2).sum() > max_edge2
                or ((pb - pc) ** 2).sum() > max_edge2
                or ((pa - pc) ** 2).sum() > max_edge2):
            continue                       # spans the road corridor / a gap
        if len(verts) + 3 > 60000:
            flush()
        va, vb, vc = vid(a), vid(b), vid(c)
        # double-sided: no underlay copy is made for ENV_ meshes
        idx.extend((va, vb, vc,  va, vc, vb))
    flush()

    # ── Distant filler trees ──
    # Where WorldCover says forest, drop cheap two-plane billboard trees on
    # the backdrop so far hillsides read as wooded rather than felt-covered —
    # the standard trick for distant landscape in AC track making. Only
    # beyond the inner terrain, where the near scatter can't reach.
    if lc is not None:
        rng_f = np.random.default_rng(17)
        fv, fi = [], []
        f_part = n_far = 0
        for k in range(len(nodes)):
            if n_far >= 6000 or d_kept[k] < TERRAIN_MAX_DIST + 15.0:
                continue
            dns = LC_TREE_DENSITY.get(lc(latlon[k][0], latlon[k][1]), 0.0)
            if dns <= 0 or rng_f.random() > dns * 0.6:
                continue
            for _ in range(int(rng_f.integers(1, 3))):
                tx = float(nodes[k][0]) + rng_f.uniform(-step/2, step/2)
                tz = float(nodes[k][1]) + rng_f.uniform(-step/2, step/2)
                ty = float(ys[k]) - 1.5          # generous sink: cell is coarse
                h = rng_f.uniform(10.0, 16.0)
                var = int(rng_f.integers(0, 4))
                w = h * TREE_ASPECT[var]
                u0, u1 = var * 0.25, var * 0.25 + 0.25
                yaw = rng_f.uniform(0.0, math.pi)
                if len(fv) + 8 > 60000:
                    out.append((f"ENV_FARTREE_{f_part}", fv, fi, "tree"))
                    fv, fi, f_part = [], [], f_part + 1
                for pi_ in range(2):             # two crossed planes
                    a_ = yaw + pi_ * math.pi / 2.0
                    ddx = math.cos(a_) * w * 0.5
                    ddz = math.sin(a_) * w * 0.5
                    v0 = len(fv)
                    fv.append(((tx-ddx, ty,   tz-ddz), (0,1,0), (u0,1), (1,0,0)))
                    fv.append(((tx+ddx, ty,   tz+ddz), (0,1,0), (u1,1), (1,0,0)))
                    fv.append(((tx+ddx, ty+h, tz+ddz), (0,1,0), (u1,0), (1,0,0)))
                    fv.append(((tx-ddx, ty+h, tz-ddz), (0,1,0), (u0,0), (1,0,0)))
                    fi.extend((v0, v0+1, v0+2,  v0, v0+2, v0+3))
                    fi.extend((v0, v0+2, v0+1,  v0, v0+3, v0+2))
                n_far += 1
        if fv:
            out.append((f"ENV_FARTREE_{f_part}", fv, fi, "tree"))
        if n_far:
            print(f"  [terrain] {n_far} distant filler trees on far forest")
    return out

def process_road(coords: list, road_width: float = 8.0,
                 smooth_factor: float = 0.3, elevations: list = None,
                 elev_profile: dict = None, grass_width: float = 10.0) -> dict:
    if len(coords) < 4:
        return {"error": "Need at least 4 points"}

    mid_lat = np.mean([c[0] for c in coords])
    mid_lon = np.mean([c[1] for c in coords])
    proj = Transformer.from_crs(
        "EPSG:4326",
        f"+proj=tmerc +lat_0={mid_lat} +lon_0={mid_lon} +units=m",
        always_xy=True
    )
    pts = np.array([proj.transform(c[1], c[0]) for c in coords])
    # AC uses a left-handed world (north = -Z). Without this negation the
    # whole track is mirrored: left turns become right turns.
    pts[:, 1] = -pts[:, 1]

    # Resample to 1m spacing
    diffs = np.diff(pts, axis=0)
    seg_len = np.sqrt((diffs**2).sum(axis=1))
    arc = np.concatenate([[0], np.cumsum(seg_len)])
    total_length = arc[-1]
    n_pts = max(int(total_length), 4)
    t_u = np.linspace(0, arc[-1], n_pts)
    x_rs = np.interp(t_u, arc, pts[:, 0])
    z_rs = np.interp(t_u, arc, pts[:, 1])

    # ── Elevation ──
    # Preferred path: distance-based profile from fetch_elevation_profile.
    # Interpolating by DISTANCE (not GPS point index) keeps gradients true.
    if elev_profile and elev_profile.get("elevs"):
        prof_d = np.array(elev_profile["dists"], dtype=float)
        prof_e = np.array(elev_profile["elevs"], dtype=float)
        # profile distances are haversine along raw coords; rescale to
        # match projected arc length (they differ by <0.1%)
        if prof_d[-1] > 0:
            prof_d = prof_d * (total_length / prof_d[-1])
        y_rs = np.interp(t_u, prof_d, prof_e)
        y_rs -= y_rs.min()
    elif elevations and len(elevations) == len(coords):
        raw_elev = np.array(elevations, dtype=float)
        raw_elev -= raw_elev.min()
        y_rs = np.interp(t_u, arc, raw_elev)
    else:
        y_rs = np.zeros(n_pts)

    # Smooth XZ planform
    smoothing = smooth_factor * n_pts
    try:
        tck, _ = splprep([x_rs, z_rs], u=t_u / max(total_length, 1e-6),
                         s=smoothing, per=False, k=3)
        xs, zs = splev(t_u / max(total_length, 1e-6), tck)
    except Exception:
        xs, zs = x_rs, z_rs

    # ── Smooth elevation ──
    # Sigma scaled to the data source. SRTM-era tiles are ~30m resolution
    # with ±1-2m noise, so sigma ≈ 30m removes quantisation steps without
    # flattening real gradients (nothing under 30m is real data anyway).
    # LiDAR has no such steps — heavy smoothing there only erases real
    # crests and compressions — so the profile carries its own sigma
    # (8m for local high-res DEMs). Gaussian (not boxcar) => continuous
    # slope => no felt "kinks" at speed.
    sigma_m = float(elev_profile.get("smooth_sigma", 30.0)) if elev_profile else 30.0
    ys = np.array(gaussian_filter1d(y_rs, sigma=sigma_m, mode='nearest'))

    # Centre at road start (origin = road start)
    ox, oz = float(xs[0]), float(zs[0])
    xs = xs - ox
    zs = zs - oz

    # Road edges
    dxs = np.gradient(xs)
    dzs = np.gradient(zs)
    lengths_xz = np.sqrt(dxs**2 + dzs**2) + 1e-9
    nx = -dzs / lengths_xz
    nz =  dxs / lengths_xz
    hw = road_width / 2.0
    gw = grass_width
    # verge drop scales with verge width (a 0.5m verge shouldn't be a cliff);
    # at grass_width=0 the drop is 0, so terrain meets the road edge flush.
    g_drop = GRASS_DROP * (grass_width / GRASS_W) if GRASS_W else 0.0

    # ── Curvature-aware offsets ──
    # An offset placed at or beyond the local centre of curvature folds the
    # edge back across itself; the folded quads become inverted, overlapping
    # collision triangles — the invisible spikes that launch/flip the car in
    # AC. Cap every inside-of-bend offset at 90% of the distance to the
    # curvature centre, so the road narrows slightly through impossibly
    # tight bends instead of folding.
    from scipy.ndimage import maximum_filter1d
    dx2 = np.gradient(dxs); dz2 = np.gradient(dzs)
    k_s = (dxs * dz2 - dzs * dx2) / (lengths_xz ** 3)
    # Windowed MAXIMUM curvature per side (±7m at 1m spacing): conservative —
    # a smoothed average would underestimate the apex and let the edge fold.
    k_left  = maximum_filter1d(np.maximum(k_s,  0.0), size=15)   # centre on +n
    k_right = maximum_filter1d(np.maximum(-k_s, 0.0), size=15)   # centre on -n

    def _safe_offsets(offset):
        """(left, right) offset distances, clamped on the inside of bends."""
        d_l = np.full(n_pts, float(offset))
        d_r = np.full(n_pts, float(offset))
        pos = k_left  > 1e-6
        neg = k_right > 1e-6
        d_l[pos] = np.minimum(offset, 0.9 / k_left[pos])
        d_r[neg] = np.minimum(offset, 0.9 / k_right[neg])
        return d_l, d_r

    sw = gw + SKIRT_W
    dl_rd, dr_rd = _safe_offsets(hw)
    dl_g,  dr_g  = _safe_offsets(hw + gw)
    dl_s,  dr_s  = _safe_offsets(hw + sw)
    lx = xs + nx*dl_rd;  lz = zs + nz*dl_rd
    rx = xs - nx*dr_rd;  rz = zs - nz*dr_rd
    # Grass outer edges
    glx = xs + nx*dl_g;  glz = zs + nz*dl_g   # left grass outer
    grx = xs - nx*dr_g;  grz = zs - nz*dr_g   # right grass outer
    # Terrain skirt outer edges (grass outer → 80m from road)
    slx_ = xs + nx*dl_s;  slz_ = zs + nz*dl_s
    srx_ = xs - nx*dr_s;  srz_ = zs - nz*dr_s

    # Safety net: repair any residual fold (e.g. a stitching kink sharper
    # than the curvature estimate resolves).
    lx,   lz   = _fix_edge_folds(lx,   lz,   dxs, dzs)
    rx,   rz   = _fix_edge_folds(rx,   rz,   dxs, dzs)
    glx,  glz  = _fix_edge_folds(glx,  glz,  dxs, dzs)
    grx,  grz  = _fix_edge_folds(grx,  grz,  dxs, dzs)
    slx_, slz_ = _fix_edge_folds(slx_, slz_, dxs, dzs)
    srx_, srz_ = _fix_edge_folds(srx_, srz_, dxs, dzs)

    # Build road mesh
    vertices, uvs, faces = [], [], []
    ts = 0.1        # texture scale for grass/skirt strips
    ts_road = 0.0125 # road: one repeat per 80m — the tall road texture packs
                    # non-repeating wear into each repeat, so the pattern
                    # doesn't telegraph every 10m like the old square tile
    for i in range(n_pts):
        uc = i / (n_pts-1) * total_length * ts_road
        vertices.append((lx[i], float(ys[i]), lz[i]))
        vertices.append((rx[i], float(ys[i]), rz[i]))
        uvs.append((0.0, uc)); uvs.append((1.0, uc))
    for i in range(n_pts - 1):
        a,b,c,d = i*2, i*2+1, i*2+2, i*2+3
        # Winding: counter-clockwise when viewed from above = normals point up
        faces.append((a+1, d+1, b+1, a+1, d+1, b+1))
        faces.append((a+1, c+1, d+1, a+1, c+1, d+1))

    # Build grass vertices (left and right strips).
    # SKIPPED entirely when grass_width is 0 (the real-terrain export path):
    # zero-width strips are degenerate zero-area PHYSICAL collision triangles
    # coincident with the road edges — in AC these behave as invisible spikes
    # that flip the car.
    grass_l_verts = []
    grass_r_verts = []
    grass_uvs = []
    grass_l_faces = []
    grass_r_faces = []

    if gw > 0:
        for i in range(n_pts):
            uc = i / (n_pts-1) * total_length * ts
            oy = float(ys[i]) - g_drop      # outer edge blends downward
            # Left grass: road left edge → grass outer left
            grass_l_verts.append((lx[i],  float(ys[i]), lz[i]))
            grass_l_verts.append((glx[i], oy,           glz[i]))
            # Right grass: road right edge → grass outer right
            grass_r_verts.append((rx[i],  float(ys[i]), rz[i]))
            grass_r_verts.append((grx[i], oy,           grz[i]))
            grass_uvs.append((0.0, uc))
            grass_uvs.append((1.0, uc))

    # Terrain skirt strips: grass outer edge → 80m out, dropping to -2m.
    # Gives the surroundings (buildings/trees) ground to stand on and
    # something to land on if you leave the road.
    skirt_l_verts, skirt_r_verts, skirt_uvs = [], [], []
    s_drop = SKIRT_DROP
    for i in range(n_pts):
        uc = i / (n_pts-1) * total_length * ts * 0.25   # bigger texture tiles
        iy = float(ys[i]) - g_drop
        oy = float(ys[i]) - s_drop
        skirt_l_verts.append((glx[i],  iy, glz[i]))
        skirt_l_verts.append((slx_[i], oy, slz_[i]))
        skirt_r_verts.append((grx[i],  iy, grz[i]))
        skirt_r_verts.append((srx_[i], oy, srz_[i]))
        skirt_uvs.append((0.0, uc))
        skirt_uvs.append((1.0, uc))

    if gw > 0:
        for i in range(n_pts - 1):
            a,b,c,d = i*2, i*2+1, i*2+2, i*2+3
            grass_l_faces.append((a+1, d+1, b+1, a+1, d+1, b+1))
            grass_l_faces.append((a+1, c+1, d+1, a+1, c+1, d+1))
            grass_r_faces.append((a+1, d+1, b+1, a+1, d+1, b+1))
            grass_r_faces.append((a+1, c+1, d+1, a+1, c+1, d+1))

    # Stats (dx2/dz2 computed above for the curvature clamp)
    curv = np.abs(k_s)
    mc   = float(np.percentile(curv, 99))
    corners = int(np.sum(np.diff((curv > mc*0.15).astype(int)) > 0))
    dys = np.diff(ys)
    has_elev = bool(elev_profile and elev_profile.get("elevs")) or \
               (elevations is not None and any(e != 0 for e in (elevations or [])))

    return {
        "vertices": vertices,
        "uvs": uvs,
        "faces": faces,
        "grass_l_verts": grass_l_verts,
        "grass_r_verts": grass_r_verts,
        "grass_uvs": grass_uvs,
        "skirt_l_verts": skirt_l_verts,
        "skirt_r_verts": skirt_r_verts,
        "skirt_uvs": skirt_uvs,
        "grass_l_faces": grass_l_faces,
        "grass_r_faces": grass_r_faces,
        "centerline": list(zip(xs.tolist(), ys.tolist(), zs.tolist())),
        "elevation_profile": ys.tolist(),
        # projection parameters so surroundings can be placed in the
        # same local coordinate space
        "proj": {"mid_lat": float(mid_lat), "mid_lon": float(mid_lon),
                 "ox": ox, "oz": oz},
        "stats": {
            "length_m":       round(total_length, 1),
            "length_km":      round(total_length / 1000, 2),
            "point_count":    n_pts,
            "corners":        corners,
            "road_width":     road_width,
            "grass_width":    grass_width,
            "grass_drop":     g_drop,
            "elev_min_m":     round(float(ys.min()), 1),
            "elev_max_m":     round(float(ys.max()), 1),
            "elev_range_m":   round(float(ys.max()-ys.min()), 1),
            "total_climb_m":  round(float(np.sum(dys[dys>0])), 1),
            "total_descent_m":round(float(np.sum(-dys[dys<0])), 1),
            "has_elevation":  has_elev,
        }
    }


# ─── OBJ Builder ──────────────────────────────────────────────────────────────

def build_obj(mesh: dict, track_name: str) -> tuple:
    """
    Build Wavefront OBJ + MTL.
    Road surface as one object (o 1ROAD).
    AC marker cubes as separate objects using correct A-to-B naming.

    For an open road (A-to-B stage), the required objects are:
      AC_AB_START_L / AC_AB_START_R  — start line left/right posts
      AC_AB_FINISH_L / AC_AB_FINISH_R — finish line left/right posts
      AC_PIT_0                        — pit/practice spawn
      AC_HOTLAP_START_0               — hotlap spawn
      AC_START_0                      — race spawn

    All cubes placed 1m above road surface, oriented with Z pointing
    in the direction of travel (AC reads heading from the cube's Z axis).
    """
    verts   = mesh["vertices"]
    uvs_raw = mesh["uvs"]
    faces   = mesh["faces"]
    cl      = mesh["centerline"]
    stats   = mesh["stats"]

    def hdg(p1, p2):
        return math.degrees(math.atan2(p2[0]-p1[0], p2[2]-p1[2]))

    hw     = stats["road_width"] / 2.0
    p0, p1 = cl[0], cl[min(1, len(cl)-1)]
    s_hdg  = hdg(p0, p1)
    hr     = math.radians(s_hdg)

    # Start line — at road start, 1m above surface, left and right of road
    sl_x = float(cl[0][0]) - math.sin(hr + math.pi/2) * hw
    sl_z = float(cl[0][2]) - math.cos(hr + math.pi/2) * hw
    sr_x = float(cl[0][0]) + math.sin(hr + math.pi/2) * hw
    sr_z = float(cl[0][2]) + math.cos(hr + math.pi/2) * hw
    start_y = float(cl[0][1]) + 1.0   # 1m above surface

    # Finish line — at road end
    pe0, pe1 = cl[-1], cl[min(len(cl)-2, len(cl)-1)]
    e_hdg = hdg(cl[-2], cl[-1])
    ehr   = math.radians(e_hdg)
    fl_x  = float(cl[-1][0]) - math.sin(ehr + math.pi/2) * hw
    fl_z  = float(cl[-1][2]) - math.cos(ehr + math.pi/2) * hw
    fr_x  = float(cl[-1][0]) + math.sin(ehr + math.pi/2) * hw
    fr_z  = float(cl[-1][2]) + math.cos(ehr + math.pi/2) * hw
    end_y = float(cl[-1][1]) + 1.0

    # Pit and spawn — 10m along road, 1m above, on road surface
    si    = min(10, len(cl)-1)
    sx    = float(cl[si][0])
    sz    = float(cl[si][2])
    sy    = float(cl[si][1]) + 1.0

    # Pit — left side of road near start
    px = float(cl[0][0]) - math.sin(hr + math.pi/2) * (hw * 0.5)
    pz = float(cl[0][2]) - math.cos(hr + math.pi/2) * (hw * 0.5)
    py = float(cl[0][1]) + 1.0

    markers = [
        ("AC_AB_START_L",    sl_x, start_y, sl_z, s_hdg),
        ("AC_AB_START_R",    sr_x, start_y, sr_z, s_hdg),
        ("AC_AB_FINISH_L",   fl_x, end_y,   fl_z, e_hdg),
        ("AC_AB_FINISH_R",   fr_x, end_y,   fr_z, e_hdg),
        ("AC_START_0",       sx,   sy,       sz,   s_hdg),
        ("AC_HOTLAP_START_0",sx,   sy,       sz,   s_hdg),
        ("AC_PIT_0",         px,   py,       pz,   s_hdg),
    ]

    lines = [
        f"# ATLAS Road World Studio - {track_name}",
        f"# A-to-B road track",
        f"mtllib {track_name}.mtl",
        "",
    ]

    # ── Road surface ──────────────────────────────────────────────────────
    lines.append(f"o 1ROAD")
    for v in verts:
        lines.append(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}")
    for u in uvs_raw:
        lines.append(f"vt {u[0]:.4f} {u[1]:.4f}")
    lines.append("vn 0.0000 1.0000 0.0000")
    lines.append("usemtl road")
    for f in faces:
        v1, v2, v3 = f[0], f[1], f[2]
        u1, u2, u3 = f[3], f[4], f[5]
        lines.append(f"f {v1}/{u1}/1 {v2}/{u2}/1 {v3}/{u3}/1")
    lines.append("")

    # ── Grass strips (left and right) ─────────────────────────────────────
    # 2m wide grass on each side of the road. Named 1GRASS so AC applies
    # the built-in GRASS surface physics (low grip, dirt).
    grass_l_verts = mesh["grass_l_verts"]
    grass_r_verts = mesh["grass_r_verts"]
    grass_uvs_raw = mesh["grass_uvs"]
    grass_l_faces = mesh["grass_l_faces"]
    grass_r_faces = mesh["grass_r_faces"]

    base_v = len(verts) + 1
    base_vt = len(uvs_raw) + 1

    if not grass_l_verts:
        obj_content = "\n".join(lines)
        return obj_content, _build_mtl()

    # Left grass
    lines.append("o 1GRASS_L")
    for v in grass_l_verts:
        lines.append(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}")
    for u in grass_uvs_raw:
        lines.append(f"vt {u[0]:.4f} {u[1]:.4f}")
    lines.append("vn 0.0000 1.0000 0.0000")
    lines.append("usemtl grass")
    for f in grass_l_faces:
        v1 = f[0] + base_v - 1
        v2 = f[1] + base_v - 1
        v3 = f[2] + base_v - 1
        u1 = f[3] + base_vt - 1
        u2 = f[4] + base_vt - 1
        u3 = f[5] + base_vt - 1
        lines.append(f"f {v1}/{u1}/2 {v2}/{u2}/2 {v3}/{u3}/2")
    lines.append("")

    base_v += len(grass_l_verts)
    base_vt += len(grass_uvs_raw)

    # Right grass
    lines.append("o 1GRASS_R")
    for v in grass_r_verts:
        lines.append(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}")
    for u in grass_uvs_raw:
        lines.append(f"vt {u[0]:.4f} {u[1]:.4f}")
    lines.append("vn 0.0000 1.0000 0.0000")
    lines.append("usemtl grass")
    for f in grass_r_faces:
        v1 = f[0] + base_v - 1
        v2 = f[1] + base_v - 1
        v3 = f[2] + base_v - 1
        u1 = f[3] + base_vt - 1
        u2 = f[4] + base_vt - 1
        u3 = f[5] + base_vt - 1
        lines.append(f"f {v1}/{u1}/3 {v2}/{u2}/3 {v3}/{u3}/3")
    lines.append("")

    # Update base_v for potential future objects
    base_v += len(grass_r_verts)

    # NOTE: AC spawn points are NOT in the OBJ file.
    # They are created as Blender empties by setup_in_blender.py.
    # OBJ cannot store object transforms, and AC reads spawn position
    # from the node transform — so empties must be created in Blender.

    obj_content = "\n".join(lines)
    return obj_content, _build_mtl()


def _build_mtl() -> str:
    return (
        "# ATLAS Road World Studio\n"
        "newmtl road\n"
        "Ka 0.2 0.2 0.2\n"
        "Kd 0.5 0.5 0.5\n"
        "Ks 0.0 0.0 0.0\n"
        "d 1.0\n"
        "illum 1\n"
        "map_Kd road.png\n"
        "\n"
        "newmtl grass\n"
        "Ka 0.1 0.2 0.1\n"
        "Kd 0.2 0.5 0.2\n"
        "Ks 0.0 0.0 0.0\n"
        "d 1.0\n"
        "illum 1\n"
        "map_Kd grass.png\n"
    )





# ─── Data Files ───────────────────────────────────────────────────────────────

def build_surfaces_ini() -> str:
    # Meshes are named 1ROAD / 1GRASS_L / 1GRASS_R, which bind to AC's
    # BUILT-IN default surfaces (KEY=ROAD, KEY=GRASS) defined in
    # assettocorsa/system/data/surfaces.ini — correct grip, sounds and
    # FMOD events out of the box.
    #
    # Redefining ROAD/GRASS here previously caused FMOD errors
    # (event:/surfaces/... not found) because of bad WAV= values.
    # Only WALL is custom; road and grass retain the system defaults.
    return (
        "; ATLAS Road World Studio — default road/grass and custom barriers.\n"
        "; Mesh prefixes 1ROAD / 1GRASS use AC's built-in default surfaces\n"
        "; from system/data/surfaces.ini (correct grip + sounds).\n"
        "; Physical roadside barriers use the custom WALL key.\n"
        "[SURFACE_0]\nKEY=WALL\nFRICTION=0.8\nDAMPING=0\n"
        "WAV=\nWAV_PITCH=0\nFF_EFFECT=NULL\nDIRT_ADDITIVE=0\n"
        "IS_VALID_TRACK=0\nBLACK_FLAG_TIME=0\nSIN_HEIGHT=0\nSIN_LENGTH=0\n"
        "IS_PITLANE=0\nVIBRATION_GAIN=0\nVIBRATION_LENGTH=0\n"
    )


def build_track_json(track_name: str, stats: dict) -> str:
    import json as _j
    name = track_name.replace('_', ' ').title()
    elev = f" Elevation {stats.get('elev_range_m',0):.0f}m range." if stats.get('has_elevation') else ''
    return _j.dumps({
        "name": name,
        "description": f"Generated from OpenStreetMap. {stats.get('length_km',0)} km.{elev}",
        "tags": ["generated", "osm", "road"],
        "geotags": [], "country": "", "city": "",
        "length":   str(int(stats.get('length_m', 1000))),
        "width":    str(int(stats.get('road_width', 8))),
        "pitboxes": "1", "run": "",
        "author": "ATLAS Road World Studio", "version": "1.0", "url": ""
    }, indent=2)


# ─── PNG Encoding (stdlib only) ───────────────────────────────────────────────

def _png_encode(width: int, height: int, pixels: bytes, rgba: bool = False) -> bytes:
    """Encode raw pixel bytes (RGB or RGBA rows, no filter bytes) as PNG."""
    import zlib as _zlib
    def chunk(name, data):
        c = name + data
        return struct.pack('>I', len(data)) + c + struct.pack('>I', _zlib.crc32(c) & 0xffffffff)
    bpp = 4 if rgba else 3
    color_type = 6 if rgba else 2
    stride = width * bpp
    rows = b''.join(
        b'\x00' + pixels[y*stride:(y+1)*stride] for y in range(height)
    )
    return (b'\x89PNG\r\n\x1a\n'
            + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, color_type, 0, 0, 0))
            + chunk(b'IDAT', _zlib.compress(rows, 6))
            + chunk(b'IEND', b''))


# ─── Procedural Textures ──────────────────────────────────────────────────────

def _pnoise(size: int, cells: int, rng) -> np.ndarray:
    """Tileable value noise: a cells×cells random grid, bilinearly upsampled
    with wrap-around, so the texture has no seam when tiled."""
    g = rng.standard_normal((cells, cells))
    gg = np.pad(g, ((0, 1), (0, 1)), mode='wrap')
    t = np.linspace(0.0, cells, size, endpoint=False)
    i = np.floor(t).astype(int)
    f = t - i
    a00 = gg[np.ix_(i,     i)];     a10 = gg[np.ix_(i + 1, i)]
    a01 = gg[np.ix_(i,     i + 1)]; a11 = gg[np.ix_(i + 1, i + 1)]
    fx = f[:, None]; fy = f[None, :]
    return a00*(1-fx)*(1-fy) + a10*fx*(1-fy) + a01*(1-fx)*fy + a11*fx*fy


def _fbm(size: int, rng, octaves=((4, 1.0), (8, 0.55), (16, 0.3), (64, 0.15))) -> np.ndarray:
    """Multi-octave tileable noise, roughly in [-1, 1]. Coarse octaves give
    patchiness, fine ones give surface detail — single-octave noise is what
    made the old ground textures read as flat colour."""
    out = np.zeros((size, size))
    total = 0.0
    for cells, amp in octaves:
        out += _pnoise(size, min(cells, size // 2), rng) * amp
        total += amp
    return out / (total * 1.6)


ROAD_STYLES = ("sealed", "highway", "rural", "gravel")

def _classify_road_style(road_type, surface) -> str:
    """Map the selected road's OSM highway/surface tags to a texture style
    (Australian conventions; other countries can hook in here later)."""
    s = (surface or "").strip().lower()
    if s in ("gravel", "unpaved", "dirt", "fine_gravel", "compacted",
             "ground", "earth", "sand") or road_type == "track":
        return "gravel"
    if road_type in ("motorway", "trunk"):
        return "highway"
    if road_type in ("tertiary", "residential", "unclassified",
                     "living_street", "service"):
        return "rural"
    return "sealed"           # primary/secondary/unknown


def build_road_texture(size: int = 256, style: str = "sealed", markings: bool = True) -> bytes:
    """Drivable-road surface in Australian marking conventions, chosen by the
    selected road's OSM highway/surface tags. 256 wide × 1024 tall = one
    repeat per 40m of road, so wear features (tar snakes, repair patches) are
    segmented and scattered instead of telegraphing every tile.
    U = across road, V = along.
      sealed  — primary/secondary: white edge lines + dashed white centre
                line (AU centre lines are white, not yellow)
      highway — motorway/trunk: fresher darker asphalt, cleanest surface
      rural   — minor sealed roads: aged patchy bitumen, faded broken edge
                lines, NO centre line, heavier wear
      gravel  — unsealed/track: red-brown laterite with compacted wheel
                tracks and stone speckle, no markings
    """
    W, H = size, size * 8
    rng = np.random.default_rng(42)
    lum = _fbm(H, rng)[:, :W]          # periodic vertically → seamless tiling
    xs = np.arange(W)

    if style == "gravel":
        img = np.empty((H, W, 3), dtype=np.float64)
        img[:, :, 0] = 148 + lum * 26
        img[:, :, 1] = 106 + lum * 22
        img[:, :, 2] = 76 + lum * 18
        for cx_f in (0.30, 0.70):          # compacted wheel tracks: lighter
            wtd = np.exp(-((xs / W - cx_f) / 0.055) ** 2)
            img += wtd[None, :, None] * np.array([16.0, 14.0, 12.0])
        dark = rng.random((H, W)) < 0.02       # stone speckle
        lite = rng.random((H, W)) < 0.015
        img[dark] -= 34
        img[lite] += 30
        return _png_encode(W, H,
                           np.clip(img, 0, 255).astype(np.uint8).tobytes())

    base = {"sealed": (57, 57, 60), "highway": (48, 48, 53),
            "rural": (74, 73, 74)}.get(style, (57, 57, 60))
    img = np.empty((H, W, 3), dtype=np.float64)
    amp = 26 if style == "rural" else 12            # old bitumen is patchier
    for ch in range(3):
        img[:, :, ch] = base[ch] + lum * amp
    if style == "rural":                            # darker resurfaced areas
        patch = _fbm(H, rng, octaves=((3, 1.0), (6, 0.5)))[:, :W]
        img -= np.clip((patch - 0.25) * 3.0, 0, 1)[:, :, None] * 18.0

    grime = _fbm(H, rng, octaves=((5, 1.0), (20, 0.5)))[:, :W]
    for cx_f in (0.30, 0.70):                       # tyre-polished strips,
        wtd = np.exp(-((xs / W - cx_f) / 0.07) ** 2)      # patchy not uniform
        img -= (wtd[None, :] * (6.0 + np.clip(grime, 0, 1) * 8.0))[:, :, None]

    # Tar snakes: SEGMENTED crack-seal lines that fade in and out, scattered
    # over the 40m repeat — full-length squiggles repeated every tile is
    # exactly what read as a pattern.
    n_snakes = {"rural": 13, "sealed": 5}.get(style, 0)
    for _ in range(n_snakes):
        ln = int(rng.uniform(H * 0.12, H * 0.3))
        r0 = int(rng.uniform(0, H - ln))
        col = float(rng.uniform(W * 0.15, W * 0.85))
        for rr in range(ln):
            col = min(max(col + rng.uniform(-1.4, 1.4), 2), W - 4)
            fade = min(1.0, rr / 24.0, (ln - rr) / 24.0)   # soft ends
            img[r0 + rr, int(col):int(col) + 2, :] -= 18.0 * fade

    # Repair patches: straight-edged rectangles of slightly different
    # asphalt — one per vertical band so they never stack on each other
    n_patch = {"rural": 5, "sealed": 2}.get(style, 0)
    for pi_ in range(n_patch):
        band = H // max(n_patch, 1)
        ph = int(rng.uniform(40, min(110, band - 10)))
        pw = int(rng.uniform(W * 0.25, W * 0.6))
        r0 = pi_ * band + int(rng.uniform(0, band - ph))
        c0 = int(rng.uniform(W * 0.12, W * 0.85 - pw))
        img[r0:r0+ph, c0:c0+pw, :] -= 7.0
        img[r0:r0+2, c0:c0+pw, :] -= 12.0             # seam edges
        img[r0+ph-2:r0+ph, c0:c0+pw, :] -= 12.0
        img[r0:r0+ph, c0:c0+2, :] -= 12.0
        img[r0:r0+ph, c0+pw-2:c0+pw, :] -= 12.0
    img = np.clip(img, 0, 255).astype(np.uint8)

    if not markings:
        return _png_encode(W, H, img.tobytes())

    # Edge lines (faded on rural, with broken/worn-away segments)
    e0 = max(2, W // 26)
    ew = max(2, W // 64)
    edge_c = 170 if style == "rural" else 225
    keep = slice(None)
    if style == "rural":
        seg = np.repeat(rng.random(32) > 0.22, H // 32)   # missing chunks
        keep = np.resize(seg, H)
    img[keep, e0:e0 + ew, :] = edge_c
    img[keep, W - e0 - ew:W - e0, :] = edge_c

    # Dashed white centre line: 10m cycle (~3m mark / 7m gap), 4 cycles per
    # repeat — none on rural back roads
    if style != "rural":
        c = W // 2
        cw = max(1, W // 128)
        cyc = H // 8
        mark = int(cyc * 0.3)
        for k in range(8):
            img[k * cyc:k * cyc + mark, c - cw - 1:c + cw + 1, :] = 210

    return _png_encode(W, H, img.tobytes())



def build_sideroad_texture(size: int = 256) -> bytes:
    """Side-road surface: worn unmarked bitumen with lighter wheel tracks
    running along V and gravelly fading edges at the U extremes. Painted
    terrain triangles carry path-aligned UVs (U across the corridor,
    V along it), so the tracks follow the road. No painted markings —
    these are lanes and minor roads where wrong markings would jar.
    Tileable along V."""
    rng = np.random.default_rng(83)
    lum = _fbm(size, rng)
    xs = np.arange(size)
    img = np.empty((size, size, 3), dtype=np.float64)
    img[:, :, 0] = 62 + lum * 18
    img[:, :, 1] = 62 + lum * 18
    img[:, :, 2] = 65 + lum * 18
    for cx_f in (0.32, 0.68):                    # wheel tracks: worn lighter
        wtd = np.exp(-((xs / size - cx_f) / 0.08) ** 2)
        img += wtd[None, :, None] * 9.0
    edge = np.clip((np.abs(xs / size - 0.5) - 0.40) / 0.10, 0, 1)
    img += (edge[None, :, None]
            * (np.array([44.0, 32.0, 10.0])[None, None, :]
               * (0.6 + 0.4 * np.clip(lum, -1, 1))[:, :, None]))
    return _png_encode(size, size,
                       np.clip(img, 0, 255).astype(np.uint8).tobytes())


def build_grass_texture(size: int = 256) -> bytes:
    """Mown verge grass: fBm luminance patches, a second fBm shifting hue
    between yellow-green and blue-green, and sparse bright blade speckles."""
    rng = np.random.default_rng(7)
    lum  = _fbm(size, rng)
    hue  = _fbm(size, rng, octaves=((3, 1.0), (9, 0.5)))
    img = np.empty((size, size, 3), dtype=np.int16)
    img[:, :, 0] = 64 + lum * 22 + hue * 12
    img[:, :, 1] = 102 + lum * 26 + hue * 9
    img[:, :, 2] = 46 + lum * 16 - hue * 10
    speck = rng.random((size, size)) < 0.012
    img[speck] = np.clip(img[speck] + 34, 0, 255)
    return _png_encode(size, size, np.clip(img, 0, 255).astype(np.uint8).tobytes())


def build_building_texture(size: int = 128) -> bytes:
    """Cream weatherboard house wall: board lines + one sparse window row.
    The old dense office window grid is most of why buildings read as
    industrial blocks instead of houses."""
    rng = np.random.default_rng(11)
    noise = rng.integers(-5, 5, size=(size, size, 1))
    base = np.full((size, size, 3), (214, 206, 190), dtype=np.int16) + noise
    img = np.clip(base, 0, 255).astype(np.uint8)
    for by in range(0, size, 10):                     # weatherboard lines
        img[by:by+1, :, :] = np.clip(img[by:by+1, :, :].astype(int) - 18, 0, 255)
    # one row of house windows, mid-wall, widely spaced, with frames
    wy0, wh, ww, step = size//2 - 11, 22, 16, 44
    for wx in range(10, size - ww, step):
        img[wy0-2:wy0+wh+2, wx-2:wx+ww+2, :] = (236, 232, 224)   # frame
        img[wy0:wy0+wh, wx:wx+ww, :] = (62, 76, 92)              # glass
        img[wy0+wh//2:wy0+wh//2+2, wx:wx+ww, :] = (236, 232, 224)  # sash
    return _png_encode(size, size, img.tobytes())


def build_building2_texture(size: int = 128) -> bytes:
    """Red-brick house wall variant with the same sparse window row."""
    rng = np.random.default_rng(29)
    noise = rng.integers(-10, 10, size=(size, size, 1))
    base = np.full((size, size, 3), (158, 106, 84), dtype=np.int16) + noise
    img = np.clip(base, 0, 255).astype(np.uint8)
    for by in range(0, size, 6):                      # mortar courses
        img[by:by+1, :, :] = np.clip(img[by:by+1, :, :].astype(int) + 22, 0, 255)
    wy0, wh, ww, step = size//2 - 11, 22, 16, 44
    for wx in range(10, size - ww, step):
        img[wy0-2:wy0+wh+2, wx-2:wx+ww+2, :] = (228, 222, 210)
        img[wy0:wy0+wh, wx:wx+ww, :] = (56, 70, 86)
        img[wy0+wh//2:wy0+wh//2+2, wx:wx+ww, :] = (228, 222, 210)
    return _png_encode(size, size, img.tobytes())


def build_building3_texture(size: int = 128) -> bytes:
    """Pale sage painted-render house wall, same sparse window row — a third
    house variant so streets don't alternate two walls A/B/A/B."""
    rng = np.random.default_rng(61)
    noise = rng.integers(-6, 6, size=(size, size, 1))
    base = np.full((size, size, 3), (196, 200, 184), dtype=np.int16) + noise
    img = np.clip(base, 0, 255).astype(np.uint8)
    wy0, wh, ww, step = size//2 - 11, 22, 16, 44
    for wx in range(10, size - ww, step):
        img[wy0-2:wy0+wh+2, wx-2:wx+ww+2, :] = (232, 230, 222)
        img[wy0:wy0+wh, wx:wx+ww, :] = (58, 72, 88)
        img[wy0+wh//2:wy0+wh//2+2, wx:wx+ww, :] = (232, 230, 222)
    return _png_encode(size, size, img.tobytes())


def build_commercial_texture(size: int = 128) -> bytes:
    """Commercial/large-building wall: concrete panels with a full-width
    glazing band. One texture tile = one storey (wall UVs tile vertically
    per ~3.2m of height), so tall buildings get a window band per floor."""
    rng = np.random.default_rng(71)
    noise = rng.integers(-6, 6, size=(size, size, 1))
    base = np.full((size, size, 3), (168, 166, 160), dtype=np.int16) + noise
    img = np.clip(base, 0, 255).astype(np.uint8)
    for by in range(0, size, size // 4):              # panel joints
        img[by:by+1, :, :] = np.clip(img[by:by+1, :, :].astype(int) - 24, 0, 255)
    wy0, wh = size//2 - 16, 34                        # glazing band
    img[wy0-2:wy0+wh+2, :, :] = (188, 188, 184)       # frame strip
    img[wy0:wy0+wh, :, :] = (52, 64, 82)              # glass
    for wx in range(0, size, 22):                     # mullions
        img[wy0:wy0+wh, wx:wx+2, :] = (150, 150, 148)
    return _png_encode(size, size, img.tobytes())


def build_asphalt_texture(size: int = 128) -> bytes:
    """Plain unmarked asphalt for adjacent roads (no centre/edge lines)."""
    rng = np.random.default_rng(97)
    noise = rng.integers(-8, 8, size=(size, size, 1))
    base = np.full((size, size, 3), (62, 62, 65), dtype=np.int16) + noise
    return _png_encode(size, size, np.clip(base, 0, 255).astype(np.uint8).tobytes())


def build_roof_texture(size: int = 64) -> bytes:
    """Muted terracotta tile — houses, not warehouses. Faint course lines."""
    rng = np.random.default_rng(83)
    noise = rng.integers(-12, 12, size=(size, size, 1))
    base = np.full((size, size, 3), (164, 92, 68), dtype=np.int16) + noise
    img = np.clip(base, 0, 255).astype(np.uint8)
    for by in range(0, size, 8):
        img[by:by+1, :, :] = np.clip(img[by:by+1, :, :].astype(int) - 20, 0, 255)
    return _png_encode(size, size, img.tobytes())


TREE_ASPECT  = (0.88, 0.70, 0.55, 1.02)   # broadleaf variant width/height
TREE2_ASPECT = (0.46, 0.32, 0.58, 0.28)   # conifer variant width/height


def _paint_broadleaf(size, rng, canopy_cy, canopy_rx, canopy_ry, clumps,
                     clump_r, foliage, trunk_rgb, trunk_w, sparse=0.45):
    """One broadleaf silhouette tile (RGBA float array). Image top = tree top."""
    from scipy.ndimage import binary_erosion
    yy, xx = np.mgrid[0:size, 0:size].astype(float)
    cx = size / 2.0
    canopy = np.zeros((size, size), dtype=bool)
    for _ in range(clumps):
        ang = rng.uniform(0, 2 * math.pi)
        rad = rng.uniform(0, 1) ** 0.55
        ex = cx + math.cos(ang) * rad * size * canopy_rx
        ey = size * canopy_cy + math.sin(ang) * rad * size * canopy_ry
        r = rng.uniform(clump_r * 0.55, clump_r) * size
        canopy |= (xx - ex) ** 2 + (yy - ey) ** 2 < r * r
    trunk = np.zeros((size, size), dtype=bool)
    t_top = canopy_cy + canopy_ry * 0.3
    for row in range(int(size * t_top), size):
        t = (row / size - t_top) / (1.0 - t_top)
        hwd = trunk_w * (0.45 + 0.55 * t) * size
        trunk[row, int(cx - hwd):int(cx + hwd)] = True
    for sgn in (-1, 1):                              # two branches
        for step in range(int(size * 0.2)):
            row = int(size * (t_top + 0.1)) - step
            col = int(cx + sgn * step * 0.7)
            if 0 <= row < size and 2 <= col < size - 2:
                trunk[row, col - 2:col + 2] = True
    alpha = canopy | trunk
    edge = alpha & ~binary_erosion(alpha, iterations=2)
    alpha &= ~(edge & (rng.random((size, size)) < sparse))
    lum = _fbm(size, rng)
    img = np.empty((size, size, 4), dtype=np.float64)
    depth = np.clip((yy / size - 0.15) * 1.4, 0, 1)
    img[:, :, 0] = foliage[0] + lum * 30 - depth * 16
    img[:, :, 1] = foliage[1] + lum * 34 - depth * 22
    img[:, :, 2] = foliage[2] + lum * 20 - depth * 12
    tv = trunk & ~binary_erosion(canopy, iterations=4)
    for ch in range(3):
        img[:, :, ch][tv] = trunk_rgb[ch] + lum[tv] * 16
    img[:, :, 3] = np.where(alpha, 255, 0)
    return img


def _paint_conifer(size, rng, tiers, width, foliage, top=0.03, bot=0.86):
    """One conifer silhouette tile (RGBA float array)."""
    from scipy.ndimage import binary_erosion
    yy, xx = np.mgrid[0:size, 0:size].astype(float)
    cx = size / 2.0
    canopy = np.zeros((size, size), dtype=bool)
    for row in range(int(size * top), int(size * bot)):
        t = (row / size - top) / (bot - top)
        tier_ph = (t * tiers) % 1.0
        w = size * width * (0.12 + 0.88 * t) * (1.0 - 0.32 * tier_ph)
        w *= rng.uniform(0.92, 1.08)
        canopy[row, int(cx - w):int(cx + w)] = True
    trunk = np.zeros((size, size), dtype=bool)
    hwd = int(size * 0.022)
    trunk[int(size * (bot - 0.06)):, int(cx - hwd):int(cx + hwd)] = True
    alpha = canopy | trunk
    edge = alpha & ~binary_erosion(alpha, iterations=2)
    alpha &= ~(edge & (rng.random((size, size)) < 0.5))
    lum = _fbm(size, rng)
    img = np.empty((size, size, 4), dtype=np.float64)
    depth = np.clip((yy / size - 0.1) * 1.2, 0, 1)
    img[:, :, 0] = foliage[0] + lum * 22 - depth * 12
    img[:, :, 1] = foliage[1] + lum * 26 - depth * 18
    img[:, :, 2] = foliage[2] + lum * 16 - depth * 10
    tv = trunk & ~canopy
    img[tv, 0] = 82; img[tv, 1] = 62; img[tv, 2] = 46
    img[:, :, 3] = np.where(alpha, 255, 0)
    return img


def build_tree_texture(size: int = 256) -> bytes:
    """Broadleaf impostor atlas: 4 variants side by side (u = variant/4 …).
    Repetition of a single silhouette is what reads as fake, so quads pick a
    random column. Variants: round oak-ish, tall irregular, gum (pale trunk,
    sparse grey-green canopy), wide spreading."""
    tiles = [
        _paint_broadleaf(size, np.random.default_rng(23), 0.34, 0.36, 0.26,
                         70, 0.13, (62, 96, 48), (88, 66, 48), 0.022),
        _paint_broadleaf(size, np.random.default_rng(29), 0.30, 0.28, 0.30,
                         55, 0.11, (70, 92, 44), (92, 72, 52), 0.020),
        _paint_broadleaf(size, np.random.default_rng(37), 0.26, 0.30, 0.24,
                         34, 0.09, (96, 108, 78), (196, 182, 158), 0.014,
                         sparse=0.62),                       # eucalypt
        _paint_broadleaf(size, np.random.default_rng(43), 0.38, 0.42, 0.22,
                         80, 0.12, (58, 100, 50), (84, 62, 44), 0.026),
    ]
    atlas = np.concatenate(tiles, axis=1)
    return _png_encode(size * 4, size,
                       np.clip(atlas, 0, 255).astype(np.uint8).tobytes(),
                       rgba=True)


def build_tree2_texture(size: int = 256) -> bytes:
    """Conifer impostor atlas: 4 variants (pine, tall spruce, broad cypress,
    wispy casuarina-ish)."""
    tiles = [
        _paint_conifer(size, np.random.default_rng(41), 7, 0.40, (48, 72, 44)),
        _paint_conifer(size, np.random.default_rng(47), 9, 0.30, (42, 64, 42)),
        _paint_conifer(size, np.random.default_rng(53), 5, 0.46, (56, 78, 48)),
        _paint_conifer(size, np.random.default_rng(59), 11, 0.26, (58, 70, 52),
                       top=0.05, bot=0.90),
    ]
    atlas = np.concatenate(tiles, axis=1)
    return _png_encode(size * 4, size,
                       np.clip(atlas, 0, 255).astype(np.uint8).tobytes(),
                       rgba=True)


def build_treeshadow_texture(size: int = 64) -> bytes:
    """Soft radial shadow blob laid under each tree. Real-time shadows fade
    with distance, so a baked ground shadow is what keeps trees looking
    anchored — standard AC track-making practice."""
    yy, xx = np.mgrid[0:size, 0:size].astype(float)
    r = np.sqrt((xx - size/2)**2 + (yy - size/2)**2) / (size/2)
    a = np.clip(1.0 - r, 0, 1) ** 1.6 * 120.0
    img = np.zeros((size, size, 4), dtype=np.uint8)
    img[:, :, 3] = a.astype(np.uint8)                # RGB stays black
    return _png_encode(size, size, img.tobytes(), rgba=True)


def build_terrain_texture(size: int = 256) -> bytes:
    """Scrubby hillside: green base with dry-earth patches where the coarse
    noise peaks, so open ground reads as mottled paddock rather than felt."""
    rng = np.random.default_rng(31)
    lum = _fbm(size, rng)
    patch = _fbm(size, rng, octaves=((3, 1.0), (6, 0.6)))
    img = np.empty((size, size, 3), dtype=np.float64)
    img[:, :, 0] = 82 + lum * 26
    img[:, :, 1] = 96 + lum * 26
    img[:, :, 2] = 54 + lum * 16
    w = np.clip((patch - 0.18) * 3.0, 0.0, 1.0)[:, :, None]   # dry patches
    img = img * (1 - w) + np.array([126.0, 108.0, 76.0]) * w
    return _png_encode(size, size, np.clip(img, 0, 255).astype(np.uint8).tobytes())


def build_forest_texture(size: int = 256) -> bytes:
    """Dark leaf-litter floor: fBm shade + brown litter speckle."""
    rng = np.random.default_rng(41)
    lum = _fbm(size, rng)
    img = np.empty((size, size, 3), dtype=np.int16)
    img[:, :, 0] = 46 + lum * 18
    img[:, :, 1] = 62 + lum * 20
    img[:, :, 2] = 36 + lum * 12
    speck = rng.random((size, size)) < 0.02
    img[speck] = (86, 70, 48)
    return _png_encode(size, size, np.clip(img, 0, 255).astype(np.uint8).tobytes())


def build_dirt_texture(size: int = 256) -> bytes:
    """Bare / built-up ground: dry earth with fBm mottling."""
    rng = np.random.default_rng(53)
    lum = _fbm(size, rng)
    img = np.empty((size, size, 3), dtype=np.int16)
    img[:, :, 0] = 132 + lum * 24
    img[:, :, 1] = 110 + lum * 20
    img[:, :, 2] = 84 + lum * 16
    return _png_encode(size, size, np.clip(img, 0, 255).astype(np.uint8).tobytes())


def build_water_texture(size: int = 128) -> bytes:
    rng = np.random.default_rng(67)
    noise = rng.integers(-8, 8, size=(size, size, 1))
    base = np.full((size, size, 3), (46, 78, 112), dtype=np.int16) + noise
    return _png_encode(size, size, np.clip(base, 0, 255).astype(np.uint8).tobytes())


# ─── Track Map (map.png + map.ini + ui images) ────────────────────────────────

def _render_route(mesh: dict, size: int, margin: int, line_px: float,
                  bg=(0, 0, 0, 0), fg=(255, 255, 255, 255)) -> tuple:
    """Render top-down route. Returns (rgba_bytes, min_x, min_z, scale)."""
    surface = mesh.get("network_surface")
    if surface is not None and not surface.is_empty:
        from PIL import Image, ImageDraw
        from road_network_v3 import _polygons
        min_x, min_z, max_x, max_z = surface.bounds
        scale = max(max_x-min_x, max_z-min_z, 1.0) / (size-2*margin)
        image = Image.new("RGBA", (size,size), bg)
        draw = ImageDraw.Draw(image)
        def pixels(ring):
            return [((x-min_x)/scale+margin, (z-min_z)/scale+margin) for x,z in ring.coords]
        for poly in _polygons(surface):
            draw.polygon(pixels(poly.exterior), fill=fg)
            for hole in poly.interiors:
                draw.polygon(pixels(hole), fill=bg)
        return image.tobytes(), min_x, min_z, scale
    cl = mesh["centerline"]
    xs = np.array([p[0] for p in cl]); zs = np.array([p[2] for p in cl])
    min_x, max_x = float(xs.min()), float(xs.max())
    min_z, max_z = float(zs.min()), float(zs.max())
    ext = max(max_x - min_x, max_z - min_z, 1.0)
    scale = ext / (size - 2 * margin)          # metres per pixel

    canvas = np.zeros((size, size, 4), dtype=np.uint8)
    canvas[:, :] = bg

    px = ((xs - min_x) / scale + margin).astype(int)
    py = ((zs - min_z) / scale + margin).astype(int)

    r = max(1, int(round(line_px / 2)))
    yy, xx = np.mgrid[-r:r+1, -r:r+1]
    disk = (xx**2 + yy**2) <= r*r
    dy, dx = np.nonzero(disk)
    dy = dy - r; dx = dx - r

    for cx, cy in zip(px, py):
        ys_ = np.clip(cy + dy, 0, size-1)
        xs_ = np.clip(cx + dx, 0, size-1)
        canvas[ys_, xs_] = fg

    return canvas.tobytes(), min_x, min_z, scale


def build_track_map(mesh: dict) -> tuple:
    """Returns (map_png, map_ini, outline_png, preview_png)."""
    size, margin = 1024, 32
    road_w = mesh["stats"]["road_width"]

    # First pass to get scale, second pass with road-width-proportional line
    _, min_x, min_z, scale = _render_route(mesh, size, margin, 1)
    line_px = max(4.0, road_w / scale)
    rgba, min_x, min_z, scale = _render_route(mesh, size, margin, line_px)
    map_png = _png_encode(size, size, rgba, rgba=True)

    map_ini = (
        "[PARAMETERS]\n"
        f"WIDTH={size}\n"
        f"HEIGHT={size}\n"
        f"X_OFFSET={margin * scale - min_x:.3f}\n"
        f"Z_OFFSET={margin * scale - min_z:.3f}\n"
        "MARGIN=20\n"
        f"SCALE_FACTOR={scale:.5f}\n"
        "MAX_SIZE=1600\n"
        "MIN_SIZE=300\n"
        "DRAWING_SIZE=10\n"
    )

    # ui/outline.png — route on transparent, smaller
    o_rgba, *_ = _render_route(mesh, 512, 24, 6)
    outline_png = _png_encode(512, 512, o_rgba, rgba=True)

    # ui/preview.png — route on dark background
    p_rgba, *_ = _render_route(mesh, 565, 40, 8,
                               bg=(24, 26, 30, 255), fg=(240, 240, 240, 255))
    preview_png = _png_encode(565, 565, p_rgba, rgba=True)

    return map_png, map_ini, outline_png, preview_png


# ─── KN5 Writer (pure Python — no Blender, no ksEditor) ───────────────────────
# Binary format of Assetto Corsa's KN5 model files, as implemented by the
# open-source GPL Blender exporters (Thomas Hagnhofer / moppius fork).
# Reimplemented here for direct export from this tool.

class _KN5:
    def __init__(self):
        self.buf = io.BytesIO()

    def u32(self, v):  self.buf.write(struct.pack('<I', int(v)))
    def i32(self, v):  self.buf.write(struct.pack('<i', int(v)))
    def u16(self, v):  self.buf.write(struct.pack('<H', int(v)))
    def f32(self, v):  self.buf.write(struct.pack('<f', float(v)))
    def byte(self, v): self.buf.write(struct.pack('<B', int(v)))
    def flag(self, v): self.buf.write(struct.pack('<?', bool(v)))
    def s(self, text):
        b = text.encode('utf-8')
        self.u32(len(b)); self.buf.write(b)
    def blob(self, b):
        self.u32(len(b)); self.buf.write(b)
    def v2(self, v): self.buf.write(struct.pack('<2f', *[float(x) for x in v]))
    def v3(self, v): self.buf.write(struct.pack('<3f', *[float(x) for x in v]))
    def v4(self, v): self.buf.write(struct.pack('<4f', *[float(x) for x in v]))

    # D3D row-major: rows are basis vectors [right, up, forward, position]
    def matrix(self, right, up, fwd, pos):
        for vec, w in ((right, 0.0), (up, 0.0), (fwd, 0.0), (pos, 1.0)):
            self.f32(vec[0]); self.f32(vec[1]); self.f32(vec[2]); self.f32(w)

    def material(self, name, shader, texture_name,
                 ambient=0.5, diffuse=0.45, specular=0.05, spec_exp=20.0,
                 alpha_tested=False, alpha_blend=0):
        self.s(name)
        self.s(shader)
        self.byte(alpha_blend)    # alphaBlendMode: 0 opaque, 1 blend
        self.flag(alpha_tested)   # alphaTested (cutout, e.g. tree impostors)
        self.i32(0)       # depthMode: DepthNormal
        props = [("ksAmbient", ambient), ("ksDiffuse", diffuse),
                 ("ksSpecular", specular), ("ksSpecularEXP", spec_exp)]
        self.u32(len(props))
        for pname, a in props:
            self.s(pname)
            self.f32(a)
            self.v2((0, 0)); self.v3((0, 0, 0)); self.v4((0, 0, 0, 0))
        self.u32(1)                    # one texture mapping
        self.s("txDiffuse")
        self.u32(0)                    # slot
        self.s(texture_name)

    def dummy_node(self, name, child_count, pos=(0, 0, 0), heading_deg=0.0):
        h = math.radians(heading_deg)
        right = ( math.cos(h), 0.0, -math.sin(h))
        up    = ( 0.0,         1.0,  0.0)
        fwd   = ( math.sin(h), 0.0,  math.cos(h))
        self.u32(1)               # class: Node
        self.s(name)
        self.u32(child_count)
        self.flag(True)           # active
        self.matrix(right, up, fwd, pos)

    def mesh_node(self, name, verts, indices, material_id, transparent=False):
        """verts: list of (pos3, normal3, uv2, tangent3). indices: flat tri list."""
        if len(verts) > 65535:
            raise ValueError(f"{name}: {len(verts)} verts exceeds 65535 limit")
        self.u32(2)               # class: Mesh
        self.s(name)
        self.u32(0)               # children (none allowed for meshes)
        self.flag(True)           # active
        self.flag(not transparent)  # castShadows (blended blobs cast none)
        self.flag(True)           # visible
        self.flag(transparent)    # transparent
        self.u32(len(verts))
        for pos, nrm, uv, tan in verts:
            self.v3(pos); self.v3(nrm); self.v2(uv); self.v3(tan)
        self.u32(len(indices))
        for i in indices:
            self.u16(i)
        self.u32(material_id)
        self.u32(0)               # layer
        self.f32(0.0)             # lodIn
        self.f32(0.0)             # lodOut (0 = always visible)
        # bounding sphere from bbox (matches reference exporter behaviour)
        px = [v[0][0] for v in verts]; py = [v[0][1] for v in verts]; pz = [v[0][2] for v in verts]
        cx = (min(px)+max(px))/2; cy = (min(py)+max(py))/2; cz = (min(pz)+max(pz))/2
        radius = max(max(px)-min(px), max(py)-min(py), max(pz)-min(pz))
        self.v3((cx, cy, cz))
        self.f32(radius)
        self.flag(True)           # renderable


def _strip_to_kn5_verts(pairs_flat, uvs, centerline):
    """
    Convert an alternating (A-side, B-side) vertex strip into KN5 vertex
    tuples with smooth up-facing normals and along-road tangents.
    Returns (verts, flip_winding).
    """
    n = len(pairs_flat) // 2
    verts = []
    for i in range(n):
        a = pairs_flat[2*i]
        b = pairs_flat[2*i + 1]
        i0, i1 = max(0, i-1), min(n-1, i+1)
        c0, c1 = centerline[i0], centerline[i1]
        fwd = (c1[0]-c0[0], c1[1]-c0[1], c1[2]-c0[2])
        fl = math.sqrt(fwd[0]**2 + fwd[1]**2 + fwd[2]**2) or 1.0
        fwd = (fwd[0]/fl, fwd[1]/fl, fwd[2]/fl)
        lat = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
        # normal = fwd × lat, forced upward
        nx = fwd[1]*lat[2] - fwd[2]*lat[1]
        ny = fwd[2]*lat[0] - fwd[0]*lat[2]
        nz = fwd[0]*lat[1] - fwd[1]*lat[0]
        nl = math.sqrt(nx*nx + ny*ny + nz*nz)
        if nl < 1e-9:
            nx, ny, nz = 0.0, 1.0, 0.0   # degenerate lateral → default up
        else:
            nx, ny, nz = nx/nl, ny/nl, nz/nl
            if ny < 0:
                nx, ny, nz = -nx, -ny, -nz
        for v, u in ((a, uvs[2*i]), (b, uvs[2*i+1])):
            verts.append(((v[0], v[1], v[2]), (nx, ny, nz), u, fwd))

    # Winding check on first quad: geometric normal of tri (0,3,1) must be up
    if n >= 2:
        p0 = pairs_flat[0]; p3 = pairs_flat[3]; p1 = pairs_flat[1]
        e1 = (p3[0]-p0[0], p3[1]-p0[1], p3[2]-p0[2])
        e2 = (p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2])
        gy = e1[2]*e2[0] - e1[0]*e2[2]     # Y of e1×e2
        flip = gy < 0
    else:
        flip = False
    return verts, flip


def _strip_indices(n_pairs, flip):
    idx = []
    for i in range(n_pairs - 1):
        a, b, c, d = 2*i, 2*i+1, 2*i+2, 2*i+3
        if flip:
            idx.extend((a, b, d)); idx.extend((a, d, c))
        else:
            idx.extend((a, d, b)); idx.extend((a, c, d))
    return idx


def _chunk_strip(verts, uvs, centerline, base_name, max_pairs=30000):
    """Split long strips into ≤65k-vertex meshes: yields (name, verts, uvs, cl)."""
    n = len(verts) // 2
    if n <= max_pairs:
        yield base_name, verts, uvs, centerline
        return
    start = 0
    part = 0
    while start < n - 1:
        end = min(start + max_pairs, n)
        name = base_name if part == 0 else f"{base_name}_{part}"
        yield (name,
               verts[2*start:2*end],
               uvs[2*start:2*end],
               centerline[start:end])
        start = end - 1   # overlap one pair to avoid gaps
        part += 1


def build_kn5(mesh: dict, track_name: str, env_meshes: list = None,
              terrain_meshes: list = None) -> bytes:
    """Build a complete, ready-to-drive KN5 — no Blender, no ksEditor.
    env_meshes: [(name, kn5_verts, indices, material_key)] visuals.
    terrain_meshes: real-elevation lateral terrain (physical); when given,
    the synthetic flat skirt is skipped."""
    k = _KN5()
    k.buf.write(b"sc6969")
    k.u32(5)                                   # file version

    # ── Textures ──
    textures = [("road.png",     build_road_texture(
                                     style=mesh.get("road_style", "sealed"), markings=not mesh.get("unified_network", False))),
                ("grass.png",    build_grass_texture()),
                ("building.png", build_building_texture()),
                ("tree.png",     build_tree_texture()),
                ("terrain.png",  build_terrain_texture()),
                ("forest.png",   build_forest_texture()),
                ("dirt.png",     build_dirt_texture()),
                ("water.png",    build_water_texture()),
                ("roof.png",      build_roof_texture()),
                ("asphalt.png",   build_asphalt_texture()),
                ("building2.png", build_building2_texture()),
                ("tree2.png",     build_tree2_texture()),
                ("building3.png",  build_building3_texture()),
                ("commercial.png", build_commercial_texture()),
                ("treeshadow.png", build_treeshadow_texture()),
                ("sideroad.png",   build_sideroad_texture()),
                ("marking.png", _png_encode(4, 4, bytes([235,235,220]*16)))]
    textures.extend((key + ".png", data) for key, data in mesh.get("sign_textures", {}).items())
    k.i32(len(textures))
    for tex_name, data in textures:
        k.i32(1)                               # active
        k.s(tex_name)
        k.blob(data)

    # ── Materials ──
    mat_ids = {"road": 0, "grass": 1, "building": 2, "tree": 3, "terrain": 4,
               "forest": 5, "dirt": 6, "water": 7, "roof": 8, "asphalt": 9,
               "building2": 10, "tree2": 11, "building3": 12, "commercial": 13,
               "treeshadow": 14, "sideroad": 15, "marking": 16}
    for key in mesh.get("sign_textures", {}):
        mat_ids[key] = len(mat_ids)
    k.i32(len(mat_ids))
    _road_style = mesh.get("road_style", "sealed")
    if _road_style == "gravel":               # dirt, not polished bitumen
        k.material("road", "ksPerPixel", "road.png", specular=0.02, spec_exp=8.0)
    else:
        k.material("road", "ksPerPixel", "road.png", specular=0.08, spec_exp=30.0)
    k.material("grass",    "ksPerPixel", "grass.png",    specular=0.01, spec_exp=5.0)
    k.material("building", "ksPerPixel", "building.png", specular=0.03, spec_exp=10.0)
    k.material("tree",     "ksPerPixel", "tree.png",     specular=0.01, spec_exp=5.0,
               alpha_tested=True)
    k.material("terrain",  "ksPerPixel", "terrain.png",  specular=0.01, spec_exp=5.0)
    k.material("forest",   "ksPerPixel", "forest.png",   specular=0.01, spec_exp=5.0)
    k.material("dirt",     "ksPerPixel", "dirt.png",     specular=0.01, spec_exp=5.0)
    k.material("water",    "ksPerPixel", "water.png",    specular=0.30, spec_exp=60.0)
    k.material("roof",     "ksPerPixel", "roof.png",     specular=0.02, spec_exp=8.0)
    k.material("asphalt",  "ksPerPixel", "asphalt.png",  specular=0.06, spec_exp=25.0)
    k.material("building2","ksPerPixel", "building2.png",specular=0.03, spec_exp=10.0)
    k.material("tree2",    "ksPerPixel", "tree2.png",    specular=0.01, spec_exp=5.0,
               alpha_tested=True)
    k.material("building3", "ksPerPixel", "building3.png",  specular=0.03, spec_exp=10.0)
    k.material("commercial","ksPerPixel", "commercial.png", specular=0.05, spec_exp=14.0)
    k.material("treeshadow","ksPerPixel", "treeshadow.png", specular=0.0,  spec_exp=1.0,
               alpha_blend=1)
    k.material("sideroad",  "ksPerPixel", "sideroad.png",   specular=0.05, spec_exp=18.0)
    k.material("marking", "ksPerPixel", "marking.png", specular=0.01, spec_exp=5.0)

    for key in mesh.get("sign_textures", {}):
        k.material(key, "ksPerPixel", key + ".png", specular=0.01, spec_exp=5.0)

    # ── Geometry ──
    cl = mesh["centerline"]
    meshes = []   # (name, verts, indices, material_id)

    def add_strip(base_name, vs_key_or_list, uvs_list, mat_id):
        vs_all = mesh[vs_key_or_list] if isinstance(vs_key_or_list, str) else vs_key_or_list
        out = []
        if len(vs_all) < 4:            # fewer than 2 pairs = no quads
            return out
        for name, vs, us, sub_cl in _chunk_strip(vs_all, uvs_list, cl, base_name):
            if len(vs) < 4:
                continue
            kv, flip = _strip_to_kn5_verts(vs, us, sub_cl)
            out.append((name, kv, _strip_indices(len(kv)//2, flip), mat_id))
        return out

    road_meshes = add_strip("1ROAD", "vertices", mesh["uvs"], 0)
    if not mesh.get("unified_network"):
        meshes.extend(road_meshes)
    grass_meshes = []
    for base_name, key in (("1GRASS_L", "grass_l_verts"), ("1GRASS_R", "grass_r_verts")):
        grass_meshes.extend(add_strip(base_name, key, mesh["grass_uvs"], 1))
    meshes.extend(grass_meshes)
    # Terrain: real-elevation grid when available, flat skirt as fallback —
    # physical (GRASS surface) either way, so leaving the road doesn't drop
    # the car into the void
    if terrain_meshes:
        for name, kv, idx, mat_key in terrain_meshes:
            meshes.append((name, kv, idx, mat_ids[mat_key]))
    elif mesh.get("skirt_l_verts"):
        for base_name, key in (("1GRASS_SL", "skirt_l_verts"),
                               ("1GRASS_SR", "skirt_r_verts")):
            meshes.extend(add_strip(base_name, key, mesh["skirt_uvs"],
                                    mat_ids["terrain"]))

    # Do not duplicate the entire physical terrain with reversed winding.
    # The old UNDERLAY meshes doubled memory, KN5 size and export time.

    # ── Environment (buildings + trees) — visual only, no physics ──
    for name, kv, idx, mat_key in (env_meshes or []):
        meshes.append((name, kv, idx, mat_ids[mat_key]))

    # ── Spawn nodes — position encoded in the NODE TRANSFORM, which is
    #    exactly what AC reads. This is the definitive fix for
    #    "NO POSITION DATA FOUND".
    stats = mesh["stats"]
    hw = stats["road_width"] / 2.0

    def hdg(p1, p2):
        return math.degrees(math.atan2(p2[0]-p1[0], p2[2]-p1[2]))

    s_hdg = hdg(cl[0], cl[min(1, len(cl)-1)])
    e_hdg = hdg(cl[-2], cl[-1])
    hr, ehr = math.radians(s_hdg), math.radians(e_hdg)

    # Ordering matters: the car must spawn BEHIND the start gate so the
    # timer starts when it crosses the line (not instantly on load).
    # spawn/pit ~5m in, start gate ~35m in, finish gate ~5m before the end.
    n_cl = len(cl)
    spawn_i = min(5,  n_cl - 1)
    pit_i   = min(8,  n_cl - 1)
    gate_i  = min(35, max(spawn_i + 5, int(n_cl * 0.05)))
    gate_i  = min(gate_i, n_cl - 2)
    fin_i   = max(n_cl - 6, gate_i + 1)

    g_hdg  = hdg(cl[gate_i], cl[min(gate_i + 1, n_cl - 1)])
    ghr    = math.radians(g_hdg)
    f_hdg  = hdg(cl[max(fin_i - 1, 0)], cl[fin_i])
    fhr    = math.radians(f_hdg)

    spawn = (float(cl[spawn_i][0]), float(cl[spawn_i][1]) + 1.5, float(cl[spawn_i][2]))
    pit   = (float(cl[pit_i][0]),   float(cl[pit_i][1])   + 1.5, float(cl[pit_i][2]))

    def edge(p, h, side, dist):
        return (p[0] + side*math.sin(h + math.pi/2)*dist,
                p[1] + 1.5,
                p[2] + side*math.cos(h + math.pi/2)*dist)

    dummies = [
        ("AC_AB_START_L",     edge(cl[gate_i], ghr, +1, hw), g_hdg),
        ("AC_AB_START_R",     edge(cl[gate_i], ghr, -1, hw), g_hdg),
        ("AC_AB_FINISH_L",    edge(cl[fin_i],  fhr, +1, hw), f_hdg),
        ("AC_AB_FINISH_R",    edge(cl[fin_i],  fhr, -1, hw), f_hdg),
        ("AC_START_0",        spawn, s_hdg),
        ("AC_HOTLAP_START_0", spawn, s_hdg),
        ("AC_PIT_0",          pit,   s_hdg),
    ]

    # ── Node tree ── root(identity) → meshes + spawn dummies
    k.dummy_node(track_name, len(meshes) + len(dummies))
    for name, kv, idx, mat_id in meshes:
        k.mesh_node(name, kv, idx, mat_id,
                    transparent=name.startswith("ENV_SHADOW"))
    for name, pos, heading in dummies:
        k.dummy_node(name, 0, pos, heading)

    return k.buf.getvalue()



# ─── Export ───────────────────────────────────────────────────────────────────


def build_blender_script(mesh: dict, track_name: str) -> str:
    """
    Generate a Blender Python script that creates AC spawn point empties
    with the correct configuration for Assetto Corsa.

    Run this AFTER importing the OBJ. It creates empties (not mesh cubes)
    at suggested positions along the road. The user can then move them
    to fine-tune placement before exporting FBX.

    AC spawn point rules (from community research):
      - Empty or cube, named AC_START_0, AC_PIT_0, etc.
      - Axis: local Y up, local Z forward (direction of travel)
      - In Blender this means: rotation_euler X=90°, Y=0°, Z=heading
      - Scale: 0.01 (DO NOT apply scale)
      - DO NOT apply rotation (Ctrl+A) — AC reads the unapplied transform
      - Height: 1-2m above road surface
    """
    cl = mesh["centerline"]
    stats = mesh["stats"]
    hw = stats["road_width"] / 2.0

    def hdg(p1, p2):
        return math.degrees(math.atan2(p2[0]-p1[0], p2[2]-p1[2]))

    s_hdg = hdg(cl[0], cl[min(1, len(cl)-1)])
    hr = math.radians(s_hdg)
    e_hdg = hdg(cl[-2], cl[-1])
    ehr = math.radians(e_hdg)

    # Spawn positions — 10 points along road, 1.5m above surface
    si = min(10, len(cl)-1)
    sx, sy, sz = float(cl[si][0]), float(cl[si][1]) + 1.5, float(cl[si][2])

    # Gate posts — at road edges
    sl_x = float(cl[0][0]) - math.sin(hr + math.pi/2) * hw
    sl_z = float(cl[0][2]) - math.cos(hr + math.pi/2) * hw
    sr_x = float(cl[0][0]) + math.sin(hr + math.pi/2) * hw
    sr_z = float(cl[0][2]) + math.cos(hr + math.pi/2) * hw
    start_y = float(cl[0][1]) + 1.5

    fl_x = float(cl[-1][0]) - math.sin(ehr + math.pi/2) * hw
    fl_z = float(cl[-1][2]) - math.cos(ehr + math.pi/2) * hw
    fr_x = float(cl[-1][0]) + math.sin(ehr + math.pi/2) * hw
    fr_z = float(cl[-1][2]) + math.cos(ehr + math.pi/2) * hw
    end_y = float(cl[-1][1]) + 1.5

    # Pit — left side of road near start
    px = float(cl[0][0]) - math.sin(hr + math.pi/2) * (hw * 0.5)
    pz = float(cl[0][2]) - math.cos(hr + math.pi/2) * (hw * 0.5)
    py = float(cl[0][1]) + 1.5

    markers = [
        ("AC_AB_START_L",    sl_x, start_y, sl_z, s_hdg),
        ("AC_AB_START_R",    sr_x, start_y, sr_z, s_hdg),
        ("AC_AB_FINISH_L",   fl_x, end_y,   fl_z, e_hdg),
        ("AC_AB_FINISH_R",   fr_x, end_y,   fr_z, e_hdg),
        ("AC_START_0",       sx,   sy,       sz,   s_hdg),
        ("AC_HOTLAP_START_0",sx,   sy,       sz,   s_hdg),
        ("AC_PIT_0",         px,   py,       pz,   s_hdg),
    ]

    lines = [
        "import bpy, math",
        "",
        f"# ATLAS Road World Studio — spawn point setup for: {track_name}",
        "#",
        "# This script creates AC spawn point empties with correct settings.",
        "# Run AFTER importing the OBJ file.",
        "#",
        "# The empties are placed at suggested positions — you can move them",
        "# to wherever you want on the track. Just keep these rules:",
        "#   - DO NOT apply rotation (no Ctrl+A > Rotation)",
        "#   - DO NOT apply scale (no Ctrl+A > Scale)",
        "#   - Keep them 1-2m above the road surface",
        "#   - The arrow points in the direction the car will face",
        "",
        "spawn_points = [",
    ]

    for name, x, y, z, h in markers:
        lines.append(f'    ("{name}", {x:.4f}, {y:.4f}, {z:.4f}, {h:.2f}),')

    lines.extend([
        "]",
        "",
        "created = []",
        "for name, x, y, z, heading in spawn_points:",
        "    empty = bpy.data.objects.new(name, None)",
        "    bpy.context.collection.objects.link(empty)",
        "    empty.empty_display_type = 'SINGLE_ARROW'",
        "    empty.empty_display_size = 2.0",
        "    empty.location = (x, y, z)",
        "    # X=90° makes local Y point up, Z becomes forward",
        "    # Z=heading rotates to face along the road",
        "    empty.rotation_euler = (math.radians(90), 0, math.radians(heading))",
        "    empty.scale = (0.01, 0.01, 0.01)",
        "    created.append(name)",
        '    print(f"  Created {name} at ({x:.1f}, {y:.1f}, {z:.1f}) heading={heading:.1f}")',
        "",
        "# Show axis on empties so you can verify orientation",
        "for obj in bpy.data.objects:",
        "    if obj.name in created:",
        "        obj.show_axis = True",
        "",
        f'print("\\nCreated {len(markers)} spawn points for {track_name}")',
        'print("You can move them — just DO NOT apply rotation or scale.")',
        'print("The arrow shows the direction the car will face.")',
    ])

    return "\n".join(lines)



def build_track_files(mesh: dict, track_name: str, env_meshes: list = None,
                      terrain_meshes: list = None) -> dict:
    """Build every file of a complete, ready-to-drive AC track.
    Returns {relative_path: bytes}."""
    stats = mesh["stats"]
    files = {}

    # ── The KN5 itself — direct export, ready to drive ──
    files[f"{track_name}.kn5"] = build_kn5(mesh, track_name, env_meshes,
                                           terrain_meshes)

    files["models.ini"] = (
        f"[MODEL_0]\nFILE={track_name}.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n"
    ).encode()

    # ── data/ ──
    files["data/surfaces.ini"] = build_surfaces_ini().encode()

    map_png, map_ini, outline_png, preview_png = build_track_map(mesh)
    files["data/map.ini"] = map_ini.encode()
    files["map.png"]      = map_png

    # ── ui/ ──
    files["ui/ui_track.json"] = build_track_json(track_name, stats).encode()
    files["ui/preview.png"]   = preview_png
    files["ui/outline.png"]   = outline_png

    # ai/ folder must exist; AC records fast_lane here on first hotlap
    files["ai/.placeholder"] = b""

    # ── extension/ — CSP config: GrassFX + RainFX ──
    # Most AC players run Custom Shaders Patch via Content Manager; this
    # config makes it spawn dense 3D grass on our grass/terrain/forest
    # materials (occluded by the road surfaces) and marks materials for
    # rain behaviour. Vanilla AC ignores the folder entirely.
    files["extension/ext_config.ini"] = (
        "[ABOUT]\n"
        f"AUTHOR = road-to-track generator\n"
        f"VERSION = 1\n"
        "\n"
        "[GRASS_FX]\n"
        "GRASS_MATERIALS = grass, terrain, forest\n"
        "OCCLUDING_MATERIALS = road, asphalt, dirt, water\n"
        "ORIGINAL_GRASS_MATERIALS =\n"
        "SHAPE_SIZE = 1.3\n"
        "SHAPE_TIDY = 0.2\n"
        "SHAPE_WIDTH = 1.0\n"
        "\n"
        "[RAIN_FX]\n"
        "PUDDLES_MATERIALS = road, asphalt\n"
        "SOAKING_MATERIALS = grass, terrain, forest, dirt\n"
        "SMOOTH_MATERIALS = road, asphalt\n"
    ).encode()

    # ── extras/ — optional Blender workflow for customisation ──
    obj, mtl = build_obj(mesh, track_name)
    if len(obj) <= 5 * 1024 * 1024:   # skip giant OBJs on long tracks
        files[f"extras/{track_name}.obj"] = obj.encode()
        files[f"extras/{track_name}.mtl"] = mtl.encode()
    files["extras/setup_in_blender.py"] = build_blender_script(mesh, track_name).encode()
    files["extras/README_EXTRAS.txt"] = (
        "These files are OPTIONAL - the track already works out of the box.\n"
        "Use them only if you want to customise the mesh in Blender and\n"
        "rebuild the KN5 yourself via FBX + ksEditor.\n"
    ).encode()

    if mesh.get("generation_report"):
        files["generation_report.json"] = json.dumps(mesh["generation_report"], indent=2).encode()

    files["OSM_ATTRIBUTION.txt"] = b"Contains information from OpenStreetMap contributors, available under the Open Database License (ODbL). https://www.openstreetmap.org/copyright\n"

    files["README.txt"] = f"""ATLAS Road World Studio Export
===================
Track: {track_name}  |  Length: {stats['length_km']} km  |  Width: {stats['road_width']} m

Generated KN5 track package - no Blender or ksEditor conversion needed.
Load and drive it in Assetto Corsa to verify collision and spawn behavior.
The .kn5 was generated directly with road, grass, textures and
all spawn points built in.

INSTALL
  1. Delete any previous version of this track folder.
  2. Extract the zip into your Assetto Corsa root folder:
     C:\\...\\steamapps\\common\\assettocorsa\\
     (so this folder ends up in assettocorsa/content/tracks/)
  3. Launch in HOTLAP or PRACTICE mode.
     (Race needs an AI line: drive the road once in Hotlap mode,
      AC saves a fast_lane candidate into the ai/ folder.)

If the car spawns facing the wrong way, tell the tool author -
the heading convention flips in one place.

extras/ contains an optional Blender workflow for customisation.
""".encode()

    return files


def find_assetto_corsa():
    """Find an existing Steam installation; never create a fake AC directory."""
    import re
    roots = []
    if os.name == "nt":
        try:
            import winreg
            for hive in (winreg.HKEY_CURRENT_USER, winreg.HKEY_LOCAL_MACHINE):
                for key in (r"Software\Valve\Steam", r"SOFTWARE\WOW6432Node\Valve\Steam"):
                    try:
                        with winreg.OpenKey(hive, key) as h:
                            for value in ("SteamPath", "InstallPath"):
                                try:
                                    roots.append(winreg.QueryValueEx(h, value)[0])
                                except OSError:
                                    pass
                    except OSError:
                        pass
        except ImportError:
            pass
        for drive in "CDEFGHIJKLMNOPQRSTUVWXYZ":
            roots.extend((f"{drive}:\\Program Files (x86)\\Steam",
                          f"{drive}:\\Program Files\\Steam", f"{drive}:\\SteamLibrary"))
    seen = set()
    for root in roots:
        if not root or root in seen:
            continue
        seen.add(root)
        libraries = [root]
        vdf = os.path.join(root, "steamapps", "libraryfolders.vdf")
        if os.path.isfile(vdf):
            try:
                with open(vdf, encoding="utf-8-sig") as f:
                    raw = f.read()
                libraries += [x.replace("\\\\", "\\") for x in
                              re.findall(r'"path"\s+"([^"]+)"', raw)]
            except OSError:
                pass
        for library in libraries:
            ac = os.path.join(library, "steamapps", "common", "assettocorsa")
            if os.path.isdir(os.path.join(ac, "content", "tracks")):
                return ac
    return None


def resolve_ac_tracks(install_path):
    """Return a real AC tracks directory or raise a helpful error."""
    root = os.path.abspath(os.path.expanduser(install_path.strip()))
    if os.path.basename(root).lower() == "tracks":
        tracks = root
    elif os.path.basename(root).lower() == "content":
        tracks = os.path.join(root, "tracks")
    else:
        tracks = os.path.join(root, "content", "tracks")
    if not os.path.isdir(tracks):
        raise FileNotFoundError("Assetto Corsa tracks folder not found: " + tracks)
    return tracks


def validate_track_files(files, track_name):
    required = (f"{track_name}.kn5", "models.ini", "data/surfaces.ini",
                "ui/ui_track.json", "ui/preview.png")
    for name in required:
        if not files.get(name):
            raise ValueError("Missing required Assetto Corsa file: " + name)
    kn5 = files[f"{track_name}.kn5"]
    if not kn5.startswith(b"sc6969") or len(kn5) < 1024:
        raise ValueError("Generated KN5 is invalid or empty")
    if f"FILE={track_name}.kn5" not in files["models.ini"].decode("utf-8"):
        raise ValueError("models.ini does not reference the generated KN5")


def export_ac_package(coords: list, road_width: float,
                      smooth_factor: float, track_name: str,
                      install_path: str = None,
                      include_env: bool = True,
                      max_grade: float = None,
                      monotonic: bool = False,
                      road_type: str = None,
                      road_surface: str = None) -> dict:
    # Elevation and land cover are required, not optional. If they can't be
    # obtained the export fails with the reason — a flat or untextured track
    # looks plausible but is wrong, which is worse than a clear failure.
    EXPORT_PROGRESS.update(stage="Fetching elevation", done=0, total=0, failed=0)
    print(f"  [export] Fetching elevation profile…")
    try:
        elev_profile = fetch_elevation_profile(coords, max_grade=max_grade,
                                               monotonic=monotonic)
    except ElevationUnavailable as e:
        return {"error": f"Elevation unavailable — {e}"}
    if not elev_profile:
        return {"error": "Elevation unavailable — no profile could be built "
                         "for this road."}
    print(f"  [export] Elevation OK")

    # Terrain runs to the road edge, so no wide grass verge is needed.
    mesh = process_road(coords, road_width, smooth_factor,
                        elev_profile=elev_profile, grass_width=0.0)
    if "error" in mesh:
        return mesh
    mesh["road_style"] = _classify_road_style(road_type, road_surface)
    print(f"  [export] Road style: {mesh['road_style']} "
          f"(highway={road_type}, surface={road_surface})")

    EXPORT_PROGRESS.update(stage="Fetching land cover", done=0, total=0)
    # ── Land cover (ESA WorldCover): surface type + vegetation everywhere,
    #    including where OpenStreetMap has no polygons drawn. ──
    try:
        lc = fetch_land_cover(coords)
    except Exception as e:
        return {"error": f"Land cover unavailable — {e}"}
    if lc is None:
        return {"error": "Land cover unavailable — ESA WorldCover could not "
                         "be read for this area."}
    pp = mesh["proj"]
    _inv = Transformer.from_crs(
        f"+proj=tmerc +lat_0={pp['mid_lat']} +lon_0={pp['mid_lon']} +units=m",
        "EPSG:4326", always_xy=True)
    _ox, _oz = pp["ox"], pp["oz"]

    def _to_latlon(x, z):
        lon, lat = _inv.transform(x + _ox, -(z + _oz))
        return lat, lon
    mesh["land_cover"] = lc
    mesh["to_latlon"] = _to_latlon

    # Surroundings are fetched BEFORE terrain so the terrain grid can be
    # graded along side roads and waterways (shelves cut into hillsides).
    surroundings = None
    if include_env:
        try:
            print(f"  [export] Fetching surroundings from OSM…")
            surroundings = fetch_surroundings(coords)
        except IOError as e:
            return {"error": f"OSM scenery could not be downloaded; buildings and roads would be missing: {e}"}
        except Exception as e:
            return {"error": f"OSM scenery generation failed; refusing incomplete export: {e}"}

    mesh["unified_network"] = True
    EXPORT_PROGRESS.update(stage="Generating terrain and meshes", done=0, total=0)
    # ── Real terrain beside the road (cliffs, valleys, hillsides) ──
    try:
        tgrid = fetch_terrain_grid(mesh)
        terrain_meshes = build_terrain_meshes(mesh, tgrid,
                                              surroundings=surroundings)
    except ElevationUnavailable as e:
        return {"error": f"Terrain elevation unavailable — {e}"}
    if not terrain_meshes:
        return {"error": "Terrain could not be built for this road."}
    ground_pts = [v[0] for _, kv, _, _ in terrain_meshes for v in kv]
    ground_pts += list(mesh["centerline"])
    print(f"  [export] Real terrain: "
          f"{sum(len(kv) for _, kv, _, _ in terrain_meshes)} verts")

    # Coarse visual-only backdrop ring, so distant road sections sit on
    # hills instead of void. Added after ground_pts so the environment's
    # ground lookup keeps only the fine grid.
    try:
        far_meshes = build_far_terrain(mesh, tgrid)
        if far_meshes:
            terrain_meshes = terrain_meshes + far_meshes
            print(f"  [export] Far terrain: "
                  f"{sum(len(kv) for _, kv, _, _ in far_meshes)} verts")
    except Exception as e:
        print(f"  [export] Far terrain skipped: {e}")

    env_meshes = []
    n_bldg = n_tree = 0
    generation_warnings = []
    network_stats = {}
    sign_stats = {"mapped": 0, "inferred": 0, "unsupported": 0}
    from road_network_v3 import build_network_surfaces, carve_terrain
    network, network_stats = build_network_surfaces(surroundings or {}, mesh, ground_pts, corridor_m=50.0)
    if not network:
        return {"error": "No drivable road surface could be generated"}
    mesh["unified_network"] = True
    env_meshes.extend(network)
    generation_warnings.extend(network_stats.get("warnings", []))
    terrain_meshes = carve_terrain(terrain_meshes, mesh)
    if surroundings is not None:
        from buildings import build_buildings
        from road_signs import build_signs
        building_meshes, n_bldg, building_warnings = build_buildings(surroundings, mesh, ground_pts)
        scenery = build_environment_meshes(surroundings, mesh, ground_pts)
        env_meshes.extend(scenery["meshes"])
        env_meshes.extend(building_meshes)
        n_tree = scenery["n_trees"]
        signs, sign_stats = build_signs(surroundings, mesh)
        env_meshes.extend(signs)
        generation_warnings.extend(building_warnings)
        if not surroundings.get("buildings"):
            generation_warnings.append("OSM returned no building footprints for this area; no buildings were invented.")
        if sign_stats["unsupported"]:
            generation_warnings.append(f'{sign_stats["unsupported"]} mapped controls have unsupported sign codes.')
    print(f"  [network] {network_stats}; buildings={n_bldg}; signs={sign_stats}")
    stats = mesh["stats"]
    EXPORT_PROGRESS.update(stage="Packaging and validating Assetto Corsa track", done=0, total=0)
    mesh["generation_report"] = {
        "corridor_metres": 50.0, "network": network_stats,
        "buildings_exported": n_bldg, "signs": sign_stats,
        "warnings": generation_warnings,
        "physical_direction_restrictions": False,
        "elevation_mode": elev_profile.get("dem_mode", "auto"),
        "accuracy_notes": [
            "Road shapes and building footprints come from OSM, not a survey.",
            "Missing road widths and building heights use documented defaults.",
            "Procedural building facades and flat roofs are approximations.",
            "Sign placement inferred from road tags is not a verified roadside inventory.",
            "Global elevation tiles cannot resolve measured camber or kerbs.",
            "Windows build and in-game collision must be tested on Windows/Assetto Corsa."
        ]
    }
    from mesh_validation import validate_meshes
    validate_meshes(env_meshes + terrain_meshes)
    files = build_track_files(mesh, track_name, env_meshes, terrain_meshes)
    validate_track_files(files, track_name)

    base = f"content/tracks/{track_name}/"
    zip_buf = io.BytesIO()
    with zipfile.ZipFile(zip_buf, 'w', zipfile.ZIP_DEFLATED, compresslevel=1) as zf:
        for rel, data in files.items():
            zf.writestr(base + rel, data)

    zip_bytes = zip_buf.getvalue()
    with open(os.path.join(OUTPUT_DIR, f"{track_name}.zip"), 'wb') as f:
        f.write(zip_bytes)

    # Install to the detected Steam library when the user leaves the path blank.
    installed_to = None
    install_error = None
    chosen_path = install_path or find_assetto_corsa()
    if chosen_path:
        try:
            tracks_dir = resolve_ac_tracks(chosen_path)
            track_dir = os.path.join(tracks_dir, track_name)
            # Stage first: failed writes must not delete a working track.
            import tempfile
            stage = tempfile.mkdtemp(prefix="atlas_install_", dir=tracks_dir)
            try:
                for rel, data in files.items():
                    dest = os.path.join(stage, rel.replace("/", os.sep))
                    os.makedirs(os.path.dirname(dest), exist_ok=True)
                    with open(dest, "wb") as f:
                        f.write(data)
                if os.path.isfile(os.path.join(track_dir, "data.acd")):
                    raise RuntimeError("Refusing to overwrite an existing packed track")
                backup = None
                if os.path.exists(track_dir):
                    backup = track_dir + ".atlas_backup"
                    if os.path.exists(backup):
                        shutil.rmtree(backup)
                    os.replace(track_dir, backup)
                try:
                    os.replace(stage, track_dir)
                except Exception:
                    if backup:
                        os.replace(backup, track_dir)
                    raise
                if backup:
                    shutil.rmtree(backup)
                installed_to = track_dir
                print(f"  [export] Installed directly to {track_dir}")
            finally:
                if os.path.isdir(stage):
                    shutil.rmtree(stage)
        except Exception as e:
            install_error = str(e)
            print(f"  [export] Direct install failed: {e}")
    else:
        install_error = ("Assetto Corsa was not detected. Set its install folder "
                         "above or download the ZIP and extract it into your AC folder.")
    EXPORT_PROGRESS.update(stage="Export complete", done=1, total=1)

    profile = mesh["elevation_profile"]
    raw = elev_profile.get("raw_elevs") or []
    unc = elev_profile.get("uncert") or []
    pmin = min(elev_profile["elevs"]) if elev_profile.get("elevs") else 0.0
    return {
        "stats":             stats,
        "network":           network_stats,
        "signs":             sign_stats,
        "warnings":          generation_warnings,
        "filename":          f"{track_name}.zip",
        "size_kb":           round(len(zip_bytes) / 1024, 1),
        "kn5_kb":            round(len(files[f"{track_name}.kn5"]) / 1024, 1),
        "buildings":         n_bldg,
        "trees":             n_tree,
        "installed_to":      installed_to,
        "install_error":     install_error,
        "elevation_profile": profile[::max(1, len(profile)//500)],
        "elevation_raw":     [round(v - pmin, 2)
                              for v in raw[::max(1, len(raw)//500)]],
        "elevation_unc":     [round(v, 2)
                              for v in unc[::max(1, len(unc)//500)]],
        "sidehill_frac":     round(elev_profile.get("sidehill_frac", 0), 3),
        "uncert_med":        round(elev_profile.get("uncert_med", 0), 1),
        "dem_mode":          elev_profile.get("dem_mode", "auto"),
        "dem_coverage":      round(elev_profile.get("dem_coverage", 0.0), 3),
    }


# ─── HTTP Handler ─────────────────────────────────────────────────────────────

def route_waypoints(waypoints, chunked=True):
    """Trace clickable stops on actual roads via the public OSRM routing service.

    Small overlapping batches prevent oversized routing requests. Each batch
    includes the previous endpoint so concatenated geometry is continuous.
    """
    if not isinstance(waypoints, list) or not 2 <= len(waypoints) <= 150:
        return {"error": "Choose 2 to 150 waypoints"}
    try:
        stops = [(float(p[0]), float(p[1])) for p in waypoints]
        if any(not (-90 <= lat <= 90 and -180 <= lon <= 180) for lat, lon in stops):
            return {"error": "Invalid waypoint coordinates"}
    except (TypeError, ValueError, IndexError):
        return {"error": "Invalid waypoint coordinates"}
    import urllib.request
    batch_size = 12 if chunked else len(stops)
    coords = []
    start = 0
    while start < len(stops) - 1:
        batch = stops[start:min(len(stops), start + batch_size)]
        pairs = ';'.join(f'{lon:.6f},{lat:.6f}' for lat, lon in batch)
        url = 'https://router.project-osrm.org/route/v1/driving/' + pairs + '?overview=full&geometries=geojson&steps=false'
        req = urllib.request.Request(url, headers={'User-Agent': 'AtlasRoadWorldStudio/1.0 (local development)'})
        try:
            with urllib.request.urlopen(req, timeout=45) as r:
                result = json.load(r)
            if result.get('code') != 'Ok' or not result.get('routes'):
                return {'error': 'No driving route between these waypoints. Move a point closer to a mapped road.'}
            section = [[lat, lon] for lon, lat in result['routes'][0]['geometry']['coordinates']]
            if coords and section and coords[-1] == section[0]:
                section = section[1:]
            coords.extend(section)
        except Exception as exc:
            return {'error': 'Road routing request failed: ' + str(exc)}
        start += len(batch) - 1
    return {'coords': coords, 'segments_used': len(stops)-1, 'segments_total': len(stops)-1}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def send_json(self, data, status=200):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", len(body))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def send_file(self, path):
        with open(path, 'rb') as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", "application/zip")
        self.send_header("Content-Length", len(data))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Disposition",
                         f'attachment; filename="{os.path.basename(path)}"')
        self.end_headers()
        self.wfile.write(data)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)

        if parsed.path == "/":
            with open(os.path.join(os.path.dirname(__file__),
                                   "static", "index.html"), 'rb') as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)

        elif parsed.path.startswith("/static/"):
            # Only serve packaged frontend assets inside the static directory.
            from pathlib import Path
            import mimetypes
            from urllib.parse import unquote
            static_root = Path(__file__).resolve().parent / "static"
            asset = (static_root / unquote(parsed.path[len("/static/"):])).resolve()
            if not asset.is_relative_to(static_root) or not asset.is_file():
                self.send_json({"error": "Not found"}, 404)
                return
            body = asset.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", mimetypes.guess_type(str(asset))[0] or "application/octet-stream")
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)

        elif parsed.path == "/api/search":
            q = qs.get("q", [""])[0]
            self.send_json(search_osm_roads(q) if q else {"error": "No query"})

        elif parsed.path == "/api/progress":
            self.send_json(dict(EXPORT_PROGRESS))

        elif parsed.path == "/api/dem_status":
            try:
                self.send_json(dem_status())
            except Exception as e:
                self.send_json({"mode": "auto", "dems": [],
                                "error": str(e)})
        elif parsed.path == "/api/geometry":
            t = qs.get("type", [""])[0]
            i = qs.get("id",   [""])[0]
            self.send_json(fetch_road_geometry(t, i) if t and i
                           else {"error": "Missing type/id"})

        elif parsed.path.startswith("/download/"):
            fname = parsed.path.replace("/download/", "")
            fpath = os.path.join(OUTPUT_DIR, fname)
            self.send_file(fpath) if os.path.exists(fpath) else self.send_json({"error": "Not found"}, 404)

        else:
            self.send_json({"error": "Not found"}, 404)

    def do_POST(self):
        parsed = urlparse(self.path)
        length = int(self.headers.get("Content-Length", 0))
        body   = json.loads(self.rfile.read(length)) if length else {}

        if parsed.path == "/api/route_waypoints":
            self.send_json(route_waypoints(body.get("waypoints", []), bool(body.get("chunked", True))))
            return

        if parsed.path == "/api/export":
            coords = body.get("coords", [])
            if not coords:
                self.send_json({"error": "No coords"}, 400)
                return
            rw    = float(body.get("road_width", 8.0))
            sm    = float(body.get("smooth", 0.3))
            name  = "".join(c for c in body.get("track_name", "my_road")
                            .replace(" ", "_").lower()
                            if c.isalnum() or c == "_")[:32]
            install_path = body.get("install_path") or None
            include_env  = bool(body.get("include_env", True))
            mg_raw = body.get("max_grade")
            mg = None if mg_raw in (None, "auto") else \
                max(0.05, min(0.35, float(mg_raw) / 100.0))
            try:
                result = export_ac_package(coords, rw, sm, name,
                                           install_path, include_env,
                                           max_grade=mg,
                                           monotonic=bool(body.get("monotonic", False)),
                                           road_type=body.get("road_type"),
                                           road_surface=body.get("road_surface"))
                self.send_json(result, 400 if "error" in result else 200)
            except Exception as e:
                import traceback
                traceback.print_exc()
                EXPORT_PROGRESS.update(stage="Export failed", failed=1)
                self.send_json({"error": f"Export failed: {e}"}, 500)

        elif parsed.path == "/api/preview":
            coords = body.get("coords", [])
            if not coords:
                self.send_json({"error": "No coords"}, 400)
                return
            try:
                mg_raw = body.get("max_grade")
                mg = None if mg_raw in (None, "auto") else \
                    max(0.05, min(0.35, float(mg_raw) / 100.0))
                prof = fetch_elevation_profile(
                    coords, max_grade=mg,
                    monotonic=bool(body.get("monotonic", False)))
            except ElevationUnavailable as e:
                self.send_json({"error": f"Elevation unavailable — {e}"}, 400)
                return
            mesh = process_road(coords,
                                float(body.get("road_width", 8.0)),
                                float(body.get("smooth", 0.3)),
                                elev_profile=prof, grass_width=0.0)
            if "error" in mesh:
                self.send_json(mesh, 400)
                return
            p    = mesh["elevation_profile"]
            step = max(1, len(p) // 500)
            out = {"stats": mesh["stats"], "elevation_profile": p[::step],
                   "dem_mode": prof.get("dem_mode", "auto"),
                   "dem_coverage": round(prof.get("dem_coverage", 0.0), 3)}
            raw = prof.get("raw_elevs")
            if raw:
                # Same min-shift as the processed profile so the two curves
                # overlay: shift by the min of the PROCESSED source data
                # (despiked+graded+flattened), which is what process_road
                # subtracts before smoothing.
                pmin = min(prof["elevs"])
                rstep = max(1, len(raw) // 500)
                out["elevation_raw"] = [round(v - pmin, 2) for v in raw[::rstep]]
                unc = prof.get("uncert") or []
                if unc:
                    ustep = max(1, len(unc) // 500)
                    out["elevation_unc"] = [round(v, 2) for v in unc[::ustep]]
                    out["sidehill_frac"] = round(prof.get("sidehill_frac", 0), 3)
                    out["uncert_med"] = round(prof.get("uncert_med", 0), 1)
                # Independent Copernicus check line. Datum differs slightly
                # between DEMs, so align by median offset against the raw AWS
                # samples at the same points — shape disagreements (artefacts)
                # then stand out instead of a constant vertical shift.
                try:
                    alt_res = fetch_elevations_copernicus(prof["coords"])
                    if alt_res:
                        astep, alt = alt_res
                        ref = raw[::astep][:len(alt)]
                        off = float(np.median(np.array(alt[:len(ref)]) -
                                              np.array(ref)))
                        out["elevation_alt"] = [round(v - off - pmin, 2)
                                                for v in alt]
                        print(f"  [elevation] Copernicus check: {len(alt)} "
                              f"points (datum offset {off:+.1f}m)")
                except Exception as e:
                    print(f"  [elevation] Copernicus check skipped: {e}")
            self.send_json(out)

        else:
            self.send_json({"error": "Not found"}, 404)


if __name__ == "__main__":
    print(f"\n  ATLAS Road World Studio")
    print("  -----------------------------")
    print(f"  Open in browser: http://localhost:{PORT}")
    print(f"  Press Ctrl+C to stop\n")
    server = ThreadingHTTPServer(("", PORT), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n  Server stopped.")
