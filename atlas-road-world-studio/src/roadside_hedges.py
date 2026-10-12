"""Continuous inferred roadside boundaries; mapped driveways remain open."""
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from shapely import segmentize
from road_network_v3 import extract_network, projector, width_for, _lines

EDGE_OFFSET = .35
LOW_HEIGHT = .85
RURAL_HEIGHT = 1.5
HEDGE_ROADS = {'secondary','tertiary','unclassified','residential','living_street','service','track'}
HOUSE_KINDS = {'house','detached','semidetached_house','terrace','bungalow','residential','apartments','yes'}

def hedge_layout(surroundings, mesh):
    """Return local hedge lines, driveway clearance and a height function.

    Follow the OUTSIDE boundary of the actual asphalt union. Intersect with
    side-edge strips to exclude export end caps, never carve a junction-radius
    gap. This also wraps the corners onto normal side roads.
    """
    xy=projector(mesh)
    houses=[]
    for building in surroundings.get('buildings',[]):
        kind=building.get('kind',building.get('tags',{}).get('building','yes'))
        if kind not in HOUSE_KINDS or len(building.get('outline',[]))<3:continue
        poly=Polygon([xy(p) for p in building['outline']])
        if poly.is_valid and not poly.is_empty:houses.append(poly)
    house_area=unary_union(houses)
    frontage=house_area.buffer(18.)
    def height(x,z):
        return LOW_HEIGHT if frontage.covers(Point(x,z)) else RURAL_HEIGHT

    def driveway(road, line):
        tags=road.get('tags',{})
        if tags.get('service')=='driveway':return True
        # Untagged private service stubs terminating at a house are entrances,
        # not side roads. Public service roads retain hedges on both sides.
        return (road.get('type')=='service' and tags.get('access')=='private'
                and line.length<=60. and any(house_area.distance(Point(p))<8.
                                            for p in (line.coords[0],line.coords[-1])))
    entrances=[]
    for road in surroundings.get('roads',[]):
        if len(road.get('pts',[]))<2:continue
        line=LineString([xy(p) for p in road['pts']])
        if driveway(road,line):
            tags=road.get('tags',{})
            width=width_for(road) if 'width' in tags or 'lanes' in tags else 3.
            entrances.append(line.buffer(width/2+EDGE_OFFSET+.01,cap_style='flat'))
    entrance_area=unary_union(entrances)
    surface=mesh.get('network_surface')
    route=LineString([(p[0],p[2]) for p in mesh['centerline']])
    if surface is None:surface=route.buffer(mesh['stats']['road_width']/2,cap_style='flat')
    sections,_,_=extract_network(surroundings,mesh)
    # Use the actual selected route edges even when OSM omits that road.
    eligible=[(route,mesh['stats']['road_width'])]
    eligible.extend((line,width_for(road)) for line,road,_ in sections
                    if road.get('type') in HEDGE_ROADS and not driveway(road,line))
    strips=[]
    for line,width in eligible:
        for side in (-1,1):
            edge=line.offset_curve(side*(width/2+EDGE_OFFSET))
            strips.append(edge.buffer(.6,cap_style='flat'))
    sides=unary_union(strips)
    edges=surface.buffer(EDGE_OFFSET,join_style='round').boundary.intersection(sides)
    edges=edges.difference(entrance_area)
    # Existing walls/fences replace inferred hedges, not other hedge records:
    # incomplete mapped hedge snippets must not punch holes in the fallback.
    replacements=[]
    for barrier in surroundings.get('barriers',[]):
        if barrier.get('type')!='hedge' and len(barrier.get('pts',[]))>=2:
            replacements.append(LineString([xy(p) for p in barrier['pts']]).buffer(.45))
    if replacements:edges=edges.difference(unary_union(replacements))
    lines=[segmentize(line,1.) for line in _lines(edges) if line.length>.05]
    return lines, entrance_area, height
