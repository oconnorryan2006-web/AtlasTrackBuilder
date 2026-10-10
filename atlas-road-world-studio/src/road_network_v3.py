"""Metric corridor extraction and a single, bidirectional collision surface.

OSM oneway tags describe traffic rules, never physical barriers. Roads on
separate OSM layers are triangulated separately instead of welding bridges.
"""
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.interpolate import LinearNDInterpolator
from pyproj import Transformer
from shapely import constrained_delaunay_triangles, make_valid, segmentize
from shapely.geometry import LineString, Polygon, Point
from shapely.ops import unary_union, linemerge, nearest_points

ROAD_WIDTHS = {'motorway':11,'trunk':10,'primary':9,'secondary':8,
               'tertiary':7,'residential':6,'unclassified':6,
               'living_street':5,'service':4,'track':3.5}

def width_for(road):
    tags=road.get('tags',{})
    try:
        w=float(str(tags.get('width','')).lower().replace('m','').strip())
        if 2 <= w <= 24:return w
    except (ValueError,TypeError):pass
    try:
        n=int(tags.get('lanes',''))
        if 1 <= n <= 6:return min(21.,max(3.5,n*3.25))
    except (ValueError,TypeError):pass
    return ROAD_WIDTHS.get(road.get('type'),5.5)

def _resample(points,closed=False,step=1.25):
    # Follow mapped chords: splines can move roundabouts through islands.
    p=np.asarray(points,dtype=float)
    if len(p)<2:return p
    p=p[np.r_[True,np.linalg.norm(np.diff(p,axis=0),axis=1)>1e-8]]
    if closed and len(p)>2 and not np.allclose(p[0],p[-1]):p=np.vstack((p,p[0]))
    if len(p)<2:return p
    line=LineString(p)
    return np.asarray(segmentize(line,step).coords)

def projector(mesh):
    pp=mesh['proj']
    proj=Transformer.from_crs('EPSG:4326',f"+proj=tmerc +lat_0={pp['mid_lat']} +lon_0={pp['mid_lon']} +units=m",always_xy=True)
    def xy(pt):
        x,y=proj.transform(pt[1],pt[0]);return x-pp['ox'],-y-pp['oz']
    return xy

def _lines(g):
    if g.is_empty:return []
    if g.geom_type in ('LineString','LinearRing'):return [LineString(g)]
    return [p for child in getattr(g,'geoms',[]) for p in _lines(child)]

def extract_network(surroundings,mesh,corridor_m=50.):
    if not math.isfinite(corridor_m) or corridor_m<=0:raise ValueError('Invalid road corridor')
    xy=projector(mesh)
    cl=mesh.get('centerline',[])
    route=LineString([(p[0],p[2]) for p in cl]) if len(cl)>=2 else None
    corridor=route.buffer(corridor_m) if route is not None else None
    ordinary=[]; rings={}; seen=set(); warnings=[]
    for road in surroundings.get('roads',[]):
        if road.get('type') not in ROAD_WIDTHS:continue
        tags=road.get('tags',{})
        if tags.get('area')=='yes' or tags.get('motor_vehicle',tags.get('vehicle',tags.get('access'))) in ('no','private'):continue
        pts=road.get('pts',[])
        if len(pts)<2:continue
        raw=np.asarray([xy(p) for p in pts],dtype=float)
        if not np.isfinite(raw).all():raise ValueError('Nonfinite OSM road coordinates')
        if len(np.unique(raw,axis=0))<2:continue
        line=LineString(raw)
        identity=(road.get('osm_id'),line.wkb)
        if identity in seen:continue
        seen.add(identity)
        # No guessed elevations for grade-separated roads: report rather than
        # turning a bridge/tunnel into a false level junction.
        if tags.get('bridge') not in (None,'no') or tags.get('tunnel') not in (None,'no') or str(tags.get('layer','0'))!='0':
            if corridor is None or line.intersects(corridor):
                raise ValueError('Grade-separated road needs an elevation profile: OSM way '+str(road.get('osm_id','unknown')))
            continue
        ring=road.get('junction',tags.get('junction'))=='roundabout'
        if ring:rings.setdefault('0',[]).append((line,road))
        else:
            clipped=line.intersection(corridor) if corridor is not None else line
            for section in _lines(clipped):
                if section.length>0.01:ordinary.append((section,road,False))
    # OSM commonly splits a roundabout into several ways. Merge arcs first,
    # then select the complete connected ring if ANY part meets the corridor.
    for group in rings.values():
        merged=linemerge([line for line,_ in group])
        for line in _lines(merged):
            if corridor is not None and not line.intersects(corridor):continue
            members=[r for arc,r in group if arc.intersection(line).length>0.01]
            if not line.is_ring:
                raise ValueError('Incomplete OSM roundabout: cannot export a broken loop')
            rd=dict(members[0]);rd['tags']=dict(rd.get('tags',{}))
            # Conservatively retain a consistent mapped width for the loop.
            widths=[width_for(r) for r in members]
            rd['tags']['width']=max(widths)
            if max(widths)-min(widths)>.1:
                warnings.append('Roundabout has inconsistent OSM lane/width tags; using a continuous '+str(round(max(widths),2))+' m inferred carriageway width.')
            ordinary.append((line,rd,True))
    return ordinary,corridor,warnings

def _polygons(g):
    if g.is_empty:return []
    if g.geom_type=='Polygon':return [g]
    return [p for child in getattr(g,'geoms',[]) for p in _polygons(child)]

def build_network_surfaces(surroundings,mesh,ground_pts,max_roads=None,corridor_m=50.):
    sections,corridor,warnings=extract_network(surroundings,mesh,corridor_m)
    terrain=np.asarray(ground_pts,dtype=float)
    if not len(terrain):raise ValueError('Road surfaces require terrain elevations')
    kd=cKDTree(terrain[:,[0,2]])
    try:ground=LinearNDInterpolator(terrain[:,[0,2]],terrain[:,1])
    except Exception:ground=None
    main=np.asarray(mesh.get('centerline',[]),dtype=float)
    main_kd=cKDTree(main[:,[0,2]]) if len(main) else None
    main_width=mesh.get('stats',{}).get('road_width',8.)
    main_route=LineString(main[:,[0,2]]) if len(main)>=2 else None
    main_arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(main[:,[0,2]],axis=0),axis=1))] if len(main)>=2 else None
    def height(x,z):
        y=float(ground(x,z)) if ground is not None else float('nan')
        if not math.isfinite(y):y=float(terrain[kd.query([x,z])[1],1])
        if main_kd is not None:
            if main_route is not None:
                point=Point(x,z);d=main_route.distance(point)
                target=float(np.interp(main_route.project(point),main_arc,main[:,1]))
            else:
                d,i=main_kd.query([x,z]);target=float(main[i,1])
            blend=np.clip((d-main_width/2)/8.,0.,1.)
            y=target*(1-blend)+y*blend
        return y
    footprints=[]
    for line,rd,ring in sections:
        footprint=line.buffer(width_for(rd)/2,cap_style='flat',join_style='round')
        if not ring and corridor is not None:footprint=footprint.intersection(corridor)
        footprints.append(footprint)
    # Use exactly the selected road's edges; it becomes part of this union,
    # rather than a second overlapping physics ribbon at every junction.
    vs=mesh.get('vertices',[])
    if len(vs)>=4:
        left=[(v[0],v[2]) for v in vs[::2]];right=[(v[0],v[2]) for v in vs[1::2]]
        footprints.append(make_valid(Polygon(left+right[::-1])))
    surface=make_valid(unary_union(footprints)) if footprints else Polygon()
    mesh['network_surface']=surface
    mesh['network_height']=height
    output=[];verts=[];idx=[];lookup={}
    def flush():
        nonlocal verts,idx,lookup
        if idx:output.append((f'1ROAD_NETWORK_{len(output)}',verts,idx,'road'))
        verts=[];idx=[];lookup={}
    for poly in _polygons(surface):
        triangles=constrained_delaunay_triangles(segmentize(poly,3.))
        for triangle in triangles.geoms:
            points=list(triangle.exterior.coords)[:3]
            a,b,c=np.array([(x,height(x,z),z) for x,z in points])
            normal=np.cross(b-a,c-a)
            if np.linalg.norm(normal)<1e-9:continue
            if normal[1]<0:points[1],points[2]=points[2],points[1]
            if len(verts)+3>60000:flush()
            ids=[]
            for x,z in points:
                key=(round(x,8),round(z,8))
                if key not in lookup:
                    lookup[key]=len(verts)
                    verts.append((tuple(float(v) for v in np.asarray([x,height(x,z),z],dtype=np.float32)),(0.,0.,0.),(float(x/8),float(z/8)),(1.,0.,0.)))
                ids.append(lookup[key])
            actual=np.asarray([verts[i][0] for i in ids])
            if np.cross(actual[1]-actual[0],actual[2]-actual[0])[1]>1e-9:
                idx.extend(ids)
    flush()
    # Area-weighted vertex normals from the actual upward-wound geometry.
    for _,vv,ii,_ in output:
        ns=np.zeros((len(vv),3));pos=np.asarray([v[0] for v in vv])
        for a,b,c in np.asarray(ii).reshape(-1,3):
            n=np.cross(pos[b]-pos[a],pos[c]-pos[a]);ns[[a,b,c]]+=n
        ns/=np.maximum(np.linalg.norm(ns,axis=1)[:,None],1e-12)
        vv[:]=[(v[0],tuple(ns[i]),v[2],v[3]) for i,v in enumerate(vv)]
    # Read heights from the actual collision triangles at terrain joins,
    # not a second interpolation that could disagree along long mesh edges.
    from shapely.strtree import STRtree
    triangles=[];planes=[]
    for _,vv,ii,mat in output:
        for ids in np.asarray(ii).reshape(-1,3):
            positions=np.asarray([vv[int(i)][0] for i in ids])
            xz=positions[:,[0,2]]
            triangles.append(Polygon(xz))
            planes.append(np.linalg.solve(np.column_stack((xz,np.ones(3))),positions[:,1]))
    tree=STRtree(triangles)
    source_height=height
    def collision_height(x,z):
        point=Point(x,z)
        matches=tree.query(point,predicate='intersects')
        if not len(matches) and surface.distance(point)<1e-4:
            matches=tree.query(point.buffer(1e-4),predicate='intersects')
        if len(matches):return float(np.dot([x,z,1.],planes[int(matches[0])]))
        return source_height(x,z)
    mesh['network_height']=collision_height
    height=collision_height
    markings,mark_count=build_lane_markings(sections,surface,height)
    output.extend(markings)
    return output,{'roads':sum(not r for _,_,r in sections),'roundabouts':sum(r for _,_,r in sections),'markings':mark_count,'warnings':warnings,'surface_area_m2':surface.area}

def carve_terrain(terrain_meshes,mesh):
    """Cut asphalt out of physical terrain and weld the surrounding collar.

    Evaluating the original triangle plane preserves topography away from the
    roads; the new boundary uses the road height field on both sides.
    """
    from shapely.geometry import Point
    surface=mesh['network_surface'];height=mesh['network_height']
    output=[];verts=[];indices=[];lookup={};mat_current=None
    def flush():
        nonlocal verts,indices,lookup
        if indices:output.append((f'1GRASS_NETWORK_{len(output)}',verts,indices,mat_current))
        verts=[];indices=[];lookup={}
    collar=surface.buffer(3.)
    for name,vv,ii,mat in terrain_meshes:
        if name.startswith('ENV_'):
            output.append((name,vv,ii,mat));continue
        if mat_current!=mat:flush();mat_current=mat
        for ids in np.asarray(ii).reshape(-1,3):
            original=np.asarray([vv[int(i)][0] for i in ids]);xz=original[:,[0,2]]
            tri=Polygon(xz)
            if tri.area<1e-9:continue
            cut=tri.difference(surface) if tri.intersects(surface) else tri
            plane=np.linalg.solve(np.column_stack((xz,np.ones(3))),original[:,1])
            for poly in _polygons(cut):
                for t in constrained_delaunay_triangles(poly).geoms:
                    coords=list(t.exterior.coords)[:3];positions=[]
                    for x,z in coords:
                        y=float(np.dot([x,z,1.],plane))
                        point=Point(x,z)
                        if collar.covers(point):
                            d=surface.distance(point);blend=np.clip(d/3.,0.,1.)
                            edge=nearest_points(surface,point)[0]
                            y=height(edge.x,edge.y)*(1-blend)+y*blend
                        positions.append((float(x),float(y),float(z)))
                    a,b,c=np.asarray(positions);normal=np.cross(b-a,c-a)
                    if np.linalg.norm(normal)<1e-9:continue
                    if normal[1]<0:positions[1],positions[2]=positions[2],positions[1];normal=-normal
                    normal/=np.linalg.norm(normal)
                    if len(verts)+3>60000:flush()
                    triangle_ids=[]
                    for p in positions:
                        p=tuple(float(v) for v in np.asarray(p,dtype=np.float32))
                        key=p
                        if key not in lookup:
                            lookup[key]=len(verts);verts.append((p,tuple(normal),(p[0]/8,p[2]/8),(1.,0.,0.)))
                        triangle_ids.append(lookup[key])
                    actual=np.asarray([verts[i][0] for i in triangle_ids])
                    final_normal=np.cross(actual[1]-actual[0],actual[2]-actual[0])
                    # Welding near-coincident clipping intersections can collapse
                    # numerical slivers. Check AFTER welding, before emitting faces.
                    if len(set(triangle_ids))==3 and final_normal[1]>1e-9:
                        indices.extend(triangle_ids)
    flush();return output

def build_lane_markings(sections,surface,height):
    """Visual lane dashes only where OSM confirms multiple lanes.

    Marks follow each road rather than repeating road-texture stripes across
    the world axes. Collision remains the single road surface underneath.
    """
    from shapely.geometry import Point
    output=[];vv=[];ii=[];count=0;seen=set()
    def flush():
        nonlocal vv,ii
        if ii:output.append((f'ENV_LANE_MARKINGS_{len(output)}',vv,ii,'marking'))
        vv=[];ii=[]
    for line,road,ring in sections:
        try:lanes=int(road.get('tags',{}).get('lanes',0))
        except (ValueError,TypeError):continue
        if not 2<=lanes<=6:continue
        w=width_for(road)
        for lane in range(1,lanes):
            offset=-w/2+w*lane/lanes
            path=line.offset_curve(offset)
            for part in _lines(path):
                for start in np.arange(0,part.length,9.):
                    end=min(start+3.,part.length)
                    if end-start<.2:continue
                    coords=[part.interpolate(float(d)).coords[0] for d in np.linspace(start,end,4)]
                    paint=LineString(coords).buffer(.06,cap_style='flat').intersection(surface)
                    key=paint.wkb
                    if key in seen:continue
                    seen.add(key)
                    for poly in _polygons(paint):
                        for tri in constrained_delaunay_triangles(poly).geoms:
                            pts=list(tri.exterior.coords)[:3]
                            a,b,c=np.asarray([(x,0.,z) for x,z in pts])
                            if np.cross(b-a,c-a)[1]<0:pts[1],pts[2]=pts[2],pts[1]
                            if len(vv)+3>60000:flush()
                            base=len(vv)
                            vv.extend([((float(x),height(x,z)+.012,float(z)),(0.,1.,0.),(0.,0.),(1.,0.,0.)) for x,z in pts]);ii.extend((base,base+1,base+2))
                    count+=1
    flush();return output,count
