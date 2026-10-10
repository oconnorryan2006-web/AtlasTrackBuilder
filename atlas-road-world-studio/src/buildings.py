"""OSM footprint buildings, including multipolygon courtyards."""
import numpy as np
from shapely import constrained_delaunay_triangles,make_valid
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union
from scipy.spatial import cKDTree
from scipy.interpolate import LinearNDInterpolator
from road_network_v3 import projector,_polygons

def build_buildings(surroundings,mesh,ground_pts):
    xy=projector(mesh);ground=np.asarray(ground_pts,dtype=float)
    kd=cKDTree(ground[:,[0,2]])
    try:interp=LinearNDInterpolator(ground[:,[0,2]],ground[:,1])
    except Exception:interp=None
    def height(p):
        y=float(interp(*p)) if interp is not None else float('nan')
        return y if np.isfinite(y) else float(ground[kd.query(p)[1],1])
    batches={};output=[];warnings=[];count=0
    def add(name,vv,ii,mat):
        if mat not in batches:batches[mat]=([],[])
        v,i=batches[mat]
        if len(v)+len(vv)>60000:
            output.append((f'ENV_BLDG_{mat}_{len(output)}',v,i,mat));v,i=[],[];batches[mat]=(v,i)
        base=len(v);v.extend(vv);i.extend(base+j for j in ii)
    parts=[]
    for part in surroundings.get('buildings',[]):
        if part.get('part'):
            try:parts.append(make_valid(Polygon([xy(p) for p in part['outline']])))
            except Exception:pass
    for b in surroundings.get('buildings',[]):
        try:
            footprint=make_valid(Polygon([xy(p) for p in b['outline']],[[xy(p) for p in ring] for ring in b.get('holes',[])]))
            if not b.get('part') and parts:
                contained=[p for p in parts if footprint.covers(p.representative_point())]
                if contained:footprint=footprint.difference(unary_union(contained))
            h=float(b.get('height',6.));h=min(150.,max(2.,h))
            if not np.isfinite(h):raise ValueError('Nonfinite height')
        except Exception as e:
            warnings.append('Invalid building '+str(b.get('osm_id',''))+': '+str(e));continue
        for poly in _polygons(footprint):
            if poly.area<1:continue
            # Diagnose overlaps without hiding footprints adjacent to roads.
            road=mesh.get('network_surface')
            if road is not None and poly.intersection(road).area>1:
                warnings.append('Building footprint overlaps carriageway: OSM '+str(b.get('osm_id','unknown')))
            poly=orient(poly,sign=1.)
            tags=b.get('tags',{})
            try:min_h=max(0.,float(str(tags.get('min_height',0)).replace('m','').strip()))
            except (ValueError,TypeError):min_h=0.
            base=min(height(p) for p in poly.exterior.coords)+min_h-(.15 if min_h==0 else 0.)
            top=max(height(p) for p in poly.exterior.coords)+h
            if top<=base:
                warnings.append('Invalid building height range: OSM '+str(b.get('osm_id','unknown')));continue
            mat='commercial' if b.get('kind') in ('commercial','retail','industrial','office','apartments') else ('building','building2','building3')[count%3]
            for ring in [poly.exterior]+list(poly.interiors):
                pts=list(ring.coords)
                for (x,z),(xx,zz) in zip(pts,pts[1:]):
                    length=np.hypot(xx-x,zz-z)
                    if length<1e-7:continue
                    # Outer ring CCW in X/Z: right normal points outwards.
                    normal=((zz-z)/length,0.,-(xx-x)/length);tan=((xx-x)/length,0.,(zz-z)/length)
                    vv=[((x,base,z),normal,(0.,0.),tan),((xx,base,zz),normal,(length/4,0.),tan),((xx,top,zz),normal,(length/4,(top-base)/3.2),tan),((x,top,z),normal,(0.,(top-base)/3.2),tan)]
                    add('wall',vv,[0,2,1,0,3,2],'building' if mat is None else mat)
            for tri in constrained_delaunay_triangles(poly).geoms:
                pts=list(tri.exterior.coords)[:3]
                vv=[((x,top,z),(0.,1.,0.),(x/6,z/6),(1.,0.,0.)) for x,z in pts]
                a,bp,c=np.asarray([v[0] for v in vv])
                ii=[0,1,2] if np.cross(bp-a,c-a)[1]>0 else [0,2,1]
                add('roof',vv,ii,'roof')
            count+=1
    for mat,(vv,ii) in batches.items():
        if ii:output.append((f'ENV_BLDG_{mat}_{len(output)}',vv,ii,mat))
    return output,count,warnings
