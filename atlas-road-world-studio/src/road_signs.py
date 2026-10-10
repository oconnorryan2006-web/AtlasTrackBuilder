"""Procedural UK controls; OSM speed limits are never guessed."""
import io,math,re
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from shapely.geometry import Point
from road_network_v3 import extract_network,projector,width_for

def sign_texture(kind,label=''):
    im=Image.new('RGB',(256,256),'white');d=ImageDraw.Draw(im)
    def font(size):
        for name in ('DejaVuSans-Bold.ttf','arialbd.ttf'):
            try:return ImageFont.truetype(name,size)
            except OSError:pass
        return ImageFont.load_default(size=size)
    if kind in ('speed','no_entry','mini_roundabout'):
        d.ellipse((8,8,248,248),fill='#cf152d')
        d.ellipse((30,30,226,226),fill='white' if kind=='speed' else '#cf152d')
        if kind=='speed':d.text((128,128),label,fill='black',font=font(94 if len(label)<3 else 70),anchor='mm')
        elif kind=='no_entry':d.rectangle((45,103,211,153),fill='white')
        else:
            d.ellipse((8,8,248,248),fill='white')
            d.ellipse((18,18,238,238),fill='#075aaa')
            for a in (0,120,240):
                d.arc((58,58,198,198),a+10,a+95,fill='white',width=15)
                r=math.radians(a+95);x=128+70*math.cos(r);y=128+70*math.sin(r)
                d.polygon([(x,y),(x+22*math.cos(r-.9),y+22*math.sin(r-.9)),(x+22*math.cos(r+1.4),y+22*math.sin(r+1.4))],fill='white')
    elif kind=='roundabout_warning':
        d.polygon([(128,8),(248,240),(8,240)],fill='#cf152d')
        d.polygon([(128,43),(211,218),(45,218)],fill='white')
        for a in (0,120,240):
            d.arc((87,115,169,197),a+10,a+95,fill='black',width=9)
            r=math.radians(a+95);x=128+41*math.cos(r);y=156+41*math.sin(r)
            d.polygon([(x,y),(x+15*math.cos(r-.9),y+15*math.sin(r-.9)),(x+15*math.cos(r+1.4),y+15*math.sin(r+1.4))],fill='black')
    elif kind=='stop':
        pts=[(128+120*math.cos(math.pi/8+j*math.pi/4),128+120*math.sin(math.pi/8+j*math.pi/4)) for j in range(8)]
        d.polygon(pts,fill='#cf152d',outline='white',width=7)
        d.text((128,128),'STOP',fill='white',font=font(62),anchor='mm')
    elif kind=='give_way':
        d.polygon([(8,15),(248,15),(128,242)],fill='#cf152d')
        d.polygon([(44,36),(212,36),(128,193)],fill='white')
    elif kind=='oneway':
        d.rectangle((8,8,248,248),fill='#075aaa')
        d.polygon([(110,215),(146,215),(146,98),(207,98),(128,25),(49,98),(110,98)],fill='white')
    elif kind=='street_name':
        d.rectangle((5,58,251,198),fill='white',outline='black',width=5)
        size=min(34,max(12,int(420/max(1,len(label)))))
        d.text((128,128),label[:60],fill='black',font=font(size),anchor='mm')
    else:raise ValueError('Unsupported sign '+kind)
    out=io.BytesIO();im.save(out,format='PNG');return out.getvalue()

def _kind(tags):
    code=str(tags.get('traffic_sign','')).lower()
    if tags.get('highway') in ('stop','give_way'):return tags['highway'],''
    if 'maxspeed' in tags:
        m=re.fullmatch(r'\s*(\d+(?:\.\d+)?)\s*(mph|km/h)?\s*',str(tags['maxspeed']))
        # UK speed sign text is mph. Unqualified OSM numeric values are km/h.
        if m:return 'speed',str(round(float(m[1]) if m[2]=='mph' else float(m[1])/1.609344))
    speed=re.search(r'670\[(\d+)\]',code)
    if speed:return 'speed',speed[1]
    if '601.1' in code or code=='stop':return 'stop',''
    if '602' in code or code=='give_way':return 'give_way',''
    if '616' in code or code=='no_entry':return 'no_entry',''
    if '609' in code:return 'mini_roundabout',''
    if '510' in code or code=='roundabout':return 'roundabout_warning',''
    if '652' in code or code=='oneway':return 'oneway',''
    return None,''

def build_signs(surroundings,mesh):
    sections,_,_=extract_network(surroundings,mesh)
    xy=projector(mesh); height=mesh['network_height'];surface=mesh['network_surface']
    candidates=[]
    for s in surroundings.get('signs',[]):
        kind,label=_kind(s.get('tags',{}))
        if kind and s.get('pt'):candidates.append((xy(s['pt']),kind,label,False,s.get('tags',{})))
    # Explicit way maxspeed is real data; placement at section entrances is
    # inferred and reported as such. No guessed junction priority controls.
    for line,road,ring in sections:
        if ring:continue
        tags=road.get('tags',{})
        ring_lines=[other for other,_,is_ring in sections if is_ring]
        rule=tags.get('oneway')
        start=Point(line.coords[0]);end=Point(line.coords[-1])
        if any(loop.distance(end)<.2 for loop in ring_lines) and rule!='-1':
            candidates.append((tuple(line.interpolate(max(0.,line.length-10.)).coords)[0],'roundabout_warning','',True,{}))
        if any(loop.distance(start)<.2 for loop in ring_lines) and rule not in ('yes','1'):
            candidates.append((tuple(line.interpolate(min(10.,line.length)).coords)[0],'roundabout_warning','',True,{'reverse':True}))
        entrance=tuple(line.interpolate(min(8.,line.length/2)).coords)[0]
        if tags.get('oneway') in ('yes','1','-1'):
            reverse=tags.get('oneway')=='-1'
            exit_point=tuple(line.interpolate(max(0.,line.length-8.)).coords)[0]
            candidates.append((exit_point if reverse else entrance,'oneway','',True,{'reverse':reverse}))
            candidates.append((entrance if reverse else exit_point,'no_entry','',True,{'reverse':not reverse}))
        if tags.get('name') and road.get('type') in ('residential','living_street','service'):
            candidates.append((entrance,'street_name',str(tags['name']),True,{}))
        kind,label=_kind({'maxspeed':tags['maxspeed']}) if 'maxspeed' in tags else (None,'')
        if kind:
            candidates.extend([(tuple(line.interpolate(min(8.,line.length/2)).coords)[0],kind,label,True,{}),
                               (tuple(line.interpolate(max(0.,line.length-8.)).coords)[0],kind,label,True,{'reverse':True})])
    result=[];textures={};dedup=[];mapped=inferred=0
    for point,kind,label,guess,tags in candidates:
        if not sections:continue
        line,road,_=min(sections,key=lambda item:item[0].distance(Point(point)))
        if line.distance(Point(point))>15:continue
        if any(k==kind and l==label and np.linalg.norm(np.array(point)-p)<10 for p,k,l in dedup):continue
        t=line.project(Point(point));before=np.array(line.interpolate(max(0,t-.5)).coords[0]);after=np.array(line.interpolate(min(line.length,t+.5)).coords[0]);direction=after-before
        direction/=max(np.linalg.norm(direction),1e-8)
        if tags.get('reverse') or tags.get('direction')=='backward':direction=-direction
        # Left verge in left-handed X/Z (UK traffic); keep geometry clear of
        # every road at junctions, not just the nearest centreline.
        side=np.array([direction[1],-direction[0]])
        centre=np.array(line.interpolate(t).coords[0]);pos=None
        for shift in (0.,4.,8.,12.,16.):
            for extra in (1.,2.,3.,4.):
                candidate=centre-direction*shift+side*(width_for(road)/2+extra)
                if not surface.buffer(.4).covers(Point(candidate)):
                    pos=candidate;break
            if pos is not None:break
        if pos is None:continue
        x,z=pos;y=height(x,z);normal=-direction
        facing=tags.get('traffic_sign:direction',tags.get('direction'))
        facing={'N':0,'E':90,'S':180,'W':270,'NE':45,'SE':135,'SW':225,'NW':315}.get(str(facing).upper(),facing)
        try:
            angle=math.radians(float(facing));normal=np.array([math.sin(angle),-math.cos(angle)])
        except (ValueError,TypeError):pass
        across=np.array([-normal[1],normal[0]])
        import hashlib
        material='sign_'+kind+('_'+hashlib.sha256(label.encode()).hexdigest()[:12] if label else '')
        textures[material]=sign_texture(kind,label)
        vv=[];ii=[]
        # Cylindrical metal post; 8 sides, double-sided face has no collision.
        pole_v=[];pole_i=[]
        for j in range(8):
            a=2*math.pi*j/8
            for yy in (y,y+2.1):pole_v.append(((float(x+.035*math.cos(a)),float(yy),float(z+.035*math.sin(a))),(math.cos(a),0.,math.sin(a)),(j/8,yy-y),(1.,0.,0.)))
        for j in range(8):
            a=2*j;b=2*((j+1)%8);pole_i.extend((a,b,a+1,b,b+1,a+1))
        n=len(result);result.append((f'ENV_SIGN_POST_{n}',pole_v,pole_i,'asphalt'))
        if kind=='give_way':shape=[(-.45,.38),(.45,.38),(0.,-.45)]
        elif kind=='stop':shape=[(.42*math.cos(math.pi/8+j*math.pi/4),.42*math.sin(math.pi/8+j*math.pi/4)) for j in range(8)]
        elif kind=='roundabout_warning':shape=[(-.45,-.38),(.45,-.38),(0.,.45)]
        elif kind=='street_name':shape=[(-.75,-.22),(.75,-.22),(.75,.22),(-.75,.22)]
        elif kind=='oneway':shape=[(-.3,-.4),(.3,-.4),(.3,.4),(-.3,.4)]
        else:shape=[(.38*math.cos(j*2*math.pi/32),.38*math.sin(j*2*math.pi/32)) for j in range(32)]
        uv_w=max(abs(u) for u,v in shape)*2;uv_h=max(abs(v) for u,v in shape)*2
        for u,v in [(0.,0.)]+shape:
            p=pos+across*u+normal*.04
            vv.append(((float(p[0]),float(y+2.05+v),float(p[1])),(float(normal[0]),0.,float(normal[1])),(.5+u/uv_w,.5-v/uv_h),(float(across[0]),0.,float(across[1]))))
        ccw=sum(shape[j][0]*shape[(j+1)%len(shape)][1]-shape[(j+1)%len(shape)][0]*shape[j][1] for j in range(len(shape)))>0
        for j in range(len(shape)):
            a,b=1+j,1+(j+1)%len(shape)
            ii.extend((0,b,a) if ccw else (0,a,b))
        result.append((f'ENV_SIGN_FACE_{kind}_{n}',vv,ii,material))
        # Separate aluminium back and 3 mm plate edge, never mirrored artwork.
        back=[];back_ix=[]
        for u,v in [(0.,0.)]+shape:
            p=pos+across*u+normal*.037
            back.append(((float(p[0]),float(y+2.05+v),float(p[1])),(-float(normal[0]),0.,-float(normal[1])),(0.,0.),(float(across[0]),0.,float(across[1]))))
        for j in range(len(shape)):
            a,b=1+j,1+(j+1)%len(shape)
            back_ix.extend((0,a,b) if ccw else (0,b,a))
        for j in range(len(shape)):
            k=(j+1)%len(shape);base=len(back)
            for index in (j,k):
                for depth in (.037,.04):
                    u,v=shape[index];p=pos+across*u+normal*depth
                    back.append(((float(p[0]),float(y+2.05+v),float(p[1])),(float(across[0]),0.,float(across[1])),(0.,0.),(float(across[0]),0.,float(across[1]))))
            back_ix.extend((base,base+1,base+2,base+1,base+3,base+2))
        result.append((f'ENV_SIGN_BACK_{kind}_{n}',back,back_ix,'building2'))
        dedup.append((np.array(point),kind,label));inferred+=guess;mapped+=not guess
    mesh['sign_textures']=textures
    return result,{'mapped':mapped,'inferred':inferred,'unsupported':len(surroundings.get('signs',[]))-sum(_kind(s.get('tags',{}))[0] is not None for s in surroundings.get('signs',[]))}
