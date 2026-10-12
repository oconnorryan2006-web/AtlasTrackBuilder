"""Shared metre-scale UK sign geometry for KN5 generation and GLB export."""
import math

def sign_geometry(kind, clearance=2.1):
    width = .75 if kind != 'oneway' else .45
    if kind == 'street_name':
        shape = [(-.75,-.22),(.75,-.22),(.75,.22),(-.75,.22)]
        uv = [(5/256,198/256),(251/256,198/256),(251/256,58/256),(5/256,58/256)]
    elif kind == 'give_way':
        h = width * math.sqrt(3) / 2
        shape = [(-width/2, h/3), (0., -2*h/3), (width/2, h/3)]
        uv = [(8/256, 15/256), (.5, 242/256), (248/256, 15/256)]
    elif kind == 'roundabout_warning':
        h = width * math.sqrt(3) / 2
        shape = [(-width/2, -h/3), (width/2, -h/3), (0., 2*h/3)]
        uv = [(8/256, 240/256), (248/256, 240/256), (.5, 8/256)]
    elif kind == 'stop':
        r = width / (2*math.cos(math.pi/8))
        shape = [(r*math.cos(math.pi/8+j*math.pi/4), r*math.sin(math.pi/8+j*math.pi/4)) for j in range(8)]
        uv = [(.5+x/r*120/256, .5-y/r*120/256) for x,y in shape]
    elif kind == 'oneway':
        shape = [(-width/2,-.45),(width/2,-.45),(width/2,.45),(-width/2,.45)]
        uv = [(8/256,248/256),(248/256,248/256),(248/256,8/256),(8/256,8/256)]
    else:
        r = width/2
        shape = [(r*math.cos(j*2*math.pi/64),r*math.sin(j*2*math.pi/64)) for j in range(64)]
        uv = [(.5+x/r*120/256,.5-y/r*120/256) for x,y in shape]
    centre_y = clearance - min(y for _,y in shape)
    vertices=[]; normals=[]; uvs=[]; groups=[[],[],[]]
    def vertex(p,n,uv=(0.,0.)):
        i=len(vertices);vertices.append(p);normals.append(n);uvs.append(uv);return i
    def triangle(group, a,b,c): groups[group].extend((a,b,c))
    front=[vertex((x,y+centre_y,.003),(0.,0.,1.),tex) for (x,y),tex in zip(shape,uv)]
    centre_uv=tuple(sum(tex[i] for tex in uv)/len(uv) for i in (0,1))
    centre=vertex((0.,centre_y,.003),(0.,0.,1.),centre_uv)
    back=[vertex((x,y+centre_y,0.),(0.,0.,-1.)) for x,y in shape]
    backcentre=vertex((0.,centre_y,0.),(0.,0.,-1.))
    for j in range(len(shape)):
        k=(j+1)%len(shape);triangle(0,centre,front[j],front[k]);triangle(1,backcentre,back[k],back[j])
        x,y=shape[j];xx,yy=shape[k];length=math.hypot(xx-x,yy-y);n=((yy-y)/length,-(xx-x)/length,0.)
        a=vertex((x,y+centre_y,0.),n);b=vertex((xx,yy+centre_y,0.),n);c=vertex((xx,yy+centre_y,.003),n);d=vertex((x,y+centre_y,.003),n)
        triangle(1,a,b,c);triangle(1,a,c,d)
    # Pole sits behind the plate; it rises through the plate centre.
    radius=.038;pole_z=-.055;pole_top=centre_y+.1;segments=16
    bottom=[];top=[]
    for j in range(segments):
        a=2*math.pi*j/segments;n=(math.cos(a),0.,math.sin(a))
        bottom.append(vertex((radius*n[0],0.,pole_z+radius*n[2]),n))
        top.append(vertex((radius*n[0],pole_top,pole_z+radius*n[2]),n))
    for j in range(segments):
        k=(j+1)%segments;triangle(2,bottom[j],top[j],bottom[k]);triangle(2,bottom[k],top[j],top[k])
    for y,n,reverse in [(0.,(0.,-1.,0.),False),(pole_top,(0.,1.,0.),True)]:
        c=vertex((0.,y,pole_z),n);ring=[vertex((radius*math.cos(j*2*math.pi/segments),y,pole_z+radius*math.sin(j*2*math.pi/segments)),n) for j in range(segments)]
        for j in range(segments):
            a,b=ring[j],ring[(j+1)%segments]
            triangle(2,c,b,a) if reverse else triangle(2,c,a,b)
    return vertices, normals, uvs, groups
