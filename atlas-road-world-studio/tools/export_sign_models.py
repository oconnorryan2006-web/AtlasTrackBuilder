"""Export approximate procedural UK signs as self-contained metre-scale GLBs.

Run: python export_sign_models.py --output /path/to/models
No external service or model-generation API is used. Front is +Z, Y is up,
and the origin is the post's ground contact. See the generated README.
"""
import argparse
import json
import math
import struct
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from road_signs import sign_texture
from sign_models import sign_geometry


def export_sign(kind, label, path, clearance=2.1):
    vertices, normals, uvs, groups = sign_geometry(kind, clearance)
    binary=bytearray();views=[];accessors=[]
    def append(data,target=None):
        while len(binary)%4:binary.append(0)
        offset=len(binary);binary.extend(data);v={'buffer':0,'byteOffset':offset,'byteLength':len(data)}
        if target:v['target']=target
        views.append(v);return len(views)-1
    def accessor(values,component,typ,fmt,target):
        flattened=[x for row in values for x in row] if typ!='SCALAR' else values
        view=append(struct.pack('<'+fmt*len(flattened),*flattened),target)
        a={'bufferView':view,'componentType':component,'count':len(values),'type':typ}
        if typ=='VEC3' and values is vertices:
            a['min']=[min(p[i] for p in vertices) for i in range(3)];a['max']=[max(p[i] for p in vertices) for i in range(3)]
        accessors.append(a);return len(accessors)-1
    pos=accessor(vertices,5126,'VEC3','f',34962);normal=accessor(normals,5126,'VEC3','f',34962);tex=accessor(uvs,5126,'VEC2','f',34962)
    primitives=[]
    for material,indices in enumerate(groups):
        index=accessor(indices,5123,'SCALAR','H',34963)
        primitives.append({'attributes':{'POSITION':pos,'NORMAL':normal,'TEXCOORD_0':tex},'indices':index,'material':material,'mode':4})
    image=append(sign_texture(kind,label))
    doc={'asset':{'version':'2.0','generator':'ATLAS procedural UK sign exporter'},'scene':0,'scenes':[{'nodes':[0]}],
         'nodes':[{'name':path.stem,'mesh':0}], 'meshes':[{'primitives':primitives}],
         'buffers':[{'byteLength':len(binary)}],'bufferViews':views,'accessors':accessors,
         'images':[{'bufferView':image,'mimeType':'image/png'}], 'textures':[{'source':0,'sampler':0}],
         'samplers':[{'magFilter':9729,'minFilter':9987,'wrapS':33071,'wrapT':33071}],
         'materials':[{'name':'reflective_front','pbrMetallicRoughness':{'baseColorTexture':{'index':0},'metallicFactor':0.,'roughnessFactor':.45}},
                      {'name':'grey_aluminium_back_and_edge','pbrMetallicRoughness':{'baseColorFactor':[.55,.57,.59,1.],'metallicFactor':.9,'roughnessFactor':.5}},
                      {'name':'galvanised_post','pbrMetallicRoughness':{'baseColorFactor':[.48,.5,.52,1.],'metallicFactor':1.,'roughnessFactor':.6}}],
         'extras':{'units':'metres','up_axis':'+Y','front_axis':'+Z','origin':'pole_ground_contact','plate_thickness_m':.003,'minimum_plate_clearance_m':clearance,'approximate_artwork':True,'kind':kind,'label':label}}
    data=json.dumps(doc,separators=(',',':')).encode();data+=b' '*((-len(data))%4);binary+=b'\0'*((-len(binary))%4)
    glb=struct.pack('<4sII',b'glTF',2,12+8+len(data)+8+len(binary))+struct.pack('<I4s',len(data),b'JSON')+data+struct.pack('<I4s',len(binary),b'BIN\0')+binary
    path.write_bytes(glb);return validate_glb(path)


def validate_glb(path):
    data=path.read_bytes();magic,version,length=struct.unpack_from('<4sII',data)
    if magic!=b'glTF' or version!=2 or length!=len(data):raise ValueError('Invalid GLB header')
    offset=12;chunks=[]
    while offset<len(data):
        n,kind=struct.unpack_from('<I4s',data,offset);offset+=8
        if n%4 or offset+n>len(data):raise ValueError('Invalid GLB chunk bounds')
        chunks.append((kind,data[offset:offset+n]));offset+=n
    if [c[0] for c in chunks]!=[b'JSON',b'BIN\0']:raise ValueError('Invalid chunks')
    doc=json.loads(chunks[0][1]);binary=chunks[1][1]
    for v in doc['bufferViews']:
        if v.get('byteOffset',0)+v['byteLength']>len(binary):raise ValueError('View outside buffer')
    def read_accessor(index):
        a=doc['accessors'][index];v=doc['bufferViews'][a['bufferView']];size={'SCALAR':1,'VEC2':2,'VEC3':3}[a['type']];fmt={5126:'f',5123:'H'}[a['componentType']]
        count=a['count']*size;start=v.get('byteOffset',0)+a.get('byteOffset',0)
        if start+struct.calcsize(fmt)*count>v.get('byteOffset',0)+v['byteLength']:raise ValueError('Accessor outside view')
        return struct.unpack_from('<'+fmt*count,binary,start),a
    for p in doc['meshes'][0]['primitives']:
        indices,_=read_accessor(p['indices']);positions,a=read_accessor(p['attributes']['POSITION']);normals,_=read_accessor(p['attributes']['NORMAL'])
        if len(indices)%3 or max(indices)>=a['count']:raise ValueError('Invalid triangle indices')
        if not all(math.isfinite(x) for x in positions+normals):raise ValueError('Nonfinite mesh')
        for i in range(0,len(indices),3):
            aa,bb,cc=[positions[3*j:3*j+3] for j in indices[i:i+3]];u=[bb[k]-aa[k] for k in range(3)];v=[cc[k]-aa[k] for k in range(3)];cross=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
            if sum(x*x for x in cross)<1e-18:raise ValueError('Degenerate triangle')
            normal=normals[3*indices[i]:3*indices[i]+3]
            if sum(cross[k]*normal[k] for k in range(3))<=0:raise ValueError('Triangle winding disagrees with normals')
    image=doc['bufferViews'][doc['images'][0]['bufferView']];start=image['byteOffset']
    if binary[start:start+8]!=b'\x89PNG\r\n\x1a\n':raise ValueError('Missing embedded PNG')
    return {'file':path.name,'bytes':len(data),'vertices':doc['accessors'][0]['count'],'triangles':sum(doc['accessors'][p['indices']]['count']//3 for p in doc['meshes'][0]['primitives'])}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,default=Path('output/uk-sign-models'));args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    signs=[('stop',''),('give_way',''),('no_entry',''),('oneway',''),('street_name','Broad Road')]+[('speed',str(n)) for n in (20,30,40,50,60,70)]
    # Availability follows road_signs' supported names; never silently export
    # the old incorrect red-rimmed roundabout artwork under a new label.
    for kind in ('mini_roundabout','roundabout_warning'):
        try:sign_texture(kind)
        except ValueError:continue
        signs.append((kind,''))
    results=[export_sign(k,label,args.output/(k+('_'+label.replace(' ','_') if label else '')+'.glb')) for k,label in signs]
    (args.output/'manifest.json').write_text(json.dumps(results,indent=2))
    (args.output/'README.md').write_text('# Procedural UK sign models\n\nSelf-contained GLB 2.0 files with embedded PNG faces, grey 3 mm aluminium backs/edges and 76 mm galvanised posts. Units: metres; +Y up; +Z front; ground-contact origin. Lowest plate edge is 2.1 m above the origin. Models use approximate dimensions/artwork and a fallback font, not certified DfT drawings or an exact Transport typeface. Diagram 609 is specifically a mini-roundabout; any warning sign is distinct. No paid API or third-party model pack was used.\n\nFront faces use standard nonmetallic PBR, not physically certified retroreflective rendering. GLB validation checks header/chunks, buffer bounds, embedded PNG, finite mesh coordinates, indices, nondegenerate triangles and winding. Appearance and KN5/game import require separate inspection.\n\nRegenerate with: `python export_sign_models.py --output /path/to/models`. Front artwork is reused from road_signs.sign_texture. See UK_SIGN_REFERENCES.md for authoritative source links, verification limitations and asset/font rights considerations.\n')
    print(json.dumps(results,indent=2))

if __name__=='__main__':main()
