import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
"""Recorded OSM integration and binary KN5 asset checks (no network needed)."""
import io,json,struct,tempfile,unittest,zipfile
from pathlib import Path
from unittest.mock import patch
import numpy as np
from PIL import Image
import server
from road_network_v3 import build_network_surfaces,projector
from buildings import build_buildings
from road_signs import build_signs
from shapely.geometry import Point

class Kn5Reader:
    def __init__(self,data):self.s=io.BytesIO(data);self.textures={};self.materials=[];self.nodes={}
    def read(self,n):
        out=self.s.read(n)
        if len(out)!=n:raise ValueError('Truncated KN5')
        return out
    def u32(self):return struct.unpack('<I',self.read(4))[0]
    def text(self):return self.read(self.u32()).decode('utf8')
    def parse(self):
        assert self.read(6)==b'sc6969';assert self.u32()==5
        for _ in range(self.u32()):
            self.u32();name=self.text();self.textures[name]=self.read(self.u32())
        for _ in range(self.u32()):
            name=self.text();shader=self.text();self.read(6)
            for _ in range(self.u32()):self.text();self.read(40)
            for _ in range(self.u32()):self.text();self.u32();assert self.text() in self.textures
            self.materials.append(name)
        def node():
            kind=self.u32();name=self.text();children=self.u32();self.read(1)
            if kind==1:self.read(64);self.nodes[name]={'kind':kind}
            elif kind==2:
                self.read(3);count=self.u32();vertices=np.frombuffer(self.read(count*44),dtype='<f4').reshape(count,11)
                n=self.u32();ix=np.frombuffer(self.read(n*2),dtype='<u2');mat=self.u32();self.read(29)
                assert n%3==0 and max(ix)<count and count<65536
                assert np.isfinite(vertices).all() and mat<len(self.materials)
                self.nodes[name]={'kind':kind,'vertices':vertices,'indices':ix,'material':self.materials[mat]}
            else:raise ValueError('Unsupported KN5 node')
            for _ in range(children):node()
        node();assert self.s.read()==b''
        return self

class IntegrationTests(unittest.TestCase):
    def test_recorded_limavady_ring_buildings_and_export_assets(self):
        base=Path(__file__).parent/'fixtures'
        raw=json.loads((base/'limavady.json').read_text());coords=json.loads((base/'limavady-route.json').read_text())
        with tempfile.TemporaryDirectory() as d,patch.object(server,'OUTPUT_DIR',d),patch.object(server,'_overpass',return_value=raw),patch.object(server.time,'sleep'):
            surroundings=server.fetch_surroundings(coords)
        mesh=server.process_road(coords,6.5,0.,elevations=[0.]*len(coords),grass_width=0.)
        ground=[(x,0.,z) for x in range(-300,301,10) for z in range(-600,301,10)]
        out,stats=build_network_surfaces(surroundings,mesh,ground)
        self.assertEqual(stats['roundabouts'],1)
        xy=projector(mesh)
        centre=xy([55.0514846,-6.92766959]);self.assertFalse(mesh['network_surface'].covers(Point(centre)))
        for e in raw['elements']:
            if e.get('tags',{}).get('junction')=='roundabout':
                for p in e['geometry']:self.assertTrue(mesh['network_surface'].buffer(.001).covers(Point(xy([p['lat'],p['lon']]))))
        buildings,n,warning=build_buildings(surroundings,mesh,ground);self.assertEqual(n,10)
        signs,counts=build_signs(surroundings,mesh);self.assertEqual(counts['mapped'],0);self.assertGreater(counts['inferred'],0)
        mesh['unified_network']=True
        files=server.build_track_files(mesh,'limavady_test',out+buildings+signs)
        server.validate_track_files(files,'limavady_test');kn=Kn5Reader(files['limavady_test.kn5']).parse()
        self.assertTrue(any(n.startswith('1ROAD_NETWORK') for n in kn.nodes))
        self.assertFalse('1ROAD' in kn.nodes,'Separate selected-road collision must not overlap the network')
        self.assertTrue(any(n.startswith('ENV_BLDG') for n in kn.nodes));self.assertTrue(any(n.startswith('ENV_SIGN_FACE') for n in kn.nodes))
        for name in ('AC_START_0','AC_HOTLAP_START_0','AC_PIT_0'):self.assertIn(name,kn.nodes)
        for name,data in kn.textures.items():Image.open(io.BytesIO(data)).verify()
        self.assertTrue(any(n.startswith('sign_') for n in kn.textures));self.assertIn('OSM_ATTRIBUTION.txt',files)
    def test_building_parts_replace_parent_volume(self):
        from test_road_network_v3 import fixture,ll
        m,g=fixture()
        footprint=[ll(20,20),ll(30,20),ll(30,30),ll(20,30),ll(20,20)]
        out,n,w=build_buildings({'buildings':[{'outline':footprint,'height':6.},{'outline':footprint,'height':12.,'part':True}]},m,g)
        self.assertEqual(n,1)
        self.assertAlmostEqual(max(v[0][1] for _,vv,ii,mat in out for v in vv),12.,places=5)
    def test_full_export_pipeline_with_explicit_synthetic_sources(self):
        from test_road_network_v3 import ll,road
        coords=[ll(0,-80),ll(0,-30),ll(0,30),ll(0,80)]
        surroundings={'roads':[road([(-40,0),(40,0)],lanes='2',maxspeed='30 mph')],
                      'buildings':[{'outline':[ll(20,20),ll(30,20),ll(30,30),ll(20,30),ll(20,20)],'height':6.}],
                      'signs':[{'pt':ll(0,0),'tags':{'highway':'give_way'}}]}
        vv=[((x,0.,z),(0.,1.,0.),(0.,0.),(1.,0.,0.)) for x,z in [(-100,-100),(100,-100),(100,100),(-100,100)]]
        profile={'elevs':[0.]*4,'dists':[0.,50.,110.,160.],'coords':coords,'smooth_sigma':8.}
        with tempfile.TemporaryDirectory() as d,patch.object(server,'OUTPUT_DIR',d),patch.object(server,'fetch_elevation_profile',return_value=profile),patch.object(server,'fetch_land_cover',return_value=lambda lat,lon:30),patch.object(server,'fetch_surroundings',return_value=surroundings),patch.object(server,'fetch_terrain_grid',return_value={}),patch.object(server,'build_terrain_meshes',return_value=[('1GRASS_TEST',vv,[0,2,1,0,3,2],'terrain')]),patch.object(server,'build_far_terrain',return_value=[]),patch.object(server,'find_assetto_corsa',return_value=None):
            result=server.export_ac_package(coords,6.,0.,'integration_fixture')
            self.assertNotIn('error',result);self.assertEqual(result['buildings'],1);self.assertEqual(result['signs']['mapped'],1)
            with zipfile.ZipFile(Path(d)/result['filename']) as z:
                kn=Kn5Reader(z.read('content/tracks/integration_fixture/integration_fixture.kn5')).parse()
                self.assertTrue(any('give_way' in n for n in kn.nodes))

if __name__=='__main__':unittest.main()
