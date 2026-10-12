"""Regression checks for physical barriers and shared sign assets."""
import unittest
import numpy as np
from test_road_network_v3 import fixture,ll,road
from test_export_pipeline import Kn5Reader
from road_network_v3 import build_network_surfaces
from road_signs import build_signs
from sign_models import sign_geometry
import server

class BarrierModelTests(unittest.TestCase):
    def test_hedges_physical_in_kn5_and_clear_of_every_road(self):
        _,g=fixture()
        m=server.process_road([ll(0,-100),ll(0,-30),ll(0,30),ll(0,100)],6.,0.,elevations=[0.]*4,grass_width=0.)
        roads=[road([(-40,0),(40,0)])]
        network,_=build_network_surfaces({'roads':roads},m,g)
        barriers=[{'type':'hedge','pts':[ll(10,-20),ll(10,20)]},
                  {'type':'hedge','pts':[ll(15,15),ll(16,15)]}]
        env=server.build_environment_meshes({'barriers':barriers},m,g)['meshes']
        physical=[entry for entry in env if entry[0].startswith('1WALL')]
        self.assertTrue(physical)
        for _,vv,ii,_ in physical:
            positions=np.array([v[0] for v in vv],dtype=np.float32)
            for k in range(0,len(ii),3):
                tri=positions[ii[k:k+3]]
                self.assertGreater(np.linalg.norm(np.cross(tri[1]-tri[0],tri[2]-tri[0])),1e-6)
                from shapely.geometry import LineString
                segment=LineString([(p[0],p[2]) for p in tri])
                self.assertFalse(segment.intersects(m['network_surface']))
            self.assertTrue(any(v[0][0]>14 for v in vv))
        m['unified_network']=True
        files=server.build_track_files(m,'hedge_test',network+env)
        kn=Kn5Reader(files['hedge_test.kn5']).parse()
        self.assertTrue(any(n.startswith('1WALL_BARR_TREE') for n in kn.nodes))
        self.assertIn('KEY=WALL',files['data/surfaces.ini'].decode())

    def test_generator_uses_new_plate_clearance_and_meshes(self):
        m,g=fixture();surroundings={'roads':[road([(0,-100),(0,100)])]};build_network_surfaces(surroundings,m,g)
        out,stats=build_signs({**surroundings,'signs':[{'pt':ll(0,0),'tags':{'highway':'give_way'}}]},m)
        self.assertEqual(stats['mapped'],1)
        face=next(vv for name,vv,_,_ in out if 'FACE' in name)
        self.assertAlmostEqual(min(v[0][1] for v in face),2.1,places=6)
        source,_,_,groups=sign_geometry('give_way')
        self.assertEqual(sum(len(ii) for _,_,ii,_ in out),sum(map(len,groups)))
        for _,vv,ii,_ in out:
            for k in range(0,len(ii),3):
                a,b,c=[np.array(vv[j][0]) for j in ii[k:k+3]]
                self.assertGreater(np.dot(np.cross(b-a,c-a),vv[ii[k]][1]),0)

if __name__=='__main__':unittest.main()
