"""Check continuity, narrow junction mouths, frontages and driveway exceptions."""
import unittest
from shapely.geometry import Point
from shapely.ops import unary_union
from test_road_network_v3 import fixture,ll,road
from road_network_v3 import build_network_surfaces
from roadside_hedges import hedge_layout,EDGE_OFFSET
import server

class RoadsideHedgeTests(unittest.TestCase):
    def layout(self, surroundings):
        m,g=fixture();build_network_surfaces(surroundings,m,g)
        lines,entrances,height=hedge_layout(surroundings,m)
        return m,g,unary_union(lines),entrances,height

    def test_unmapped_backroad_has_continuous_both_sides(self):
        _,_,hedges,_,_=self.layout({})
        for x in (-3-EDGE_OFFSET,3+EDGE_OFFSET):
            for z in range(-95,96):
                self.assertLess(hedges.distance(Point(x,z)),1e-5)
        self.assertGreater(hedges.distance(Point(0,100)),2.)

    def test_turnoff_hugs_both_road_edges_and_only_its_mouth_is_open(self):
        s={'roads':[road([(0,0),(45,0)],width='4')]}
        m,_,hedges,_,_=self.layout(s)
        # The west verge is uninterrupted, including opposite the junction.
        for z in range(-90,91):self.assertLess(hedges.distance(Point(-3.35,z)),1e-5)
        # East verge resumes immediately beyond a 4 m turnoff + 35 cm offset.
        for z in (-3.,3.):self.assertLess(hedges.distance(Point(3.35,z)),1e-5)
        for x in (4.,8.,20.,40.):
            for z in (-2.35,2.35):self.assertLess(hedges.distance(Point(x,z)),1e-5)
        self.assertGreater(hedges.distance(Point(3.35,0)),2.)
        self.assertFalse(hedges.intersects(m['network_surface']))

    def test_house_frontage_lower_but_continuous_and_exported(self):
        house={'outline':[ll(10,-5),ll(18,-5),ll(18,5),ll(10,5),ll(10,-5)],'kind':'house'}
        m,g,hedges,_,height=self.layout({'buildings':[house]})
        self.assertLess(hedges.distance(Point(3.35,0)),1e-5)
        self.assertEqual(height(3.35,0),.85)
        self.assertEqual(height(3.35,60),1.5)
        out=server.build_environment_meshes({'buildings':[house]},m,g)['meshes']
        tops=[v[0] for name,vv,_,_ in out if name.startswith('1WALL_BARR_HEDGE') for v in vv if v[0][1]>0]
        self.assertTrue(any(abs(x-3.35)<1e-5 and abs(z)<1 and abs(y-.85)<1e-5 for x,y,z in tops))

    def test_private_driveway_open_without_hedges_along_it(self):
        drive=road([(0,0),(20,0)],service='driveway',access='private',width='3')
        drive['type']='service'
        m,_,hedges,entrances,_=self.layout({'roads':[drive]})
        self.assertLess(hedges.intersection(entrances).length,1e-6)
        self.assertGreater(hedges.distance(Point(3.35,0)),1.5)
        for z in (-2.,2.):self.assertLess(hedges.distance(Point(3.35,z)),1e-5)
        self.assertGreater(hedges.distance(Point(15,1.85)),5.)
        self.assertFalse(hedges.intersects(m['network_surface']))

    def test_public_driveway_does_not_receive_side_hedges(self):
        drive=road([(0,0),(20,0)],service='driveway',width='3')
        drive['type']='service'
        _,_,hedges,_,_=self.layout({'roads':[drive]})
        for x in (4.,8.,15.):
            for z in (-1.85,1.85):self.assertGreater(hedges.distance(Point(x,z)),.1)
        for z in (-2.,2.):self.assertLess(hedges.distance(Point(3.35,z)),1e-5)

    def test_empty_osm_still_exports_opaque_hedges(self):
        import io
        from PIL import Image
        m,g=fixture();build_network_surfaces({},m,g)
        out=server.build_environment_meshes({},m,g)['meshes']
        self.assertTrue(any(mat=='hedge' for _,_,_,mat in out))
        texture=Image.open(io.BytesIO(server.build_hedge_texture()))
        self.assertEqual(texture.mode,'RGB')

    def test_sparse_mapped_hedges_do_not_disable_continuous_fallback(self):
        _,_,hedges,_,_=self.layout({'barriers':[{'type':'hedge','pts':[ll(3.35,-50),ll(3.35,-45)]}]})
        for z in range(-90,91):self.assertLess(hedges.distance(Point(3.35,z)),1e-5)

if __name__=='__main__':unittest.main()
