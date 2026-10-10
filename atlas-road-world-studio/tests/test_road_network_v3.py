import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import math,unittest, tempfile, json, pathlib, io
from unittest.mock import patch
import numpy as np
from pyproj import Transformer
from shapely.geometry import Point,Polygon,LineString
from shapely.ops import unary_union
from PIL import Image
from road_network_v3 import width_for,_resample,build_network_surfaces,extract_network,carve_terrain
from buildings import build_buildings
from road_signs import build_signs,sign_texture

INV=Transformer.from_crs('+proj=tmerc +lat_0=55 +lon_0=-7 +units=m','EPSG:4326',always_xy=True)
def ll(x,z):
    lon,lat=INV.transform(x,-z);return [lat,lon]
def road(points,**tags):
    return {'pts':[ll(x,z) for x,z in points],'type':'residential','tags':tags,'junction':tags.get('junction')}
def fixture():
    mesh={'proj':{'mid_lat':55.,'mid_lon':-7.,'ox':0.,'oz':0.},'centerline':[(0.,0.,-100.),(0.,0.,100.)],'stats':{'road_width':6.},'vertices':[(-3.,0.,-100.),(3.,0.,-100.),(-3.,0.,100.),(3.,0.,100.)]}
    ground=[(x,0.,z) for x in range(-250,251,10) for z in range(-250,251,10)]
    return mesh,ground
def area(meshes):
    return unary_union([Polygon([(vs[j][0][0],vs[j][0][2]) for j in ix[i:i+3]]) for _,vs,ix,mat in meshes if mat=='road' for i in range(0,len(ix),3)])
def circle(cx=80,r=35):return [(cx+r*math.cos(j*2*math.pi/32),r*math.sin(j*2*math.pi/32)) for j in range(33)]

class CorridorTests(unittest.TestCase):
    def test_lane_width(self):
        self.assertEqual(width_for({'type':'residential','tags':{'lanes':'2'}}),6.5)
        self.assertEqual(width_for({'type':'residential','tags':{'width':'7 m'}}),7.)
    def test_20m_and_45m_roads_included_51m_excluded(self):
        m,g=fixture();out,s=build_network_surfaces({'roads':[road([(x,-80),(x,80)]) for x in (20,45,51)]},m,g)
        a=area(out);self.assertEqual(s['roads'],2)
        self.assertTrue(a.covers(Point(20,0)));self.assertTrue(a.covers(Point(45,0)));self.assertFalse(a.covers(Point(51,0)))
    def test_crossing_sparse_route_and_strict_clip(self):
        m,g=fixture();out,s=build_network_surfaces({'roads':[road([(-200,0),(200,0)])]},m,g)
        a=area(out);self.assertTrue(a.covers(Point(0,0)));self.assertTrue(a.covers(Point(49,0)))
        self.assertFalse(a.covers(Point(51,0)));self.assertAlmostEqual(a.bounds[0],-50,places=5)
    def test_physical_t_and_cross_junctions_no_duplicate_area(self):
        m,g=fixture();out,_=build_network_surfaces({'roads':[road([(-30,0),(30,0)]),road([(20,0),(20,40)])]},m,g)
        union=area(out)
        tri_area=sum(Polygon([(vs[j][0][0],vs[j][0][2]) for j in ix[i:i+3]]).area for _,vs,ix,mat in out for i in range(0,len(ix),3))
        self.assertAlmostEqual(tri_area,union.area,places=6)
        self.assertEqual(union.geom_type,'Polygon')
        for p in [(0,0),(20,0),(20,20),(20,39)]:self.assertTrue(union.covers(Point(p)))
    def test_private_motor_access_and_paths_excluded(self):
        m,g=fixture();out,s=build_network_surfaces({'roads':[road([(20,0),(20,50)],motor_vehicle='no'),dict(road([(30,0),(30,50)]),type='footway')]},m,g)
        self.assertEqual(s['roads'],0)
    def test_continuous_height_between_sparse_route_samples(self):
        m,g=fixture();m['centerline']=[(0.,0.,0.),(6.,.6,0.)];m.pop('vertices')
        build_network_surfaces({'roads':[road([(0,0),(6,0)],width='6')]},m,g)
        h=m['network_height']
        self.assertAlmostEqual(h(2.999,0.),.2999,places=4)
        self.assertAlmostEqual(h(3.001,0.),.3001,places=4)
    def test_invalid_coordinates_fail(self):
        m,g=fixture()
        with self.assertRaises(ValueError):build_network_surfaces({'roads':[{'pts':[[55,-7],[float('nan'),-7]],'type':'residential'}]},m,g)
    def test_grade_separation_does_not_become_false_junction(self):
        m,g=fixture()
        with self.assertRaisesRegex(ValueError,'elevation profile'):build_network_surfaces({'roads':[road([(-20,0),(20,0)],bridge='yes',layer='1')]},m,g)

class RoundaboutTests(unittest.TestCase):
    def test_full_loop_outside_corridor_and_island_hole(self):
        m,g=fixture();out,s=build_network_surfaces({'roads':[road(circle(),junction='roundabout',oneway='yes'),road([(0,0),(45,0)])]},m,g)
        a=area(out);self.assertEqual(s['roundabouts'],1);self.assertFalse(a.covers(Point(80,0)))
        self.assertEqual(a.geom_type,'Polygon');self.assertGreaterEqual(len(a.interiors),1)
        for angle in np.linspace(0,2*math.pi,100):self.assertTrue(a.covers(Point(80+35*math.cos(angle),35*math.sin(angle))))
        self.assertTrue(a.covers(Point(114,0)))
    def test_split_osm_arcs_assembled_without_chord(self):
        m,g=fixture();pts=circle();out,s=build_network_surfaces({'roads':[road(pts[:17],junction='roundabout'),road(pts[16:],junction='roundabout')]},m,g)
        self.assertEqual(s['roundabouts'],1);self.assertFalse(area(out).covers(Point(80,0)))
    def test_oneway_never_changes_collision(self):
        surfaces=[]
        for rule in ('yes','no','-1'):
            m,g=fixture();out,_=build_network_surfaces({'roads':[road(circle(),junction='roundabout',oneway=rule)]},m,g);surfaces.append(area(out))
        self.assertTrue(surfaces[0].equals(surfaces[1]));self.assertTrue(surfaces[0].equals(surfaces[2]))
    def test_incomplete_loop_rejected(self):
        m,g=fixture()
        with self.assertRaisesRegex(ValueError,'Incomplete'):build_network_surfaces({'roads':[road(circle()[8:24],junction='roundabout')]},m,g)
    def test_resampling_does_not_move_mapped_vertices(self):
        p=np.array([[0.,10.],[10.,0.],[0.,-10.],[-10.,0.],[0.,10.]])
        q=_resample(p,True)
        for point in p:self.assertTrue(any(np.allclose(point,x) for x in q))
        self.assertTrue(np.allclose(q[0],q[-1]))

class AssetTests(unittest.TestCase):
    def test_normals_winding_and_mesh_integrity(self):
        m,g=fixture();out,_=build_network_surfaces({'roads':[road(circle(),junction='roundabout'),road([(0,0),(45,0)])]},m,g)
        for name,vv,ii,mat in out:
            self.assertTrue(name.startswith('1ROAD_'));self.assertLess(len(vv),65536)
            v=np.array([x[0] for x in vv]);self.assertTrue(np.isfinite(v).all())
            for a,b,c in np.asarray(ii).reshape(-1,3):self.assertGreater(np.cross(v[b]-v[a],v[c]-v[a])[1],1e-9)
            for vertex in vv:self.assertGreater(vertex[1][1],0.)
    def test_building_courtyard_and_complex_footprint(self):
        m,g=fixture();outer=[(20,0),(40,0),(40,20),(20,20),(20,0)];hole=[(25,5),(35,5),(35,15),(25,15),(25,5)]
        b={'outline':[ll(*p) for p in outer],'holes':[[ll(*p) for p in hole]],'height':8.}
        out,count,w=build_buildings({'buildings':[b]},m,g);self.assertEqual(count,1)
        roofs=unary_union([Polygon([(vv[j][0][0],vv[j][0][2]) for j in ii[i:i+3]]) for _,vv,ii,mat in out if mat=='roof' for i in range(0,len(ii),3)])
        self.assertAlmostEqual(roofs.area,300.,places=4);self.assertFalse(roofs.covers(Point(30,10)))
        # The former generator silently discarded footprints above 60 points.
        complex_b={'outline':[ll(80+10*math.cos(a),40+10*math.sin(a)) for a in np.linspace(0,2*math.pi,81)],'height':5.}
        self.assertEqual(build_buildings({'buildings':[complex_b]},m,g)[1],1)
    def test_building_ground_contact_and_height(self):
        m,g=fixture();g=[(x,.05*x,z) for x,_,z in g]
        b={'outline':[ll(20,0),ll(30,0),ll(30,10),ll(20,10),ll(20,0)],'height':6.}
        out,count,_=build_buildings({'buildings':[b]},m,g)
        ys=[v[0][1] for _,vv,ii,mat in out for v in vv]
        self.assertLessEqual(min(ys),1.);self.assertAlmostEqual(max(ys),7.5,places=5)
    def test_mapped_signs_and_explicit_speed_textures(self):
        m,g=fixture();s={'roads':[road([(20,-80),(20,80)],maxspeed='30 mph',oneway='yes',name='Test Street')],'signs':[{'pt':ll(20,0),'tags':{'highway':'stop'}}]}
        build_network_surfaces(s,m,g);out,counts=build_signs(s,m)
        self.assertEqual(counts['mapped'],1);self.assertGreaterEqual(counts['inferred'],3)
        names=[n for n,_,_,_ in out];self.assertTrue(any('stop' in n for n in names));self.assertTrue(any('speed' in n for n in names));self.assertTrue(any('oneway' in n for n in names))
        for name,vv,ii,mat in out:
            if 'FACE' in name:self.assertFalse(m['network_surface'].buffer(.1).covers(Point(vv[0][0][0],vv[0][0][2])))
        for tex in m['sign_textures'].values():
            im=Image.open(io.BytesIO(tex));self.assertGreaterEqual(len(im.getcolors(65536)),2)
    def test_reverse_oneway_signs_swap_ends(self):
        m,g=fixture();s={'roads':[road([(20,-50),(20,50)],oneway='-1')]};build_network_surfaces(s,m,g)
        out,counts=build_signs(s,m)
        one=next(vv for n,vv,ii,mat in out if 'FACE_oneway' in n)
        no=next(vv for n,vv,ii,mat in out if 'FACE_no_entry' in n)
        self.assertGreater(one[0][0][2],30.)
        self.assertLess(no[0][0][2],-30.)
    def test_mini_roundabout_texture_has_blue_face_white_rim(self):
        im=Image.open(io.BytesIO(sign_texture('mini_roundabout')))
        self.assertEqual(im.getpixel((128,10)),(255,255,255))
        self.assertEqual(im.getpixel((128,23)),(7,90,170))
        from road_signs import _kind
        self.assertEqual(_kind({'traffic_sign':'GB:609'})[0],'mini_roundabout')
        self.assertEqual(_kind({'traffic_sign':'GB:510'})[0],'roundabout_warning')
        warning=Image.open(io.BytesIO(sign_texture('roundabout_warning')))
        self.assertEqual(warning.getpixel((128,15)),(207,21,45))
    def test_no_invented_speed_limit(self):
        m,g=fixture();s={'roads':[road([(20,-50),(20,50)])]};build_network_surfaces(s,m,g)
        out,count=build_signs(s,m);self.assertFalse(any('speed' in n for n,_,_,_ in out))
    def test_valid_thin_triangle_avoids_float32_cross_product_cancellation(self):
        from mesh_validation import validate_meshes
        # These serialized positions have exactly 0.5 square metres of area.
        points=[(0.,0.,0.),(12346.,0.,12345.),(12345.,0.,12344.)]
        vv=[(p,(0.,1.,0.),(0.,0.),(1.,0.,0.)) for p in points]
        validate_meshes([('1GRASS_NETWORK_0',vv,[0,1,2],'terrain')])
        with self.assertRaisesRegex(ValueError,'Degenerate'):
            validate_meshes([('1GRASS_NETWORK_0',vv,[0,1,1],'terrain')])
        with self.assertRaisesRegex(ValueError,'Inverted'):
            validate_meshes([('1GRASS_NETWORK_0',vv,[0,2,1],'terrain')])
    def test_clipping_slivers_are_checked_after_vertex_welding(self):
        from mesh_validation import validate_meshes
        m,g=fixture();build_network_surfaces({'roads':[]},m,g)
        positions=[(10.,0.,0.),(10.00000002,0.,0.),(10.,0.,1.),(20.,0.,0.),(20.,0.,1.),(21.,0.,1.)]
        vv=[(p,(0.,1.,0.),(0.,0.),(1.,0.,0.)) for p in positions]
        out=carve_terrain([('1GRASS_TEST',vv,[0,2,1,3,4,5],'terrain')],m)
        validate_meshes(out)
        self.assertEqual(sum(len(ii) for _,vv,ii,mat in out),3)
    def test_terrain_is_cut_out_at_road_boundary(self):
        m,g=fixture();build_network_surfaces({'roads':[road([(-40,0),(40,0)])]},m,g)
        vv=[((x,0.,z),(0.,1.,0.),(0.,0.),(1.,0.,0.)) for x,z in [(-60,-60),(60,-60),(60,60),(-60,60)]]
        out=carve_terrain([('1GRASS_TEST',vv,[0,2,1,0,3,2],'terrain')],m)
        for name,vv,ii,mat in out:
            for i in range(0,len(ii),3):
                t=Polygon([(vv[j][0][0],vv[j][0][2]) for j in ii[i:i+3]])
                self.assertLess(t.intersection(m['network_surface']).area,1e-6)

class RetrievalTests(unittest.TestCase):
    def test_sparse_route_queries_intermediate_boxes_and_no_cap(self):
        import server
        calls=[]
        def fetch(q,timeout):
            calls.append(q)
            return {'elements':[{'type':'way','id':1,'tags':{'highway':'residential'},'geometry':[{'lat':55.,'lon':-7.},{'lat':55.01,'lon':-7.}]}]}
        with tempfile.TemporaryDirectory() as d,patch.object(server,'OUTPUT_DIR',d),patch.object(server,'_overpass',side_effect=fetch),patch.object(server.time,'sleep'):
            out=server.fetch_surroundings([ll(0,0),ll(0,1000)])
        self.assertGreaterEqual(len(calls),6);self.assertTrue(all('out geom 3000' not in q for q in calls));self.assertTrue(all('relation["building"]' in q for q in calls))
    def test_partial_download_cannot_silently_drop_buildings(self):
        import server
        data={'elements':[{'type':'way','id':1,'tags':{'highway':'residential'},'geometry':[{'lat':55.,'lon':-7.},{'lat':55.01,'lon':-7.}]}]}
        with tempfile.TemporaryDirectory() as d,patch.object(server,'OUTPUT_DIR',d),patch.object(server,'_overpass',side_effect=[data,IOError('offline')]),patch.object(server.time,'sleep'):
            with self.assertRaisesRegex(IOError,'incomplete'):server.fetch_surroundings([ll(0,0),ll(0,100)])

if __name__=='__main__':unittest.main()
