import math
import unittest
from project import camera_coordinates, clip_segment, project

C={'fx':100.,'fy':100.,'cx':50.,'cy':50.,'width':100,'height':100,'near':1.}

class ProjectionTests(unittest.TestCase):
    def test_principal_axis_and_scale(self):
        self.assertEqual(project((0,0,2),C),(50,50))
        self.assertEqual(project((1,0,2),C),(100,50))
        self.assertEqual(project((2,0,4),C),(100,50))

    def test_inverse_pose(self):
        x,y,z=camera_coordinates((2,3,1),(1,2,1),math.pi/2)
        self.assertAlmostEqual(x,0); self.assertEqual(y,1); self.assertAlmostEqual(z,1)

    def test_near_plane_crossing(self):
        a,b=clip_segment((0,0,-2),(0,0,3),C)
        self.assertAlmostEqual(a[2],1)
        self.assertEqual(b,(0,0,3))

    def test_fractional_near_intersection_remains_projectable(self):
        a, b = clip_segment((0, 0, -.3), (0, 0, 2.1), C)
        self.assertGreaterEqual(a[2], C['near'])
        self.assertEqual(project(a, C), (50, 50))
        self.assertEqual(b, (0, 0, 2.1))
        self.assertEqual(project((0, 0, 1-2e-16), C), (50, 50))
        self.assertIsNone(project((0, 0, 1-1e-8), C))

    def test_visible_endpoints_preserved_without_false_clip(self):
        a, b = (.1, .2, 2.3), (.3, .4, 3.7)
        clipped = clip_segment(a, b, C)
        self.assertIs(clipped[0], a)
        self.assertIs(clipped[1], b)
        self.assertEqual(clipped, (a, b))

    def test_offscreen_rejection(self):
        self.assertIsNone(clip_segment((10,0,2),(20,0,2),C))
        self.assertIsNone(clip_segment((0,0,-2),(0,0,-1),C))

    def test_frame_crossing(self):
        a,b=clip_segment((-10,0,2),(10,0,2),C)
        self.assertEqual(project(a,C),(0,50))
        self.assertEqual(project(b,C),(100,50))

    def test_reverse_edge_invariance(self):
        a,b=(-5,1,.5),(3,-1,4)
        forward=clip_segment(a,b,C); backward=clip_segment(b,a,C)
        for p,q in zip(forward,reversed(backward)):
            for x,y in zip(p,q): self.assertAlmostEqual(x,y)

    def test_invalid_intrinsics(self):
        with self.assertRaises(ValueError): project((0,0,1),{**C,'fx':0})
        with self.assertRaises(ValueError): project((float('nan'),0,1),C)

if __name__=='__main__': unittest.main()
