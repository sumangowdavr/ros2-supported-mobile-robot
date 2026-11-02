import math
import unittest
from beni_sim.control import follow_command, guarded_command

class ControllerTests(unittest.TestCase):
    def guard(self, v=0.2, w=0.0, age=0.0, scan_age=0.0, ranges=None):
        return guarded_command(v,w,age,scan_age,
            [2.0]*361 if ranges is None else ranges, -math.pi, math.pi/180, .05, 10)
    def test_follow_forward(self):
        self.assertEqual(follow_command(0,0,0,3,0),(.3,0))
    def test_follow_stop_at_standoff(self):
        self.assertEqual(follow_command(2,0,0,3,0),(0,0))
    def test_rotate_before_forward(self):
        v,w=follow_command(0,0,0,0,3)
        self.assertEqual(v,0); self.assertGreater(w,0)
    def test_heading_wrap(self):
        v,w=follow_command(0,0,math.pi-.01,-3,-.01)
        self.assertGreater(v,0); self.assertLess(abs(w),.1)
    def test_nonfinite_target(self):
        self.assertEqual(follow_command(0,0,0,float('nan'),1),(0,0))
    def test_timeouts(self):
        self.assertEqual(self.guard(age=.6),(0,0))
        self.assertEqual(self.guard(scan_age=.6),(0,0))
    def test_missing_scan(self):
        self.assertEqual(self.guard(ranges=[]),(0,0))
    def test_nan_scan(self):
        self.assertEqual(self.guard(ranges=[math.nan]*361),(0,0))
    def test_clear_no_return(self):
        self.assertEqual(self.guard(ranges=[math.inf]*361),(.2,0))
    def test_front_obstacle(self):
        ranges=[2.0]*361; ranges[180]=.2
        self.assertEqual(self.guard(ranges=ranges),(0,0))
    def test_back_obstacle(self):
        ranges=[2.0]*361; ranges[0]=.2
        self.assertEqual(self.guard(v=-.2,ranges=ranges),(0,0))
    def test_side_obstacle_allows_forward(self):
        ranges=[2.0]*361; ranges[90]=.2
        self.assertEqual(self.guard(ranges=ranges),(.2,0))
    def test_limits(self):
        self.assertEqual(self.guard(v=5,w=-5),(.4,-1.2))
    def test_nonfinite_command(self):
        self.assertEqual(self.guard(v=math.nan),(0,0))

if __name__ == '__main__': unittest.main()
