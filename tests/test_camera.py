import math
import unittest
from dsiq_native.camera import FreeCamera

class CameraTests(unittest.TestCase):
    def test_direction_and_target_are_metric(self):
        cam=FreeCamera.look_at((3.,0.,1.),(0.,0.,1.))
        eye,target=cam.eye_target(sight_distance_m=2.)
        self.assertEqual(eye,(3.,0.,1.))
        self.assertAlmostEqual(target[0],1.,places=9)
        self.assertAlmostEqual(target[2],1.,places=9)

    def test_wasd_qe_move_in_local_free_camera_coordinates(self):
        cam=FreeCamera(position_m=(0.,0.,1.),yaw_rad=0.,pitch_rad=0.,speed_m_s=2.)
        cam.move("w",.5)
        self.assertAlmostEqual(cam.position_m[0],1.)
        cam.move("d",.5)
        self.assertAlmostEqual(cam.position_m[1],1.)
        cam.move("e",.5)
        self.assertAlmostEqual(cam.position_m[2],2.)
        cam.move("s",.25,sprint=True)
        self.assertAlmostEqual(cam.position_m[0],-.5)
        cam.move("q",.5)
        self.assertAlmostEqual(cam.position_m[2],1.)

    def test_mouse_look_orientation_and_pitch_limits(self):
        cam=FreeCamera.look_at((2.,-3.,2.),(0.,0.,0.))
        before=cam.forward()
        cam.mouse_look(50.,12.)
        self.assertNotEqual(cam.forward(),before)
        cam.mouse_look(0.,-10000.)
        self.assertLessEqual(cam.pitch_rad,1.45)
        cam.mouse_look(0.,10000.)
        self.assertGreaterEqual(cam.pitch_rad,-1.45)

    def test_invalid_camera_commands_fail(self):
        with self.assertRaises(ValueError):
            FreeCamera.look_at((0.,0.,0.),(0.,0.,0.))
        with self.assertRaises(ValueError):
            FreeCamera(position_m=(float("nan"),0.,0.)).validate()
        with self.assertRaises(ValueError):
            FreeCamera().move("panic")
        with self.assertRaises(ValueError):
            FreeCamera().move("w",seconds=0)
        with self.assertRaises(ValueError):
            FreeCamera().mouse_look(float("inf"),0)

if __name__=="__main__":
    unittest.main()
