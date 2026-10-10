import json
from pathlib import Path
import unittest
import tempfile
import time
from unittest.mock import patch
import cv2
import numpy as np
from src.calibration import Calibration
from src.control import Navigator, Observation, MotorDriver, Action, PID
from src.detection import Diamond, DiamondDetector, select_diamond
from src.planning import shortest_path, heading, angle_error
from src.pose import PoseEstimator
from src.camera import LatestCamera, Frame

ROOT = Path(__file__).resolve().parents[1]


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((ROOT/'src/config.json').read_text(encoding='utf-8'))

    def test_every_grid_pair_has_shortest_adjacent_route(self):
        for start in range(1,17):
            for goal in range(1,17):
                route = shortest_path(start,goal)
                r1,c1 = divmod(start-1,4); r2,c2 = divmod(goal-1,4)
                self.assertEqual(len(route)-1,abs(r2-r1)+abs(c2-c1))
                for a,b in zip(route,route[1:]):
                    self.assertIn(heading(a,b),(0,90,180,270))
        self.assertEqual(angle_error(0,359),1)
        self.assertEqual(angle_error(359,0),-1)

    def test_selected_marker_keeps_own_center(self):
        a = Diamond((1,2,3,4),1,np.array([[310,230],[330,230],[330,250],[310,250]],float))
        b = Diamond((5,6,7,8),2,a.corners+100)
        self.assertIs(select_diamond([a,b],(640,480)),a)
        self.assertIs(select_diamond([a,b],(640,480),2),b)

    def test_generated_diamond_is_detected(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        board = cv2.aruco.CharucoBoard((3,3),.04,.02,dictionary,np.array([1,2,3,4],dtype=np.int32))
        image = board.generateImage((600,600),marginSize=30)
        diamonds = DiamondDetector({'1,2,3,4':1}).detect(image)
        self.assertEqual(len(diamonds),1)
        self.assertEqual(diamonds[0].node,1)

    def test_pinhole_and_fisheye_pose_recovery(self):
        k = np.array([[700.,0,320],[0,700,240],[0,0,1]])
        for model,d in [('pinhole',np.zeros(5)),('fisheye',np.array([.01,-.005,0,0]))]:
            calibration = Calibration(k,d,(640,480),model)
            estimator = PoseEstimator(calibration,.05)
            rvec = np.array([.1,-.15,.4])
            tvec = np.array([.01,.005,.5])
            corners = calibration.project(estimator.points,rvec,tvec,(640,480))
            pose = estimator.estimate(Diamond((1,2,3,4),1,corners),(640,480))
            self.assertIsNotNone(pose)
            np.testing.assert_allclose(pose.tvec.flatten(),tvec,atol=1e-5)
            self.assertLess(pose.reprojection_error_px,.01)

    def test_supplied_calibration_formats(self):
        original = Calibration.load(ROOT/'data/calibration_param.npz')
        self.assertEqual(original.model,'fisheye')
        self.assertEqual(original.image_size,(640,480))
        with self.assertRaises(ValueError):
            Calibration.load(ROOT/'data/calib_1109.npz')
        alternate = Calibration.load(ROOT/'data/calib_1109.npz',(640,480))
        self.assertEqual(alternate.model,'pinhole')
        scaled = original.matrix_for((320,240))
        np.testing.assert_allclose(scaled[0],original.matrix[0]/2)

    def test_latest_frame_filters_sequence_and_capture_time(self):
        camera = LatestCamera(0,(640,480))
        camera.latest = Frame(4,10,np.zeros((480,640,3),dtype=np.uint8))
        self.assertIsNone(camera.read(after_sequence=4,timeout=0))
        self.assertIsNone(camera.read(after_time=11,timeout=0))
        packet = camera.read(after_sequence=3,after_time=9,timeout=0)
        self.assertEqual(packet.sequence,4)
        packet.image[0,0] = 255
        self.assertEqual(int(camera.latest.image[0,0,0]),0)

    def test_center_rotate_move_and_arrive(self):
        nav = Navigator(self.cfg)
        def observe(node,dx=0,dy=0,yaw=0,now=1):
            return nav.step(Observation(node,dx,dy,yaw,now),now)
        self.assertEqual(observe(1,dx=0,dy=30).state,'CENTERING')
        self.assertEqual(observe(1,yaw=40,now=1.1).state,'ROTATING')
        self.assertEqual(observe(1,now=1.2).state,'SETTLING')
        self.assertEqual(observe(1,now=1.3).state,'MOVING')
        self.assertEqual(nav.next_node,5)
        self.assertEqual(observe(1,now=1.4).state,'MOVING')
        self.assertEqual(nav.step(None,1.5).kind,'stop')
        self.assertEqual(observe(5,now=1.6).state,'SETTLING')
        self.assertEqual(nav.current_node,5)
        self.assertEqual(observe(16,now=1.7).state,'SETTLING')
        self.assertEqual(observe(16,now=1.8).state,'ARRIVED')

    def test_missing_stale_and_transition_timeout_stop(self):
        nav = Navigator(self.cfg)
        self.assertEqual(nav.step(None,1).kind,'stop')
        self.assertEqual(nav.step(Observation(1,0,0,0,0),1).kind,'stop')
        nav.step(Observation(1,0,0,0,1),1)
        nav.step(Observation(1,0,0,0,1.1),1.1)
        self.assertEqual(nav.step(None,20).state,'TRANSITION_TIMEOUT')

    def test_pid_output_magnitude_and_driver_exception_stop(self):
        pid = PID([.1,0,0],2)
        self.assertGreater(pid.update(10,1),pid.update(1,1.1))
        class BrokenChassis:
            stopped = False
            def set_velocity(self,speed,angle,angular):
                self.stopped = speed == 0 and angular == 0
            def translation(self,x,y):
                raise RuntimeError('Mock device failure')
        driver = MotorDriver(self.cfg)
        driver.drive=True; driver.chassis=BrokenChassis()
        with self.assertRaises(RuntimeError):
            driver.execute(Action('CENTERING','translation',1,1,duration=.1))
        self.assertTrue(driver.chassis.stopped)

    def test_full_pipeline_destination_stops_and_writes_log(self):
        from src import autonomous_drive as app
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        board = cv2.aruco.CharucoBoard((3,3),.04,.02,dictionary,np.array([61,62,63,64],dtype=np.int32))
        gray = np.full((480,640),255,np.uint8)
        gray[90:390,170:470] = board.generateImage((300,300),marginSize=15)
        image = cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
        class FakeCamera:
            def __init__(self,*args): self.sequence=0
            def start(self): return self
            def read(self,*args):
                self.sequence+=1
                if self.sequence > 5:
                    raise RuntimeError('Pipeline failed to arrive')
                return Frame(self.sequence,time.monotonic(),image.copy())
            def close(self): pass
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            calibration = {'model':'pinhole','image_size':[640,480],
                           'K':[[500,0,320],[0,500,240],[0,0,1]],'D':[0,0,0,0,0]}
            (root/'calibration.json').write_text(json.dumps(calibration))
            cfg=self.cfg.copy();cfg['calibration']='calibration.json'
            (root/'config.json').write_text(json.dumps(cfg))
            argv=['cart','--headless','--config',str(root/'config.json'),'--log',str(root/'log.csv')]
            with patch.object(app,'LatestCamera',FakeCamera),patch('sys.argv',argv):
                app.main()
            log=(root/'log.csv').read_text()
            self.assertIn('ARRIVED,stop',log)


if __name__ == '__main__':
    unittest.main()
