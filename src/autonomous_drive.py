"""Run the camera -> Diamond -> pose -> BFS -> feedback-control pipeline."""
import argparse
from pathlib import Path
import time
import cv2
from .calibration import Calibration
from .camera import LatestCamera
from .configuration import load_config
from .control import Navigator, MotorDriver, Observation
from .detection import DiamondDetector, select_diamond
from .pose import PoseEstimator
from .telemetry import Telemetry


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('config.json'))
    parser.add_argument('--drive',action='store_true')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--log',type=Path,default=Path('data/logs')/f'run_{time.time_ns()}.csv')
    args = parser.parse_args()
    cfg = load_config(args.config)
    calibration = Calibration.load(cfg['calibration'],cfg['calibration_resolution'])
    detector = DiamondDetector(cfg['marker_nodes'],cfg['square_marker_ratio'])
    estimator = PoseEstimator(calibration,cfg['diamond_side_m'],cfg['max_reprojection_error_px'])
    navigator = Navigator(cfg)
    motors = MotorDriver(cfg,args.drive)
    camera = LatestCamera(cfg['camera'],cfg['resolution'])
    logger = None
    sequence,after_time = 0,0.0
    previous_state = None
    try:
        logger = Telemetry(args.log)
        camera.start()
        while True:
            packet = camera.read(sequence,after_time,cfg['camera_timeout_s'])
            pose,observation = None,None
            started = time.monotonic()
            if packet is not None:
                sequence = packet.sequence
                image = packet.image
                size = (image.shape[1],image.shape[0])
                diamonds = detector.detect(image)
                diamond = select_diamond(diamonds,size,navigator.next_node)
                if diamond is not None:
                    pose = estimator.estimate(diamond,size)
                    if pose is not None:
                        offset = diamond.center-[size[0]/2,size[1]/2]
                        yaw = cfg['yaw_sign']*pose.yaw_deg+cfg['yaw_offset_deg']
                        observation = Observation(diamond.node,float(offset[0]),float(offset[1]),yaw,packet.captured_at)
                        if not args.headless:
                            estimator.draw(image,pose)
                            cv2.polylines(image,[diamond.corners.round().astype('int32')],True,(0,165,255),2)
            now = time.monotonic()
            action = navigator.step(observation,now)
            processing_ms = (now-started)*1000
            logger.write(observation,pose,action,processing_ms)
            if action.state != previous_state:
                print(action.state,'route=',navigator.route,'next=',navigator.next_node,flush=True)
                previous_state = action.state
            if packet is not None and not args.headless:
                label = f'{action.state} | node {observation.node if observation else "-"}'
                cv2.putText(image,label,(10,30),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,165,255),2)
                cv2.imshow('Ceiling Marker Cart',image)
            if not args.headless and cv2.waitKey(1)&255 in (27,ord('q')):
                break
            motors.execute(action)
            # Skip frames acquired while motors were executing a pulse.
            if action.kind != 'stop':
                after_time = time.monotonic()
            if action.state in ('ARRIVED','TRANSITION_TIMEOUT'):
                break
    except KeyboardInterrupt:
        print('Stopped by user')
    finally:
        motors.stop()
        camera.close()
        if logger is not None:
            logger.close()
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
