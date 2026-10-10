"""Ceiling-marker cart controller."""
import argparse
import json
import math
import sys
import time
from collections import deque
from pathlib import Path


def shortest_path(start, goal, grid=4):
    if not (1 <= start <= grid * grid and 1 <= goal <= grid * grid):
        raise ValueError("Node must be within grid")
    pending = deque([(start, [start])])
    seen = {start}
    while pending:
        node, path = pending.popleft()
        if node == goal:
            return path
        row, col = divmod(node - 1, grid)
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            r, c = row + dr, col + dc
            nxt = r * grid + c + 1
            if 0 <= r < grid and 0 <= c < grid and nxt not in seen:
                seen.add(nxt)
                pending.append((nxt, path + [nxt]))
    raise ValueError("No route")


def heading(current, following, grid=4):
    r1, c1 = divmod(current - 1, grid)
    r2, c2 = divmod(following - 1, grid)
    return {(-1, 0): 180, (1, 0): 0, (0, -1): 270, (0, 1): 90}[(r2-r1, c2-c1)]


def angle_error(target, actual):
    return (target - actual + 180) % 360 - 180


class PID:
    def __init__(self, p, i, d, limit):
        self.p, self.i, self.d, self.limit = p, i, d, limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous = None
        self.clock = None

    def update(self, error):
        now = time.monotonic()
        dt = max(0.001, now - self.clock) if self.clock else 0.1
        self.integral = max(-100, min(100, self.integral + error * dt))
        derivative = (error-self.previous)/dt if self.previous is not None else 0
        self.previous, self.clock = error, now
        return max(-self.limit, min(self.limit,
            self.p*error + self.i*self.integral + self.d*derivative))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--drive', action='store_true', help='Enable physical motors')
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('config.json'))
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text(encoding='utf-8'))
    import cv2
    import numpy as np
    calibration = args.config.parent / cfg['calibration']
    data = json.loads(calibration.read_text(encoding='utf-8'))
    matrix, distortion = np.asarray(data['K'], dtype=float), np.asarray(data['D'], dtype=float)
    width, height = cfg['resolution']
    calibration_size = cfg['calibration_resolution']
    if calibration_size is None:
        raise ValueError('Set calibration_resolution to the actual calibration image size')
    matrix = matrix.copy()
    matrix[0, :] *= width / calibration_size[0]
    matrix[1, :] *= height / calibration_size[1]
    chassis = None
    if args.drive:
        sys.path.insert(0, cfg['sdk_path'])
        import HiwonderSDK.mecanum as mecanum
        chassis = mecanum.MecanumChassis()
    def stop():
        if chassis is not None:
            chassis.set_velocity(0, 90, 0)
    def pulse(kind, value):
        print(kind, value)
        if chassis is not None:
            if kind == 'rotate':
                chassis.set_velocity(60, 90, math.copysign(cfg['rotation_speed'], value))
            elif kind == 'center':
                chassis.translation(*value)
            else:
                chassis.set_velocity(cfg['forward_speed'], 90, 0)
            try:
                time.sleep(cfg['pulse_seconds'])
            finally:
                stop()
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    detector = aruco.ArucoDetector(dictionary) if hasattr(aruco, 'ArucoDetector') else None
    side = cfg['diamond_side_m']
    points = np.array([[-side/2,-side/2,0], [side/2,-side/2,0],
                       [side/2,side/2,0], [-side/2,side/2,0]], dtype=np.float32)
    rotation_pid = PID(.3, .01, .01, 25)
    x_pid, y_pid = PID(.3, .01, .01, 25), PID(.3, .01, .01, 25)
    departing = None
    last_node = None
    cap = cv2.VideoCapture(cfg['camera'])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    try:
        if not cap.isOpened():
            raise RuntimeError('Camera unavailable')
        while True:
            ok, frame = cap.read()
            if not ok:
                stop()
                raise RuntimeError('Camera frame unavailable')
            # Process a newly acquired image after each movement pulse.
            frame = cv2.resize(frame, (width, height))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray) if detector else aruco.detectMarkers(gray, dictionary)
            observed = None
            if ids is not None:
                diamonds, diamond_ids = aruco.detectCharucoDiamond(gray, corners, ids, cfg['square_marker_ratio'])
                if diamonds is not None and len(diamonds):
                    center = np.array([width/2, height/2])
                    chosen = min(range(len(diamonds)), key=lambda j: np.linalg.norm(diamonds[j].reshape(4,2).mean(axis=0)-center))
                    image_points = diamonds[chosen].reshape(4,2)
                    key = ','.join(str(int(i)) for i in diamond_ids[chosen].flatten())
                    node = cfg['marker_nodes'].get(key)
                    success, rvec, tvec = cv2.solvePnP(points, image_points, matrix, distortion)
                    if success and node is not None:
                        rotation, _ = cv2.Rodrigues(rvec)
                        yaw = math.degrees(math.atan2(rotation[1,0], rotation[0,0]))
                        offset = image_points.mean(axis=0)-center
                        observed = (node, yaw, offset)
                        cv2.drawFrameAxes(frame, matrix, distortion, rvec, tvec, side/2)
                        cv2.putText(frame, f'Node {node} | yaw {yaw:.1f}', (10,30), cv2.FONT_HERSHEY_SIMPLEX, .65, (0,165,255), 2)
            cv2.imshow('ArUco Cart', frame)
            if cv2.waitKey(1) & 255 in (27, ord('q')):
                break
            if observed is None:
                stop()
                departing = None
                continue
            node, yaw, offset = observed
            if node != last_node:
                print('route', shortest_path(node, cfg['goal']))
                rotation_pid.reset(); x_pid.reset(); y_pid.reset()
                last_node = node
            if departing == node:
                pulse('forward', None)
                continue
            departing = None
            dx, dy = offset
            if abs(dx) >= cfg['center_tolerance_px'] or abs(dy) >= cfg['center_tolerance_px']:
                vx, vy = x_pid.update(float(dx)), y_pid.update(float(dy))
                pulse('center', (cfg['x_sign']*vx, cfg['y_sign']*vy))
                continue
            if node == cfg['goal']:
                stop()
                print('ARRIVED: centered at destination')
                break
            route = shortest_path(node, cfg['goal'])
            error = angle_error(heading(node, route[1]), yaw)
            if abs(error) > cfg['angle_tolerance_deg']:
                pulse('rotate', cfg['rotation_sign']*rotation_pid.update(error))
            else:
                departing = node
                pulse('forward', None)
    finally:
        stop()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
