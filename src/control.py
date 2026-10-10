"""Observation-driven navigation and bounded PID motor commands."""
import math
import time
from dataclasses import dataclass
from .planning import shortest_path, heading, angle_error


class PID:
    def __init__(self, gains, limit, integral_limit=100):
        self.p,self.i,self.d = gains
        self.limit,self.integral_limit = limit,integral_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous = None
        self.last_time = None

    def update(self, error, now):
        dt = max(.001,now-self.last_time) if self.last_time is not None else .1
        derivative = (error-self.previous)/dt if self.previous is not None else 0.0
        candidate = max(-self.integral_limit,min(self.integral_limit,self.integral+error*dt))
        raw = self.p*error+self.i*candidate+self.d*derivative
        output = max(-self.limit,min(self.limit,raw))
        if abs(raw) <= self.limit or raw*error < 0:
            self.integral = candidate
        self.previous,self.last_time = error,now
        return output


@dataclass
class Observation:
    node: int
    dx: float
    dy: float
    yaw_deg: float
    captured_at: float


@dataclass
class Action:
    state: str
    kind: str = 'stop'
    x: float = 0.0
    y: float = 0.0
    angular: float = 0.0
    duration: float = 0.0


class Navigator:
    def __init__(self, cfg):
        self.cfg = cfg
        self.current_node = None
        self.next_node = None
        self.route = []
        self.departed_at = None
        self.arrived = False
        self.stable = 0
        self.previous_state = None
        self.x_pid = PID(cfg['position_gains'],cfg['translation_limit'])
        self.y_pid = PID(cfg['position_gains'],cfg['translation_limit'])
        self.rotate_pid = PID(cfg['rotation_gains'],cfg['rotation_limit'])

    def _reset_pids(self):
        for pid in (self.x_pid,self.y_pid,self.rotate_pid):
            pid.reset()

    def _action(self, state, kind='stop', **kwargs):
        if state != self.previous_state:
            self._reset_pids()
            self.previous_state = state
        return Action(state,kind,**kwargs)

    def step(self, observation, now=None):
        now = time.monotonic() if now is None else now
        if self.arrived:
            return self._action('ARRIVED')
        if self.departed_at is not None and now-self.departed_at > self.cfg['transition_timeout_s']:
            return self._action('TRANSITION_TIMEOUT')
        if observation is None or now-observation.captured_at > self.cfg['max_frame_age_s']:
            self.stable = 0
            return self._action('NO_OBSERVATION')
        node = observation.node
        if self.next_node is not None:
            if node == self.current_node:
                return self._action('MOVING','forward',duration=self.cfg['pulse_seconds'])
            # Expected target or another known marker: reacquire and replan.
            self.next_node = None
            self.departed_at = None
        if node != self.current_node:
            self.current_node = node
            self.route = shortest_path(node,self.cfg['goal'],self.cfg['grid_size'])
            self.stable = 0
            self._reset_pids()
        if abs(observation.dx) > self.cfg['center_tolerance_px'] or abs(observation.dy) > self.cfg['center_tolerance_px']:
            self.stable = 0
            self._action('CENTERING')
            return Action('CENTERING','translation',
                          self.cfg['x_sign']*self.x_pid.update(observation.dx,now),
                          self.cfg['y_sign']*self.y_pid.update(observation.dy,now),
                          duration=self.cfg['pulse_seconds'])
        if node != self.cfg['goal']:
            target = heading(node,self.route[1],self.cfg['grid_size'])
            error = angle_error(target,observation.yaw_deg)
            if abs(error) > self.cfg['angle_tolerance_deg']:
                self.stable = 0
                self._action('ROTATING')
                output = self.cfg['rotation_sign']*self.rotate_pid.update(error,now)
                return Action('ROTATING','rotation',angular=output,duration=self.cfg['pulse_seconds'])
        self.stable += 1
        if self.stable < self.cfg['stable_frames']:
            return self._action('SETTLING')
        self.stable = 0
        if node == self.cfg['goal']:
            self.arrived = True
            return self._action('ARRIVED')
        self.next_node = self.route[1]
        self.departed_at = now
        return self._action('MOVING','forward',duration=self.cfg['pulse_seconds'])


class MotorDriver:
    def __init__(self, cfg, drive=False):
        self.cfg,self.drive,self.chassis = cfg,drive,None
        self.board = None
        if drive:
            import sys
            sys.path.insert(0,cfg['sdk_path'])
            from HiwonderSDK.mecanum import MecanumChassis
            from HiwonderSDK import Board
            self.board = Board
            self.chassis = MecanumChassis()
            self.stop()
            if cfg.get('servo_initial_pulses'):
                from HiwonderSDK import Board
                for index,pulse in cfg['servo_initial_pulses'].items():
                    Board.setPWMServoPulse(int(index),pulse,1000)

    def stop(self):
        if self.board is not None and hasattr(self.board,'stopAllMotors'):
            self.board.stopAllMotors()
        elif self.chassis is not None:
            self.chassis.set_velocity(0,90,0)

    def close(self):
        if self.board is not None and hasattr(self.board,'close'):
            self.board.close()

    def execute(self, action):
        if action.kind == 'stop':
            self.stop()
            return
        if not self.drive:
            return
        try:
            if action.kind == 'translation':
                self.chassis.translation(action.x,action.y)
            elif action.kind == 'rotation':
                self.chassis.set_velocity(0,90,action.angular)
            elif action.kind == 'forward':
                self.chassis.set_velocity(self.cfg['forward_speed'],90,0)
            else:
                raise ValueError('Unknown motor action')
            time.sleep(min(action.duration,self.cfg['pulse_seconds']))
        finally:
            self.stop()
