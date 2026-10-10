"""Load validated settings and resolve repository-relative paths."""
import json
from pathlib import Path


def load_config(path):
    path = Path(path).resolve()
    cfg = json.loads(path.read_text(encoding='utf-8'))
    cfg['calibration'] = str((path.parent/cfg['calibration']).resolve())
    if not 1 <= cfg['goal'] <= cfg['grid_size']**2:
        raise ValueError('Goal outside grid')
    for key in ('resolution','calibration_resolution'):
        if cfg[key] is not None and (len(cfg[key]) != 2 or min(cfg[key]) <= 0):
            raise ValueError('Invalid '+key)
    for key in ('diamond_side_m','square_marker_ratio','pulse_seconds','max_frame_age_s',
                'center_tolerance_px','angle_tolerance_deg','translation_limit','rotation_limit',
                'transition_timeout_s'):
        if cfg[key] <= 0:
            raise ValueError('Require positive '+key)
    if cfg['square_marker_ratio'] <= 1 or cfg['stable_frames'] < 1:
        raise ValueError('Invalid marker ratio or settling count')
    for key in ('x_sign','y_sign','rotation_sign','yaw_sign'):
        if cfg[key] not in (-1,1):
            raise ValueError('Require coordinate sign +1 or -1')
    for node in cfg['marker_nodes'].values():
        if not isinstance(node,int) or not 1 <= node <= cfg['grid_size']**2:
            raise ValueError('Invalid marker node')
    return cfg
