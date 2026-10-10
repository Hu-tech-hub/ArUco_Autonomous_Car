"""CSV observations and commands for replay and measurement."""
import csv
from pathlib import Path


class Telemetry:
    def __init__(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True,exist_ok=True)
        self.file = path.open('w',newline='',encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['captured_at','node','dx_px','dy_px','yaw_deg','reprojection_error_px',
                              'processing_ms','state','command','x','y','angular'])

    def write(self, observation, pose, action, elapsed_ms):
        row = [observation.captured_at,observation.node,observation.dx,observation.dy,observation.yaw_deg] if observation else ['']*5
        self.writer.writerow(row+[pose.reprojection_error_px if pose else '',elapsed_ms,
                                  action.state,action.kind,action.x,action.y,action.angular])
        self.file.flush()

    def close(self):
        self.file.close()
