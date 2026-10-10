"""PnP pose in the marker coordinate system."""
import math
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class Pose:
    rvec: np.ndarray
    tvec: np.ndarray
    camera_position: np.ndarray
    yaw_deg: float
    reprojection_error_px: float


class PoseEstimator:
    def __init__(self, calibration, diamond_side, max_error=5.0):
        self.calibration, self.max_error = calibration, max_error
        half = diamond_side/2
        # OpenCV Diamond order: top-left, top-right, bottom-right, bottom-left.
        # X points right and Y points down on the upright printed marker.
        self.points = np.array([[-half,-half,0],[half,-half,0],
                                [half,half,0],[-half,half,0]],dtype=float)

    def estimate(self, diamond, image_size):
        points,k,d = self.calibration.pinhole_points(diamond.corners,image_size)
        ok,rvec,tvec = cv2.solvePnP(self.points,points,k,d,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or not np.isfinite(tvec).all() or tvec[2,0] <= 0:
            return None
        projected = self.calibration.project(self.points,rvec,tvec,image_size)
        error = float(np.sqrt(np.mean(np.sum((projected-diamond.corners)**2,axis=1))))
        if not np.isfinite(error) or error > self.max_error:
            return None
        rotation,_ = cv2.Rodrigues(rvec)
        position = (-rotation.T@tvec).flatten()
        yaw = math.degrees(math.atan2(rotation[1,0],rotation[0,0]))
        return Pose(rvec,tvec,position,yaw,error)

    def draw(self, image, pose):
        size = (image.shape[1],image.shape[0])
        side = float(self.points[1,0]-self.points[0,0])/2
        points = np.array([[0,0,0],[side,0,0],[0,side,0],[0,0,-side]],dtype=float)
        pixels = self.calibration.project(points,pose.rvec,pose.tvec,size).round().astype(int)
        for point,color in zip(pixels[1:],[(0,0,255),(0,255,0),(255,0,0)]):
            cv2.line(image,tuple(pixels[0]),tuple(point),color,2)
