"""Detect complete ChArUco Diamonds and bind IDs to their own corners."""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class Diamond:
    ids: tuple
    node: int
    corners: np.ndarray

    @property
    def center(self):
        return self.corners.mean(axis=0)


class DiamondDetector:
    def __init__(self, marker_nodes, ratio=2.0):
        self.nodes = {tuple(int(x) for x in key.split(',')): node for key,node in marker_nodes.items()}
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.detector = cv2.aruco.ArucoDetector(dictionary)
        # Diamond uses a 3x3 board; ratio is square side / printed marker side.
        board = cv2.aruco.CharucoBoard((3,3), float(ratio), 1.0, dictionary)
        self.diamond_detector = cv2.aruco.CharucoDetector(board)

    def detect(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None or len(ids) < 4:
            return []
        diamond_corners, diamond_ids, _, _ = self.diamond_detector.detectDiamonds(
            gray, markerCorners=corners, markerIds=ids)
        if diamond_ids is None:
            return []
        result = []
        for corners, ids in zip(diamond_corners, diamond_ids):
            key = tuple(int(x) for x in ids.flatten())
            if key in self.nodes:
                result.append(Diamond(key, self.nodes[key], np.asarray(corners,dtype=float).reshape(4,2)))
        return result


def select_diamond(diamonds, image_size, preferred_node=None):
    if not diamonds:
        return None
    preferred = [d for d in diamonds if d.node == preferred_node]
    candidates = preferred or diamonds
    center = np.asarray(image_size,dtype=float)/2
    return min(candidates, key=lambda d: float(np.linalg.norm(d.center-center)))
