"""Pinhole and fisheye calibration are kept distinct."""
import json
from dataclasses import dataclass
from pathlib import Path
import cv2
import numpy as np


@dataclass
class Calibration:
    matrix: np.ndarray
    distortion: np.ndarray
    image_size: tuple
    model: str = 'pinhole'

    @classmethod
    def load(cls, path, image_size=None, model=None):
        path = Path(path)
        if path.suffix == '.npz':
            with np.load(path, allow_pickle=False) as data:
                if 'k_array' in data:
                    k, d = data['k_array'], data['d_array']
                    size = tuple(int(x) for x in data['dim_array'])
                    inferred = 'fisheye'
                else:
                    k = data['K']
                    d = data['D'] if 'D' in data else data['dists']
                    size = image_size
                    inferred = 'pinhole'
        else:
            data = json.loads(path.read_text(encoding='utf-8'))
            k, d = data['K'], data['D']
            size = data.get('image_size') or image_size
            inferred = data.get('model', 'pinhole')
        if size is None:
            raise ValueError('Set calibration_resolution to the calibration image width and height')
        result = cls(np.asarray(k, dtype=float), np.asarray(d, dtype=float), tuple(size), model or inferred)
        if result.model not in ('pinhole', 'fisheye'):
            raise ValueError('Unsupported calibration model')
        if result.matrix.shape != (3, 3) or min(result.image_size) <= 0:
            raise ValueError('Invalid calibration matrix or resolution')
        if not np.isfinite(result.matrix).all() or not np.isfinite(result.distortion).all():
            raise ValueError('Calibration must contain finite values')
        if result.model == 'fisheye' and result.distortion.size != 4:
            raise ValueError('Fisheye requires four coefficients')
        return result

    def matrix_for(self, size):
        k = self.matrix.copy()
        k[0] *= size[0] / self.image_size[0]
        k[1] *= size[1] / self.image_size[1]
        return k

    def pinhole_points(self, points, size):
        k = self.matrix_for(size)
        if self.model == 'fisheye':
            points = cv2.fisheye.undistortPoints(np.asarray(points, dtype=float).reshape(-1,1,2), k,
                                               self.distortion.reshape(4,1), P=k).reshape(-1,2)
            return points, k, np.zeros((5,1))
        return np.asarray(points, dtype=float).reshape(-1,2), k, self.distortion

    def project(self, points, rvec, tvec, size):
        k = self.matrix_for(size)
        if self.model == 'fisheye':
            return cv2.fisheye.projectPoints(np.asarray(points, dtype=float).reshape(-1,1,3),
                                            rvec, tvec, k, self.distortion.reshape(4,1))[0].reshape(-1,2)
        return cv2.projectPoints(points, rvec, tvec, k, self.distortion)[0].reshape(-1,2)
