"""Capture ChArUco calibration views and write pinhole K/D JSON."""
import argparse
import json
from pathlib import Path

def main():
    import cv2
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--square', type=float, required=True, help='Board square side in metres')
    parser.add_argument('--marker', type=float, required=True, help='Printed marker side in metres')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1]/'data/calibration.json')
    args = parser.parse_args()
    if not 0 < args.marker < args.square:
        parser.error('Require 0 < marker < square')
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    board = aruco.CharucoBoard((5,5), args.square, args.marker, dictionary) if hasattr(aruco,'CharucoBoard') else aruco.CharucoBoard_create(5,5,args.square,args.marker,dictionary)
    detector = aruco.ArucoDetector(dictionary) if hasattr(aruco,'ArucoDetector') else None
    cap = cv2.VideoCapture(args.camera)
    views, ids_list = [], []
    image_size = None
    try:
        if not cap.isOpened():
            raise RuntimeError('Camera unavailable')
        print('SPACE: capture view | ESC: calculate and save')
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('Camera read failed')
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            size = (gray.shape[1],gray.shape[0])
            if image_size is not None and size != image_size:
                raise RuntimeError('Camera resolution changed')
            corners, ids, _ = detector.detectMarkers(gray) if detector else aruco.detectMarkers(gray,dictionary)
            count, cc, ci = (0,None,None)
            if ids is not None:
                count, cc, ci = aruco.interpolateCornersCharuco(corners,ids,gray,board)
                if cc is not None:
                    aruco.drawDetectedCornersCharuco(frame,cc,ci)
            cv2.imshow('Calibration',frame)
            key = cv2.waitKey(1)&255
            if key == 27:
                break
            if key == 32 and count >= 6:
                views.append(cc.copy()); ids_list.append(ci.copy()); image_size=size
                print('Captured',len(views))
        if len(views) < 10:
            raise RuntimeError('Capture at least 10 varied board views')
        rms, matrix, distortion, _, _ = aruco.calibrateCameraCharuco(views,ids_list,board,image_size,None,None)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps({'K':matrix.tolist(),'D':distortion.tolist(),'image_size':list(image_size),'rms_reprojection_error_px':rms},indent=2),encoding='utf-8')
        print('Saved',args.output,'RMS(px):',rms,'resolution:',image_size)
        print('Set calibration_resolution in src/config.json to',image_size)
    finally:
        cap.release()
        cv2.destroyAllWindows()
if __name__ == '__main__':
    main()
