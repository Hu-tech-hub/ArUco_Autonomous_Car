"""Capture ChArUco calibration observations and save K/D with image size."""
import argparse
import json
from pathlib import Path
import cv2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera',type=int,default=0)
    parser.add_argument('--square',type=float,required=True,help='Printed square side in metres')
    parser.add_argument('--marker',type=float,required=True,help='Printed marker side in metres')
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'data/calibration.json')
    args = parser.parse_args()
    if not 0 < args.marker < args.square:
        parser.error('Require 0 < marker < square')
    board = cv2.aruco.CharucoBoard((5,5),args.square,args.marker,
                                  cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250))
    detector = cv2.aruco.CharucoDetector(board)
    cap = cv2.VideoCapture(args.camera)
    views,ids_list = [],[]
    image_size = None
    try:
        if not cap.isOpened():
            raise RuntimeError('Camera unavailable')
        print('SPACE: capture | ESC: calculate and save')
        while True:
            ok,frame = cap.read()
            if not ok:
                raise RuntimeError('Camera read failed')
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            size = (gray.shape[1],gray.shape[0])
            if image_size is not None and size != image_size:
                raise RuntimeError('Camera resolution changed')
            corners,ids,_,_ = detector.detectBoard(gray)
            if ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(frame,corners,ids)
            cv2.imshow('Calibration',frame)
            key = cv2.waitKey(1)&255
            if key == 27:
                break
            if key == 32 and ids is not None and len(ids) >= 6:
                views.append(corners.copy()); ids_list.append(ids.copy()); image_size=size
                print('Captured',len(views))
        if len(views) < 10:
            raise RuntimeError('Capture at least ten varied views')
        rms,k,d,_,_ = cv2.aruco.calibrateCameraCharuco(views,ids_list,board,image_size,None,None)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps({'model':'pinhole','image_size':list(image_size),
            'K':k.tolist(),'D':d.tolist(),'rms_reprojection_error_px':rms},indent=2),encoding='utf-8')
        print('Saved',args.output,'RMS reprojection error (px):',rms)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
