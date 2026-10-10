#!/usr/bin/env python
import cv2
import numpy as np
import sys
import math

#미완성본 Centering_2.py 사용할것

#가상환경에서 python3.9, OpenCV 4.5.1 버전에서만 실행 가능 
# 다음과 같이 터미널에 입력
# 1. source myvenv/Scripts/activate (이렇게 하면 가상환경이 실행됨)
# 2. cd RaspberryPi_Car
# 3. python Centering.py 

#!! VideoCapture 객체 release 안시키면 다음 코드 실행에 문제가 생길 수 있음!!

# 카메라 파라미터를 얻은 직후 처음에 drawaxis에서 보드가 보이지 않으면 오류 출력한다. (제일 처음에 회전 벡터와 이동벡터가 none 이기 때문)
# 그냥 이동 벡터에다가 상수 연산한다고 좌표가 이동되는 것은 아니다.
# draw axis 말고 다른 함수 써서 그냥 9.875, 9.875 를 중심으로 축을 그리자(그림 그리는 함수)
# x, y, z는 각 상수값을 더해서 중심을 맞추자
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard_create(5, 5, 3.875, 1.875, aruco_dict)

def read_chessboards(frames):
    """
    Charuco base pose estimation.
    """ 
    all_corners = []
    all_ids = []

    for frame in frames:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(corners) > 0:
            ret, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if ret > 0:
                all_corners.append(c_corners)
                all_ids.append(c_ids)
        else:
            print('Failed!')

    imsize = gray.shape
    return all_corners, all_ids, imsize

def calculate_camera_position_and_orientation(rvec, tvec):
    """
    rvec: 회전 벡터 (3x1 numpy 배열)
    tvec: 변환 벡터 (3x1 numpy 배열)
    """
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    R_inv = np.linalg.inv(rotation_matrix)

    cam_pos = -np.dot(R_inv, tvec)
    X, Y, Z = cam_pos.flatten()

    pitch = math.atan2(-rotation_matrix[2, 0], math.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2))
    yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])

    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)

    print(f"Camera Position (X, Y, Z): {X:.2f}, {Y:.2f}, {Z:.2f}")
    print(f"Pan (Yaw): {yaw_deg:.2f} degrees")

def capture_camera(dev_num=0, num=1, mirror=False, size=None):
    frames = []
    cap = cv2.VideoCapture(dev_num)

    while True:
        ret, frame = cap.read()

        if mirror is True:
            frame = cv2.flip(frame, 1)

        if size is not None and len(size) == 2:
            frame = cv2.resize(frame, size)

        cv2.imshow('Java', frame)

        k = cv2.waitKey(1)
        if k == 27:  # Esc
            cap.release()
            break
        elif k == 10 or k == 32:  # Enter or Space
            frames.append(frame)
            print('Frame captured!')
            if len(frames) == num:
                #cap.release()
                #cv2.destroyAllWindows()
                break

    return frames


def draw_axis(frame, camera_matrix, dist_coeff, board, verbose=True, prev_rvec=None, prev_tvec=None):
    """
    회전 벡터와 변환 벡터를 연속적으로 추정합니다.
    """
    corners, ids, rejected_points = cv2.aruco.detectMarkers(frame, aruco_dict)
    
    if corners is None or ids is None:
        return frame, prev_rvec, prev_tvec 

    if len(corners) != len(ids) or len(corners) == 0:
        return frame, prev_rvec, prev_tvec  

    try:
        ret, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, frame, board)

        if prev_rvec is None or prev_tvec is None:
            p_rvec = np.zeros((3, 1))
            p_tvec = np.zeros((3, 1))
        else:
            p_rvec = prev_rvec
            p_tvec = prev_tvec

        ret, p_rvec, p_tvec = cv2.aruco.estimatePoseCharucoBoard(c_corners, c_ids, board, camera_matrix, dist_coeff, p_rvec, p_tvec)

        if np.isnan(p_rvec).any() or np.isnan(p_tvec).any():
            return frame, prev_rvec, prev_tvec  

        # 축 그리기
        # 9.875, 9.875를 중심으로 좌표 이동
        center_x, center_y = 9.875, 9.875
        axis_length = 3  # 축 길이

        # 회전 행렬을 기반으로 축을 그리기
        axis = np.array([
            [center_x, center_y, 0],
            [center_x + axis_length, center_y, 0],
            [center_x, center_y + axis_length, 0],
            [center_x, center_y, axis_length]
        ])

        # 변환 벡터를 추가하여 축을 실제 좌표로 이동
        axis = axis + p_tvec.flatten()

        # 좌표계를 그리기
        for i in range(3):
            cv2.line(frame, 
                     (int(axis[i][0]), int(axis[i][1])), 
                     (int(axis[i+1][0]), int(axis[i+1][1])), 
                     (0, 0, 255), 5)
        
        calculate_camera_position_and_orientation(p_rvec, p_tvec)

    except cv2.error:
        return frame, prev_rvec, prev_tvec  

    return frame, p_rvec, p_tvec

def main():
    video_dev = 0
    frames = capture_camera(video_dev, 10)
    if len(frames) == 0:
        print('No frame captured')
        sys.exit(1)
    
    all_corners, all_ids, imsize = read_chessboards(frames)
    all_corners = [x for x in all_corners if len(x) >= 4]
    all_ids = [x for x in all_ids if len(x) >= 4]
    ret, camera_matrix, dist_coeff, rvec, tvec = cv2.aruco.calibrateCameraCharuco(all_corners, all_ids, board, imsize, None, None)

    print('> Camera matrix')
    print(camera_matrix)
    print('> Distortion coefficients')
    print(dist_coeff)

    cap = cv2.VideoCapture(video_dev)

    prev_rvec = None
    prev_tvec = None

    while True:
        ret, frame = cap.read()
        k = cv2.waitKey(1)
        if k == 27:  # Esc
            cap.release()
            break

        axis_frame, prev_rvec, prev_tvec = draw_axis(frame, camera_matrix, dist_coeff, board, prev_rvec, prev_tvec)
        
        cv2.imshow('Java', axis_frame)

if __name__ == '__main__':
    main()
