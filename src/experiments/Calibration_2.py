#!/usr/bin/env python
import cv2
import numpy as np
import sys
import math

#가상환경에서 python3.9, OpenCV 4.5.1 버전에서만 실행 가능 
# 다음과 같이 터미널에 입력
# 1. source myvenv/Scripts/activate (이렇게 하면 가상환경이 실행됨)
# 2. cd RaspberryPi_Car
# 3. python Calibration_2.py 

#!! VideoCapture 객체 release 안시키면 다음 코드 실행에 문제가 생길 수 있음!!

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
# Note: Pattern generated using the following link
# https://calib.io/pages/camera-calibration-pattern-generator
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
            # ret is the number of detected corners
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
    # 회전 벡터를 회전 행렬로 변환
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    R_inv = np.linalg.inv(rotation_matrix)

    # 카메라 위치 계산
    cam_pos = -np.dot(R_inv, tvec)
    X, Y, Z = cam_pos.flatten()

    # Roll, Pitch, Yaw 계산
    pitch = math.atan2(-rotation_matrix[2, 0], math.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2))
    yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])

    # 라디안을 디그리로 변환
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)

    # 결과 출력
    print(f"Camera Position (X, Y, Z): {X:.2f}, {Y:.2f}, {Z:.2f}")
    print(f"Pan (Yaw): {yaw_deg:.2f} degrees")
    #print(f"Tilt (Pitch): {pitch_deg:.2f} degrees")
    #print(f"Roll: {roll_deg:.2f} degrees")

def capture_camera(dev_num=0, num=1, mirror=False, size=None):
    frames = []

    cap = cv2.VideoCapture(dev_num)

    while True:
        ret, frame = cap.read()

        if mirror is True:
            frame = cv2.flip(frame, 1)

        if size is not None and len(size) == 2:
            frame = cv2.resize(frame, size)

        # My config applies floating layout for windows named 'Java'
        cv2.imshow('Java', frame)

        k = cv2.waitKey(1)
        if k == 27:  # Esc
            cap.release()  # 기존 객체를 해제
            break
        elif k == 10 or k == 32:  # Enter or Space
            frames.append(frame)
            print('Frame captured!')
            if len(frames) == num:
                break

    return frames


def draw_axis(frame, camera_matrix, dist_coeff, board, verbose=True, prev_rvec=None, prev_tvec=None):
    """
    회전 벡터와 변환 벡터를 연속적으로 추정합니다.
    """
    corners, ids, rejected_points = cv2.aruco.detectMarkers(frame, aruco_dict)
    # rejected_points는 마커 후보(marker candidates)에 해당합니다. 이는 마커로 인식되지 않았지만 마커일 가능성이 있는 후보 영역을 나타냅니다.
    if corners is None or ids is None:
        return frame, prev_rvec, prev_tvec  # None을 반환하는 대신 frame과 이전 값 그대로 반환

    if len(corners) != len(ids) or len(corners) == 0:
        return frame, prev_rvec, prev_tvec  # 마찬가지로 유효하지 않으면 이전 값 그대로 반환

    try:
        ret, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, frame, board)

        # 첫 번째 추정 시에는 초기값을 np.zeros()로 설정
        if prev_rvec is None or prev_tvec is None:
            p_rvec = np.zeros((1, 3))  # 초기 회전 벡터 (ndarray로 설정)
            p_tvec = np.zeros((1, 3))  # 초기 변환 벡터 (ndarray로 설정)
        else:
            p_rvec = prev_rvec  # 이전 추정값을 사용
            p_tvec = prev_tvec  # 이전 추정값을 사용

        # 회전 벡터와 변환 벡터 추정
        ret, p_rvec, p_tvec = cv2.aruco.estimatePoseCharucoBoard(c_corners, c_ids, board, camera_matrix, dist_coeff, p_rvec, p_tvec)

        # 유효성 검사 (nan 값 확인)
        if np.isnan(p_rvec).any() or np.isnan(p_tvec).any():
            return frame, prev_rvec, prev_tvec  # 유효하지 않으면 이전 값 그대로 반환

        # 축 그리기
        cv2.aruco.drawAxis(frame, camera_matrix, dist_coeff, p_rvec, p_tvec, 3)

        calculate_camera_position_and_orientation(p_rvec, p_tvec)

    except cv2.error:
        return frame, prev_rvec, prev_tvec  # 예외 발생 시 이전 값 그대로 반환

    if verbose:
        print('Translation : {0}'.format(p_tvec))
        print('Rotation    : {0}'.format(p_rvec))
        print('Distance from camera: {0} m'.format(np.linalg.norm(p_tvec)))

    # 이전 회전 벡터와 변환 벡터를 반환
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
    ret, camera_matrix, dist_coeff, rvec, tvec = cv2.aruco.calibrateCameraCharuco( all_corners, all_ids, board, imsize, None, None )

    print('> Camera matrix')
    print(camera_matrix)
    print('> Distortion coefficients')
    print(dist_coeff)

    # Real-time axis drawing
    cap = cv2.VideoCapture(video_dev)

    prev_rvec = None
    prev_tvec = None

    while True:
        ret, frame = cap.read()
        k = cv2.waitKey(1)
        if k == 27:  # Esc
            cap.release()  # 기존 객체를 해제!!!!
            break

        # draw_axis가 None을 반환하지 않도록 수정한 코드
        axis_frame, prev_rvec, prev_tvec = draw_axis(frame, camera_matrix, dist_coeff, board, True, prev_rvec, prev_tvec)
        
        # frame을 출력
        cv2.imshow('Java', axis_frame)


if __name__ == '__main__':
    main()