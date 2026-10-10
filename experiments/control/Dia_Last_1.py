#!/usr/bin/python3
# coding=utf8

import sys
sys.path.append('/home/pi/TurboPi/')  # Add the TurboPi directory to the system path
import cv2  # OpenCV for image processing
import time  # Time module for delays
import math  # Math module for mathematical operations
import signal  # Signal module for handling interrupts
import queue
import argparse  # Argument parsing module
import threading  # Threading module for concurrent execution
import numpy as np  # NumPy for numerical operations
import yaml_handle  # Custom module for handling YAML files
import HiwonderSDK.PID as PID  # PID control from HiwonderSDK
import HiwonderSDK.Misc as Misc  # Miscellaneous utilities from HiwonderSDK
import HiwonderSDK.Board as Board  # Board control from HiwonderSDK
import HiwonderSDK.mecanum as mecanum  # Mecanum wheel control from HiwonderSDK

#sudo python3 Last_move.py

#1. 이걸 카트에서 실행시켜 본다. 

#2. 이게 잘 돌아가면 이상 없는거다. 

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# Initialize the mecanum chassis
car = mecanum.MecanumChassis()

# Initial servo positions
servo1 = 1500
servo2 = 1500
servo_x = servo2
servo_y = servo1

# Aruco 사전 및 Charuco 보드 정의
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
# board = cv2.aruco.CharucoBoard_create(5, 5, 3.0, 1.5, aruco_dict)

# Initialize marker tracking variables
prev_rvec, prev_tvec = None, None
cam_pos, yaw = None, None
camera_matrix, dist_coeff = None, None


# Flags for car and wheel enablement
car_en = False
wheel_en = False
detect = False
__isRunning = False  
centering = False


#중심 좌표 
marker_center_x = None
marker_center_y = None
center_x, center_y = None, None

# PID 객체 초기화 rotation
rotate_pid = PID.PID(P=0.3, I=0.01, D=0.01)  # 적절한 값으로 튜닝
rotate_pid.setWindup(100)  # Windup 방지
rotate_pid.SetPoint = 0.0  # 목표 각도 (0°)

# PID 객체 초기화 centering
centering_pid_x = PID.PID(P=0.3, I=0.01, D=0.01)  # 적절한 값으로 튜닝
centering_pid_y = PID.PID(P=0.3, I=0.01, D=0.01)  # 적절한 값으로 튜닝
centering_pid_x.setWindup(100)  # Windup 방지
centering_pid_y.setWindup(100)  # Windup 방지
centering_pid_x.SetPoint = 0.0  # 목표 각도 (0°)
centering_pid_y.SetPoint = 0.0  # 목표 각도 (0°)

# Variables for configuration data
servo_data = None
size = (640, 480)  # Image size

def load_config():
    global camera_matrix, dist_coeff
    global servo_data

    # 카메라 파라미터 읽기? 근데 이거 Camera 모듈에 있지않나?
    with np.load('calibration_param.npz') as X:
        camera_matrix = X['K']
        dist_coeff = X['D']
    
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)
    # 초기 servo 모터값. 지금은 정면으로 되어있다. 초기를 천장으로 설정해 놓아라.
    

# Move servos to initial position
def initMove():
    Board.setPWMServoPulse(1, servo1, 1000)
    Board.setPWMServoPulse(2, servo2, 1000)

# Set the buzzer on the board
def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

# Stop the car by setting velocity to zero
def car_stop():
    car.set_velocity(0, 90, 0)  # Stop all motors

    # Reset all variables to their initial state
def reset():
    global car_en
    global servo1, servo2
    global servo_x, servo_y
    global prev_rvec, prev_tvec
    global cam_pos, yaw
    global detect
    # 초기화 시킬 변수 추가
    
    detect = False
    prev_rvec, prev_tvec = None, None
    cam_pos = np.array([0.0, 0.0, 0.0])
    yaw = None
    car_en = False
    servo1 = servo_data['servo1']
    servo2 = servo_data['servo2']
    servo_x = servo2
    servo_y = servo1
    rotate_pid.clear()

# Initialize the application
def init():
    print("Marker Tracking Init")
    load_config()
    reset()
    initMove()

# Start the application
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorTracking Start")

# Stop the application
def stop():
    global __isRunning
    reset()
    initMove()
    car_stop()
    __isRunning = False
    print("ColorTracking Stop")

def exit():
    global __isRunning
    reset()
    initMove()
    car_stop()
    __isRunning = False
    print("ColorTracking Exit")

def calculate_center(camera_matrix, dist_coeff, rvec, tvec, size):
    global center_x, center_y
    global marker_center_x, marker_center_y

    origin = np.float32([[7.5, 7.5, 0]]).reshape(-1, 3)
    points, _ = cv2.projectPoints(origin, rvec, tvec, camera_matrix, dist_coeff)
    
    projected_point = points[0].ravel().astype(int)

    print(f"Projected point: {projected_point}")

    center_x, center_y = size[0] // 2, size[1] // 2
    print(f"Image center: ({center_x}, {center_y})")

    marker_center_x = projected_point[0]
    marker_center_y = projected_point[1]

def calculate_camera_position_and_orientation(rvec, tvec):
    global yaw

    rotation_matrix, _ = cv2.Rodrigues(rvec)
    R_inv = np.linalg.inv(rotation_matrix)
    cam_pos = -np.dot(R_inv, tvec).flatten()
    pitch = math.atan2(-rotation_matrix[2, 0], math.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2))
    yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    print(f"Camera Position (X, Y, Z): {cam_pos[0]:.2f}, {cam_pos[1]:.2f}, {cam_pos[2]:.2f}")
    print(f"Yaw (Pan): {math.degrees(yaw):.2f} degrees")
    
    yaw = math.degrees(yaw)

def draw_custom_axis(frame, camera_matrix, dist_coeff, rvec, tvec, length=3):

    origin = np.float32([[7.5, 7.5, 0]]).reshape(-1, 3)

    axis = np.float32([
        [7.5 + length, 7.5, 0],
        [7.5, 7.5 + length, 0],
        [7.5, 7.5, length]
    ]).reshape(-1, 3)

    points, _ = cv2.projectPoints(np.vstack((origin, axis)), rvec, tvec, camera_matrix, dist_coeff)

    origin = tuple(points[0].ravel().astype(int))

    x_axis, y_axis, z_axis = points[1:]
    """
    나머지 점들은 X, Y, Z 축의 끝점을 나타내며, 이를 각각 x_axis, y_axis, z_axis에 저장합니다.
    """
    frame = cv2.line(frame, origin, tuple(x_axis.ravel().astype(int)), (0, 0, 255), 2)
    frame = cv2.line(frame, origin, tuple(y_axis.ravel().astype(int)), (0, 255, 0), 2)
    frame = cv2.line(frame, origin, tuple(z_axis.ravel().astype(int)), (255, 0, 0), 2)
    return frame

def calibrate_camera(video_dev=0, max_frames=10):
    global aruco_dict, board

    cap = cv2.VideoCapture(video_dev)
    if not cap.isOpened():
        print("Failed to open camera.")
        sys.exit(1)

    # Calibration frame capture
    frames = []
    print("Press Space to capture calibration frames. Press ESC to exit.")
    while len(frames) < max_frames:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame.")
            continue
        cv2.imshow("Calibration", frame)
        k = cv2.waitKey(1)
        if k == 32:  # Space
            frames.append(frame)
            print(f"Captured {len(frames)} frames.")
        elif k == 27:  # ESC
            cap.release()
            cv2.destroyAllWindows()
            sys.exit(0)

    cap.release()
    cv2.destroyAllWindows()

    # Process captured frames for calibration
    all_corners, all_ids, imsize = [], [], None
    for frame in frames:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
        if len(corners) > 0:
            ret, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if ret > 0 and len(c_corners) >= 4:
                all_corners.append(c_corners)
                all_ids.append(c_ids)
        imsize = gray.shape[::-1]

    if len(all_corners) < 1:
        print("Error: Not enough valid frames for calibration.")
        sys.exit(1)

    # Camera calibration
    ret, camera_matrix, dist_coeff, _, _ = cv2.aruco.calibrateCameraCharuco(
        all_corners, all_ids, board, imsize, None, None
    )

    if not ret:
        print("Error: Calibration failed.")
        sys.exit(1)

    print("> Calibration complete.")
    print("> Camera Matrix:\n", camera_matrix)
    print("> Distortion Coefficients:\n", dist_coeff)

    return camera_matrix, dist_coeff, imsize

def process_rotation():
    """목표 각도와 현재 각도에 대한 회전 처리"""
    global yaw, rotate_pid  # 전역 변수 사용

    # 현재 각도를 0~360 범위로 정규화
    yaw = (yaw + 360) % 360

    # 목표 각도와 현재 각도의 차이 계산
    delta_theta = rotate_pid.SetPoint - yaw 

    # 최소 회전 각도 계산 (180도를 넘으면 반대 방향으로 회전)
    if abs(delta_theta) > 180:
        delta_theta = delta_theta - 360 if delta_theta > 180 else delta_theta + 360

    rotate = rotate_pid.SetPoint - delta_theta

    # PID 업데이트 및 출력
    rotate_pid.update(rotate)
    pid_r_output = rotate_pid.output
    print(f'pid_r_output: {pid_r_output}')

    # 회전 속도 계산
    if abs(pid_r_output) >= 3:
        rotation_time = (min(25, abs(pid_r_output)) / 25) * 0.3
        direction = 2 if pid_r_output > 0 else -2  # 회전 방향 결정
        car.set_velocity(60, 90, direction)  # 회전 속도 적용
        time.sleep(rotation_time)
        car_stop()
    else:
        print("Target angle reached. Stopping rotation.")
        car_stop()

move_thread_lock = threading.Lock()

def move():
    global __isRunning, detect, centering  # 플래그 3개

    # size에서 가로, 세로 크기 추출
    width, height = size

    while True:
        if __isRunning:
            with move_thread_lock:  # Ensure safe access to shared variables
                if detect:
                    # 회전 처리 함수 호출
                    # 회전 처리 함수 호출
                    dx = center_x - marker_center_x
                    dy = center_y - marker_center_y

                    # centering 여부 확인
                    centering = abs(dx) > 10 or abs(dy) > 10

                    if centering:
                        # PID 업데이트
                        centering_pid_x.update(dx)
                        centering_pid_y.update(dy)

                         # pid_output 계산 및 클리핑, 정규화, 15 추가/제거
                        pid_x_output = max(-25, min(25, centering_pid_x.output))
                        pid_y_output = max(-25, min(25, centering_pid_y.output))

                        pid_x_output = (pid_x_output / 25) * 5 + (15 if pid_x_output > 0 else -15)
                        pid_y_output = (pid_y_output / 25) * 5 + (15 if pid_y_output > 0 else -15)

                        # 이동 명령
                        car.translation(pid_x_output, pid_y_output)
                        time.sleep(0.2)
                        car_stop()


                    process_rotation()

                    detect = False

        time.sleep(0.01)  # Avoid busy-waiting

# Run child thread for movement processing
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# Processing before exit
def manual_stop(signum, frame):
    global __isRunning
    
    print('Closing...')
    __isRunning = False
    car_stop()  # Turn off all motors
    initMove()  # Return motors to initial position

# 전역 변수
frame_queue = queue.Queue(maxsize=1)  # 캡처된 이미지를 저장할 큐
capture_thread_running = True         # 쓰레드 실행 상태 플래그

# Thread-safe stop flag
capture_thread_event = threading.Event()

def camera_capture_thread(video_dev=0):
    """
    Thread function to capture frames from the camera.
    """
    global frame_queue, size

    cap = cv2.VideoCapture(video_dev)
    if not cap.isOpened():
        print("Failed to open camera.")
        sys.exit(1)

    while not capture_thread_event.is_set():  # Use event to check thread termination
        ret, frame = cap.read()
        if not ret:
            print("Frame capture failed.")
            continue
        height, width = frame.shape[:2]
        size = (width, height)
        # Add the latest frame to the queue
        if frame_queue.full():
            frame_queue.get()  # Remove the oldest frame
        frame_queue.put(frame)

    cap.release()
    print("Camera capture thread stopped.")

# 1. Charuco Board
# def run(frame, camera_matrix, dist_coeff, prev_rvec, prev_tvec):
#     global detect
#     """
#     프레임 처리 및 Pose Estimation 실행 함수
#     """
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
#     if len(corners) > 0:
#         ret, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
#         if ret > 0:
#             ret, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
#                 c_corners, c_ids, board, camera_matrix, dist_coeff, prev_rvec, prev_tvec
#             )
#             if ret:
#                 frame = draw_custom_axis(frame, camera_matrix, dist_coeff, rvec, tvec)
#                 calculate_camera_position_and_orientation(rvec, tvec)
#                 calculate_center(camera_matrix, dist_coeff, rvec, tvec, size)
#                 prev_rvec, prev_tvec = rvec, tvec
#                 detect = True

#     return frame, prev_rvec, prev_tvec

#2. Diamond Charuco
def run(frame, camera_matrix, dist_coeff, prev_rvec, prev_tvec):
    global detect
    """
    프레임 처리 및 Pose Estimation 실행 함수
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
    if len(corners) > 0:
        ret, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
        if ret > 0:
            ret, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                c_corners, c_ids, board, camera_matrix, dist_coeff, prev_rvec, prev_tvec
            )
            if ret:
                frame = draw_custom_axis(frame, camera_matrix, dist_coeff, rvec, tvec)
                calculate_camera_position_and_orientation(rvec, tvec)
                calculate_center(camera_matrix, dist_coeff, rvec, tvec, size)
                prev_rvec, prev_tvec = rvec, tvec
                detect = True

    return frame, prev_rvec, prev_tvec

def main():
    global frame_queue

    # Start camera capture thread
    video_dev = 0
    capture_thread = threading.Thread(target=camera_capture_thread, args=(video_dev,))
    capture_thread.start()

    prev_rvec, prev_tvec = None, None
    print("Press ESC to exit.")

    try:
        while True:
            if not frame_queue.empty():
                # Get the latest frame from the queue
                frame = frame_queue.get()

                # Process the frame
                frame, prev_rvec, prev_tvec = run(frame, camera_matrix, dist_coeff, prev_rvec, prev_tvec)

                # Display the processed frame
                cv2.imshow("Pose Estimation", frame)

            if cv2.waitKey(1) == 27:  # ESC to exit
                break
    finally:
        # Ensure proper cleanup
        capture_thread_event.set()  # Signal the capture thread to stop
        capture_thread.join()  # Wait for the capture thread to exit
        cv2.destroyAllWindows()
        print(f'size: {size}')
        print("Program terminated.")

if __name__ == "__main__":
    init()  # Initialize the application
    start()  # Start the application
    main()
