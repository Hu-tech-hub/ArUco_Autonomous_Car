#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/TurboPi/')  # Add the TurboPi directory to the system path
import cv2  # OpenCV for image processing
import time  # Time module for delays
import math  # Math module for mathematical operations
import signal  # Signal module for handling interrupts
import Camera  # Custom Camera module
import argparse  # Argument parsing module
import threading  # Threading module for concurrent execution
import numpy as np  # NumPy for numerical operations
import yaml_handle  # Custom module for handling YAML files
import HiwonderSDK.PID as PID  # PID control from HiwonderSDK
import HiwonderSDK.Misc as Misc  # Miscellaneous utilities from HiwonderSDK
import HiwonderSDK.Board as Board  # Board control from HiwonderSDK
import HiwonderSDK.mecanum as mecanum  # Mecanum wheel control from HiwonderSDK


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
board = cv2.aruco.CharucoBoard_create(5, 5, 3.875, 1.875, aruco_dict)

# Initialize marker tracking variables
marker_center_x, marker_center_y = -1, -1
prev_rvec, prev_tvec = None, None
cam_pos, yaw = None, None
camera_matrix, dist_coeff = None, None

# Flags for car and wheel enablement
car_en = False
wheel_en = False
size = (640, 480)  # Image size
# target_color = ()  # Target color for tracking
__isRunning = False  # Flag to check if the program is running
#__isRunning 변수는 Start()에서 True Ctrl+C 누르면 False


# PID controllers for car and servo movements
car_x_pid = PID.PID(P=0.15, I=0.001, D=0.0001)  # PID for car x-axis
car_y_pid = PID.PID(P=1.00, I=0.001, D=0.0001)  # PID for car y-axis
servo_x_pid = PID.PID(P=0.06, I=0.0003, D=0.0006)  # PID for servo x-axis
servo_y_pid = PID.PID(P=0.06, I=0.0003, D=0.0006)  # PID for servo y-axis

# Variables for configuration data
servo_data = None

def load_config():
    global servo_data
    global camera_matrix, dist_coeff
    # lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

    # 카메라 파라미터 읽기? 근데 이거 Camera 모듈에 있지않나?
    with np.load('CameraCalibration/calibration_param.npz') as X:
        camera_matrix = X['k_array']
        dist_coeff = X['d_array']
        # camera_matrix = np.array(X['k_array'].tolist())
        # dist_coeff = np.array(X['d_array'].tolist())

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
    global servo1, servo2, wheel_en
    global servo_x, servo_y
    global prev_rvec, prev_tvec
    global cam_pos, yaw
    global marker_center_x, marker_center_y
    # 초기화 시킬 변수 추가
     
    prev_rvec, prev_tvec = None, None
    cam_pos = np.array([0.0, 0.0, 0.0])
    yaw = None
    car_en = False
    wheel_en = False
    servo1 = servo_data['servo1']
    servo2 = servo_data['servo2']
    servo_x = servo2
    servo_y = servo1
    marker_center_x = -1
    marker_center_y = -1
    car_x_pid.clear()
    car_y_pid.clear()

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

# Enable or disable vehicle following
def setVehicleFollowing(state):
    global wheel_en
    print("wheel_en", state)
    wheel_en = state
    if not wheel_en:
        car_stop()
    return (True, ())

def calculate_camera_position_and_orientation(rvec, tvec):
    global cam_pos
    global yaw

    rotation_matrix, _ = cv2.Rodrigues(rvec)
    R_inv = np.linalg.inv(rotation_matrix)

    cam_pos = -np.dot(R_inv, tvec).flatten()
    
    #pitch = math.atan2(-rotation_matrix[2, 0], math.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2))
    yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    #roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])

    #print(f"Camera Position (X, Y, Z): {cam_pos[0]:.2f}, {cam_pos[1]:.2f}, {cam_pos[2]:.2f}")
    #print(f"Yaw (Pan): {math.degrees(yaw):.2f} degrees")


def calculate_marker_center(frame, camera_matrix, dist_coeff, rvec, tvec):
    # (9.875, 9.875, 0) 좌표를 이미지 상에 투영
    global marker_center_x, marker_center_y

    origin = np.float32([[9.875, 9.875, 0]]).reshape(-1, 3)
    points, _ = cv2.projectPoints(origin, rvec, tvec, camera_matrix, dist_coeff)
    
    # 투영된 점의 좌표
    projected_point = points[0].ravel().astype(int)
    marker_center_x, marker_center_y = projected_point

    # 이미지 상에 점을 찍기
    cv2.circle(frame, (marker_center_x, marker_center_y), 5, (0, 0, 255), -1)  # 빨간색 점 (5px 크기)

    #print(f"Projected point: {projected_point}")

    return frame


# Movement processing function
def move():
    global __isRunning, car_en, wheel_en
    global servo_x, servo_y
    global marker_center_x, marker_center_y
    global cam_pos, yaw
    
    while True:
        if __isRunning:
            if marker_center_x != -1 and marker_center_y != -1:
                # Camera pan-tilt tracking
                # Track based on the camera x-axis coordinate
                if abs(marker_center_x - img_w/2.0) < 15:  # Small movement, no need to adjust
                    marker_center_x = img_w/2.0

                servo_x_pid.SetPoint = img_w/2.0  # Set target point
                servo_x_pid.update(marker_center_x)  # Update PID with current position
                servo_x += int(servo_x_pid.output)  # Adjust servo position
                
                # Clamp servo position within range
                servo_x = 800 if servo_x < 800 else servo_x
                servo_x = 2200 if servo_x > 2200 else servo_x
                
                # Track based on the y-axis coordinate
                if abs(marker_center_y - img_h/2.0) < 10:  # Small movement, no need to adjust
                    marker_center_y = img_h/2.0
                servo_y_pid.SetPoint = img_h/2.0  # Set target point
                servo_y_pid.update(marker_center_y)  # Update PID with current position
                servo_y -= int(servo_y_pid.output)  # Adjust servo position
                
                # Clamp servo position within range
                servo_y = 1200 if servo_y < 1200 else servo_y
                servo_y = 1900 if servo_y > 1900 else servo_y
                
                # Set servo pulses
                Board.setPWMServoPulse(1, servo_y, 20)
                Board.setPWMServoPulse(2, servo_x, 20)
                time.sleep(0.01)
                
                # Car following
                if wheel_en:
                    # Track according to the target distance
                    if abs(cam_pos[2]) < 10:
                        car_y_pid.SetPoint = cam_pos[2]
                    else:
                        car_y_pid.SetPoint = 60
                    car_y_pid.update(cam_pos[2])
                    dy = car_y_pid.output  # Get PID output
                    dy = 0 if abs(dy) < 15 else dy  # Set speed range
                    
                    # Track according to the x-axis value of the servo
                    if abs(servo_x - servo2) < 15:
                        car_x_pid.SetPoint = servo_x
                    else:
                        car_x_pid.SetPoint = servo2
                    car_x_pid.update(servo_x)
                    dx = car_x_pid.output  # Get PID output
                    dx = 0 if abs(dx) < 15 else dx  # Set speed range
                    
                    # Set car translation speed
                    car.translation(dx, dy)
                    car_en = True
                
                time.sleep(0.01)
                
            else:
                if car_en:
                    car_stop()
                    car_en = False
                    print("Marker not detected. Stopping vehicle.")
        else:
            if car_en:
                car_stop()
                car_en = False
            time.sleep(0.01)

# Run child thread for movement processing
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

def run(img):
    global __isRunning
    global prev_rvec, prev_tvec
    global marker_center_x, marker_center_y
    global img_h, img_w

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning:
        return img
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
    if len(corners) > 0:
        ret, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
        if ret > 0:
            ret, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                c_corners, c_ids, board, camera_matrix, dist_coeff, prev_rvec, prev_tvec
            )
            if ret:
                calculate_camera_position_and_orientation(rvec, tvec)
                img = calculate_marker_center(img_copy, camera_matrix, dist_coeff, rvec, tvec)
                prev_rvec, prev_tvec = rvec, tvec
                if cam_pos[2] > 100.0:
                    cam_pos[2] = 0.0
                    marker_center_x = -1
                    marker_center_y = -1
                    print("Marker not detected. Stopping vehicle.")
                return img
            else: 
                cam_pos[2] = 0.0
                marker_center_x = -1
                marker_center_y = -1

    return img

# Processing before exit
def manual_stop(signum, frame):
    global __isRunning
    
    print('Closing...')
    __isRunning = False
    car_stop()  # Turn off all motors
    initMove()  # Return motors to initial position

if __name__ == '__main__':
    init()  # Initialize the application
    start()  # Start the application
    wheel_en = True  # Enable wheel movement
    camera = Camera.Camera()  # Initialize the camera
    camera.camera_open(correction=True)  # Open the camera with distortion correction
    signal.signal(signal.SIGINT, manual_stop)  # Set up signal handler for manual stop
    while __isRunning:
        img = camera.frame  # Get the current frame from the camera
        if img is not None:
            frame = img.copy()
            Frame = run(frame)  # Process the frame
            frame_resize = cv2.resize(Frame, (320, 240))  # Resize the frame for display
            cv2.imshow('frame', frame_resize)  # Show the frame
            key = cv2.waitKey(1)
            if key == 27:  # Exit on 'ESC' key
                break
        else:
            time.sleep(0.01)
    camera.camera_close()  # Close the camera
    cv2.destroyAllWindows()  # Destroy all OpenCV windows

