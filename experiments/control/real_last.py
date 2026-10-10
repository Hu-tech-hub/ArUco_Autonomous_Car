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

# Camera 모듈 안쓰겠음. 

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
prev_rvec, prev_tvec = None, None
cam_pos, yaw = None, None
camera_matrix, dist_coeff = None, None

# Flags for car and wheel enablement
car_en = False
size = (640, 480)  # Image size
__isRunning = False  # Flag to check if the program is running
detect_marker = False # Flag to check if the marker is detected
#__isRunning 변수는 Start()에서 True Ctrl+C 누르면 False

car_x_pid = PID.PID(P=0.15, I=0.001, D=0.0001)  # PID for car x-axis
car_y_pid = PID.PID(P=1.00, I=0.001, D=0.0001)  # PID for car y-axis

# Variables for configuration data
servo_data = None

def load_config():
    global camera_matrix, dist_coeff
    global servo_data

    with np.load('CameraCalibration/calibration_param.npz') as X:
        camera_matrix = X['k_array']
        dist_coeff = X['d_array']
    
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)
    # 초기 servo 모터값. 지금은 정면으로 되어있다. 초기를 천장으로 설정해 놓아라.

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
    global detec_marker
    # 초기화 시킬 변수 추가
     
    prev_rvec, prev_tvec = None, None
    detect_marker = False
    cam_pos = np.array([0.0, 0.0, 0.0])
    yaw = None
    car_en = False
    servo1 = servo_data['servo1']
    servo2 = servo_data['servo2']
    servo_x = servo2
    servo_y = servo1
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


def calculate_distance_from_center(frame, camera_matrix, dist_coeff, rvec, tvec, imsize):
    # (9.875, 9.875, 0) 좌표를 이미지 상에 투영
    origin = np.float32([[9.875, 9.875, 0]]).reshape(-1, 3)
    points, _ = cv2.projectPoints(origin, rvec, tvec, camera_matrix, dist_coeff)
    
    # 투영된 점의 좌표
    projected_point = points[0].ravel().astype(int)
    print(f"Projected point: {projected_point}")

    # 이미지의 중심
    center_x, center_y = imsize[0] // 2, imsize[1] // 2
    print(f"Image center: ({center_x}, {center_y})")

    # 중심과 투영된 점 사이의 거리 계산
    distance = math.sqrt((projected_point[0] - center_x)**2 + (projected_point[1] - center_y)**2)
    
    print(f"Distance from image center to projected point: {distance} pixels")
    return distance

def calculate_camera_position_and_orientation(rvec, tvec):
    global detect_marker
    """
    카메라의 위치와 방향을 계산하여 출력
    """
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    R_inv = np.linalg.inv(rotation_matrix)
    cam_pos = -np.dot(R_inv, tvec).flatten()
    pitch = math.atan2(-rotation_matrix[2, 0], math.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2))
    yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    print(f"Camera Position (X, Y, Z): {cam_pos[0]:.2f}, {cam_pos[1]:.2f}, {cam_pos[2]:.2f}")
    print(f"Yaw (Pan): {math.degrees(yaw):.2f} degrees")

    detect_marker = True



def draw_custom_axis(frame, camera_matrix, dist_coeff, rvec, tvec, length=3):

    origin = np.float32([[9.875, 9.875, 0]]).reshape(-1, 3)
    axis = np.float32([
        [9.875 + length, 9.875, 0],
        [9.875, 9.875 + length, 0],
        [9.875, 9.875, length]
    ]).reshape(-1, 3)

    points, _ = cv2.projectPoints(np.vstack((origin, axis)), rvec, tvec, camera_matrix, dist_coeff)

    origin = tuple(points[0].ravel().astype(int))
    """
    points[0]은 원점의 2D 좌표입니다. 이를 정수형으로 변환하여 사용합니다.
    """
    x_axis, y_axis, z_axis = points[1:]
    """
    나머지 점들은 X, Y, Z 축의 끝점을 나타내며, 이를 각각 x_axis, y_axis, z_axis에 저장합니다.
    """
    frame = cv2.line(frame, origin, tuple(x_axis.ravel().astype(int)), (0, 0, 255), 2)
    frame = cv2.line(frame, origin, tuple(y_axis.ravel().astype(int)), (0, 255, 0), 2)
    frame = cv2.line(frame, origin, tuple(z_axis.ravel().astype(int)), (255, 0, 0), 2)
    return frame


def calibrate_camera(video_dev = 0, max_frames = 10):

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

    return camera_matrix, dist_coeff

# Processing before exit
def manual_stop(signum, frame):
    global __isRunning
    print('Closing...')
    __isRunning = False
    car_stop()  # Turn off all motors
    initMove()  # Return motors to initial position

# Movement processing function
def move():
    global __isRunning, car_en
    global cam_pos, yaw
    
    while True:
        if __isRunning:
            if detect_marker == True
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

                car_en = 3333
                detect_marker = False
                
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

def main():
    global camera_matrix, dist_coeff
    global detect_marker

    video_dev = 0
    max_frames = 10

    while True:
        user_input = input('Do you want to rewrite camera_matrix and dist_coeff? (y/n): ').strip().lower()
        if user_input == 'y':
            print("Starting camera calibration...")
            camera_matrix, dist_coeff = calibrate_camera(video_dev, board=board, aruco_dict=aruco_dict, max_frames=max_frames)
            break
        elif user_input == 'n':
            print("Skipping camera calibration.")
            break
        else:
            print("Invalid input. Please enter 'y' or 'n'.")

    # 실시간 Pose Estimation
    prev_rvec, prev_tvec = None, None

    print("Press ESC to exit.")
    signal.signal(signal.SIGINT, manual_stop)

    cap = cv2.VideoCapture(video_dev)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame capture failed.")
            continue

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
                    distance = calculate_distance_from_center(frame, camera_matrix, dist_coeff, rvec, tvec, imsize)
                    prev_rvec, prev_tvec = rvec, tvec
                else :
                    detect_marker = False
        cv2.imshow("Pose Estimation", frame)
        if cv2.waitKey(1) == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

