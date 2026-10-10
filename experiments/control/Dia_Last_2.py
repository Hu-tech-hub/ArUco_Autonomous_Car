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
from collections import deque
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

# Initialize marker tracking variables
squareLength = 1.0  
markerLength = 0.5  
cam_pos, yaw = None, None
closest_id = None
camera_matrix, dist_coeff = None, None


# Flags for car and wheel enablement
car_en = False
wheel_en = False
detect = False
__isRunning = False  
centering = False
rotation = False

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
centering_pid_x.SetPoint = 0.0 
centering_pid_y.SetPoint = 0.0  


#최단거리 알고리즘
start_node = 1
end_node = 16
move_next = False
prev_id = None

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
    global cam_pos, yaw
    global detect
    global centering
    global rotation
    global closest_id
    # 초기화 시킬 변수 추가
    
    rotation = False
    centering = False
    detect = False
    closest_id = None
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

# ArUco 마커 감지 및 이미지 중심으로부터의 가장 가까운 마커 ID 계산
def find_closest_marker_to_center(gray, diamondCorners, diamondIds):
    global center_x, center_y
    global marker_center_x, marker_center_y

    # 이미지 크기에서 중심 계산
    img_center = np.array([gray.shape[1] / 2, gray.shape[0] / 2])  # (x, y)

    center_x, center_y = img_center

    # 각 마커의 중심 좌표 계산 및 거리 측정
    min_distance = float('inf')
    closest_id = None

    for marker_corners, marker_id in zip(diamondCorners, diamondIds):
        # 마커의 코너 좌표 평균으로 중심 계산
        marker_center = np.mean(marker_corners, axis=0)

        marker_center_x = marker_center[0][0]
        marker_center_y = marker_center[0][1]

        # 이미지 중심과의 거리 계산
        distance = np.linalg.norm(marker_center - img_center)

        # 최소 거리 갱신
        if distance < min_distance:
            min_distance = distance
            closest_id = marker_id

    return closest_id

def calculate_camera_position_and_orientation(rvec, tvec): ## 그대로 사용
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

def draw_custom_axis(frame, camera_matrix, dist_coeff, rvec, tvec):

    # 좌표축을 그리기 위해 월드 좌표계의 축 정의
    axis_points = np.array([
                            [0, 0, 0],       # 원점
                            [markerLength, 0, 0],  # X축
                            [0, markerLength, 0],  # Y축
                            [0, 0, -markerLength]  # Z축
                            ], dtype=np.float32)

    # 월드 좌표계 축을 이미지 좌표계로 투영
    img_points, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeff)

                        # 축 그리기 (빨강: X, 초록: Y, 파랑: Z)
    origin = tuple(img_points[0].ravel().astype(int))  # 원점
    cv2.line(frame, origin, tuple(img_points[1].ravel().astype(int)), (0, 0, 255), 3)  # X축
    cv2.line(frame, origin, tuple(img_points[2].ravel().astype(int)), (0, 255, 0), 3)  # Y축
    cv2.line(frame, origin, tuple(img_points[3].ravel().astype(int)), (255, 0, 0), 3)  # Z축

# ArUco 마커 ID -> 노드 번호 매핑
def map_marker_to_node(marker_id):
    marker_to_node_map = {
    tuple(range(i, i + 4)): node
    for node, i in enumerate(range(1, 65, 4), start=1)}
    marker_tuple = tuple(marker_id.flatten())  # ndarray를 튜플로 변환
    return marker_to_node_map.get(marker_tuple, None)  # 노드 번호 반환

'''
marker_to_node_map = {
    (1, 2, 3, 4): 1,
    (5, 6, 7, 8): 2,
    (9, 10, 11, 12): 3,
    (13, 14, 15, 16): 4,
    (17, 18, 19, 20): 5,
    (21, 22, 23, 24): 6,
    (25, 26, 27, 28): 7,
    (29, 30, 31, 32): 8,
    (33, 34, 35, 36): 9,
    (37, 38, 39, 40): 10,
    (41, 42, 43, 44): 11,
    (45, 46, 47, 48): 12,
    (49, 50, 51, 52): 13,
    (53, 54, 55, 56): 14,
    (57, 58, 59, 60): 15,
    (61, 62, 63, 64): 16
}
'''

# 노드 번호 -> aruco id
def get_aruco_ids_for_node(node_number):
    start_id = (node_number - 1) * 4 + 1
    end_id = start_id + 4
    return np.array([list(range(start_id, end_id))])

# 노드와 회전 각도 매핑(입력 시작 노드 번호, 종점 노드 번호)
def calculate_node_based_angles_with_mapping(start, end, grid_size=4):
    # 이동 방향 설정 (상, 하, 좌, 우)
    directions = {
        (-1, 0): 180,  # 상단 (↑)
        (1, 0): 0,     # 하단 (↓)
        (0, -1): 270,  # 좌측 (←)
        (0, 1): 90     # 우측 (→)
    }

    # BFS를 이용한 최단 경로 탐색
    def find_shortest_path(start, end):
        queue = deque([(start, [start])])  # (현재 노드, 경로)
        visited = set()

        while queue:
            current, path = queue.popleft()
            if current == end:
                return path  # 종점 도달 시 경로 반환

            x, y = (current - 1) // grid_size, (current - 1) % grid_size
            for dx, dy in directions.keys():
                nx, ny = x + dx, y + dy
                if 0 <= nx < grid_size and 0 <= ny < grid_size:
                    next_node = nx * grid_size + ny + 1
                    if next_node not in visited:
                        visited.add(next_node)
                        queue.append((next_node, path + [next_node]))
        return []

    # 최단 경로 찾기
    path = find_shortest_path(start, end)
    if not path:
        return None, None, None  # 경로가 없으면 None 반환

    # 각도 계산 및 매핑 딕셔너리 생성
    angles = []
    node_to_angle_mapping = {}
    for i in range(len(path) - 1):
        current_node = path[i]
        next_node = path[i + 1]

        # 현재 노드와 다음 노드의 좌표 계산
        x1, y1 = (current_node - 1) // grid_size, (current_node - 1) % grid_size
        x2, y2 = (next_node - 1) // grid_size, (next_node - 1) % grid_size

        # 이동 방향 계산
        dx, dy = x2 - x1, y2 - y1
        angle = directions[(dx, dy)]
        angles.append(angle)

        # ArUco ID 매핑 추가
        aruco_id = get_aruco_ids_for_node(current_node)
        node_to_angle_mapping[current_node] = {"aruco_id": aruco_id, "angle": angle}

    # 마지막 노드도 매핑에 추가 (종료점에는 각도가 없음)
    aruco_id = get_aruco_ids_for_node(path[-1])
    node_to_angle_mapping[path[-1]] = {"aruco_id": aruco_id, "angle": None}

    return node_to_angle_mapping

# `closest_id`가 ndarray 형식이므로 이를 기반으로 노드를 찾음
def find_node_from_aruco_id(closest_id, node_to_angle_mapping):
    closest_id_str = str(closest_id)
    for node, data in node_to_angle_mapping.items():
        if str(data["aruco_id"]) == closest_id_str:
            return node, data["angle"]  # 노드와 각도 반환
    return None, None  # 일치하는 노드가 없으면 None 반환

# 현재 위치에 따른 rotate 값 설정
def update_rotate_pid(closest_id, start_node, end_node):
    node_to_angle_mapping = calculate_node_based_angles_with_mapping(start_node, end_node)
    _, angle = find_node_from_aruco_id(closest_id, node_to_angle_mapping)
    if angle is not None:
        rotate_pid.SetPoint = angle
    return angle

def process_rotation():
    """목표 각도와 현재 각도에 대한 회전 처리"""
    global yaw, rotate_pid, rotation  # 전역 변수 사용

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
        rotation = False
        time.sleep(rotation_time)
        car_stop()
    else:
        print("Target angle reached. Stopping rotation.")
        rotation = True
        car_stop()

def process_centering():
    global centering, centering_pid_x, centering_pid_y

    # 회전 처리 관련 변수
    dx = marker_center_x - center_x
    dy = marker_center_y - center_y

    # centering 여부 확인
    centering = abs(dx) < 10 or abs(dy) < 10

    if not centering:
        # PID 업데이트
        centering_pid_x.update(dx)
        centering_pid_y.update(dy)

        # pid_output 계산 및 클리핑, 정규화, 15 추가/제거
        pid_x_output = max(-25, min(25, centering_pid_x.output))
        pid_y_output = max(-25, min(25, centering_pid_y.output))

        # 벡터 합 계산
        vector_x = pid_x_output
        vector_y = pid_y_output
        vector_magnitude = math.sqrt(vector_x**2 + vector_y**2)

        # 벡터 크기를 0 ~ 0.3으로 정규화
        max_magnitude = 25 * math.sqrt(2)  # 최대 벡터 크기 (-25, 25의 범위에서 대각선일 때)
        normalized_magnitude = (vector_magnitude / max_magnitude) * 0.3

        # Calculate angle theta in radians
        theta = math.atan2(dy, dx)
    
        # Calculate dx and dy
        x = -40 * math.cos(theta)  # Negative 40 * cos(theta)
        y = 40 * math.sin(theta)   # Positive 40 * sin(theta)

        # 이동 명령
        car.translation(x, y)
        time.sleep(normalized_magnitude)
        car_stop()

    else: 
        print("Target position reached. Stopping rotation.")

move_thread_lock = threading.Lock()

def move():
    global __isRunning, detect, centering, rotation, move_next  # 플래그 3개
    global closest_id, prev_id
    global rotate_pid
    global start_node

    # rotate 값 초기화
    rotate_pid.SetPoint = 0  # 기본 값: 0도

    while True:
        if __isRunning:
            with move_thread_lock:  # 전역 변수 보호
                if detect:
                    if move_next:
                        if np.array_equal(closest_id, prev_id):
                            car.set_velocity(40, 90, 0)
                            time.sleep(0.3)
                            car_stop()
                            continue
                        else : 
                            move_next = False
                    # 중심 맞추기와 회전 수행
                    # 초기 angle 설정
                    angle = update_rotate_pid(closest_id, start_node, end_node)

                    # angle이 None인 경우 start_node 업데이트 후 다시 angle 설정
                    if angle is None:
                        start_node = map_marker_to_node(closest_id)
                        update_rotate_pid(closest_id, start_node, end_node)

                    process_centering()
                    process_rotation()

                    # 중심과 회전 작업 완료 후 detect 플래그 해제
                    detect = False

                    if rotation and centering:
                        closest_id = prev_id
                        move_next = True 

                elif move_next:
                    car.set_velocity(40, 90, 0)
                    time.sleep(0.3)
                    car_stop()
                else:
                    car_stop()
        
        time.sleep(0.01)  # Busy-waiting 방지

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


def monitor_key_input():
    global __isRunning
    while True:
        if user_input == 'q':
            __isRunning = False
            print("Motor stopped manually.")
        elif user_input == 'w':
            if not __isRunning:
                __isRunning = True
                print("Motor started manually.")
            else:
                print("Motor is already running.")


# 카메라 쓰레드를 위한전역 변수
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

#2. Diamond Charuco
def run(frame, camera_matrix, dist_coeff):
    global detect
    global closest_id

    frame_copy = frame.copy()

    #1. gray scale로 이미지를 변환한다.
    gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)

    #2. 마커의 id, corner를 얻는다.
    #입력: 그레이 스케일 이미지, 아루코 마커 딕셔너리
    #출력: corners: (n (n개 마커), 4 (코너 4개), 2 (x, y 두 좌표))
    #       ids : (n, 1) 
    #ex) corners[0]과 ids[0]이 매칭
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)  # First, detect markers

    # 마커가 하나라도 감지된다면 if문 실행
    if ids is not None and len(ids) > 0:  # if there is at least one marker detected
            
        #3 감지된 다이아몬드(중앙의 검은 사각형)의 4개 코너 좌표와, id를 출력
        #입력: 그레이 스케일 이미지, 마커의 코너, 마커의 id, 마커와 다이아몬드의 길이 비 
        #출력: diamondCorners: (n, 4, 2) (n개의 마커, 4개의 코너, x, y 두 좌표)
        #     diamondIds: (N, 4) (N개의 다이아몬드, 4개의 마커 ids로 조합된 ID)
        diamondCorners, diamondIds = cv2.aruco.detectCharucoDiamond(gray, corners, ids, squareLength / markerLength)  
            
        # Second, detect diamond markers
        cv2.aruco.drawDetectedMarkers(frame_copy, corners, ids)

        if len(diamondCorners) >= 1:  # 하나 이상의 다이아몬드 마커가 탐지된 경우
            # 월드 좌표계에서 다이아몬드의 3D 코너 점 정의 (XY 평면, Z=0)
            closest_id = find_closest_marker_to_center(gray, diamondCorners, diamondIds)

            object_points = np.array([
                [-markerLength / 2, -markerLength / 2, 0], #좌하단
                [ markerLength / 2, -markerLength / 2, 0], #우하단
                [ markerLength / 2,  markerLength / 2, 0], #우상단
                [-markerLength / 2,  markerLength / 2, 0]  #좌하단
            ], dtype=np.float32)

            for d_corners, d_id in zip(diamondCorners, diamondIds):
                # 다이아몬드 마커의 네 모서리 좌표로 solvePnP 계산
                if np.array_equal(d_id, closest_id):   #같은 배열인지 확인하는 방법이다. 즉 가장 가까운 마커에서만 진행한다는 뜻
                    image_points = d_corners.reshape(-1, 2)

                    # Use solvePnP to estimate the rotation and translation vectors
                    success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeff)

                    if success:
                        calculate_camera_position_and_orientation(rvec, tvec)

                        draw_custom_axis(frame_copy, camera_matrix, dist_coeff, rvec, tvec)

                        detect = True
    return frame_copy

def main():
    global frame_queue, __isRunning

    # Start camera capture thread
    video_dev = 0
    capture_thread = threading.Thread(target=camera_capture_thread, args=(video_dev,))
    capture_thread.start()

    print("Press ESC to exit.")

    try:
        while True:
            if not frame_queue.empty():
                # Get the latest frame from the queue
                frame = frame_queue.get()

                # Process the frame
                frame= run(frame, camera_matrix, dist_coeff)

                # Display the processed frame
                cv2.imshow("Pose Estimation", frame)

            if cv2.waitKey(1) == 27:  # ESC to exit
                break
    finally:
        # Ensure proper cleanup
        capture_thread_event.set()  # Signal the capture thread to stop
        capture_thread.join()  # Wait for the capture thread to exit
        cv2.destroyAllWindows()
        print("Program terminated.")

        #11/29 Esc 누른 후 종료되는 코드 추가
        __isRunning = False
        car_stop()  # Turn off all motors
        initMove()  # Return motors to initial position
        

if __name__ == "__main__":
    init()  # Initialize the application

    user_input = input("Enter 's' to begin: ").strip()
    while user_input.lower() != 's':
        print("Invalid input. Please type 's' to continue.")
        user_input = input("Enter 's' to begin: ").strip()
    
    # Input 모니터링을 별도의 스레드로 실행
    threading.Thread(target=monitor_key_input, daemon=True).start()
    
    start()  # Start the application

    #11/29 ctrl + c 누르면 종료 함수 추가
    signal.signal(signal.SIGINT, manual_stop)
    main()