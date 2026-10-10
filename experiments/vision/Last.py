import cv2
import numpy as np
import sys
import math


#1. 이걸 카트에서 실행시켜 본다. 

#2. 이게 잘 돌아가면 이상 없는거다. 

camera_matrix, dist_coeff = None, None

def load_config():
    global camera_matrix, dist_coeff

    # 카메라 파라미터 읽기? 근데 이거 Camera 모듈에 있지않나?
    with np.load('CameraCalibration/calibration_param.npz') as X:
        camera_matrix = X['k_array']
        dist_coeff = X['d_array']
        # camera_matrix = np.array(X['k_array'].tolist())
        # dist_coeff = np.array(X['d_array'].tolist())


# Aruco 사전 및 Charuco 보드 정의
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard_create(5, 5, 3.875, 1.875, aruco_dict)

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

def draw_custom_axis(frame, camera_matrix, dist_coeff, rvec, tvec, length=3):

    origin = np.float32([[9.875, 9.875, 0]]).reshape(-1, 3)

    axis = np.float32([
        [9.875 + length, 9.875, 0],
        [9.875, 9.875 + length, 0],
        [9.875, 9.875, length]
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


def calibrate_camera(video_dev = 0, max_frames = 10):

    cap = cv2.VideoCapture(video_dev)
    if not cap.isOpened():
        print("Failed to open camera.")
        sys.exit(1)

    # Calibration frame capture``
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

def main():
    global camera_matrix, dist_coeff

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
        cv2.imshow("Pose Estimation", frame)
        if cv2.waitKey(1) == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

