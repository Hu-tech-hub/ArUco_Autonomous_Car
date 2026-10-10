import cv2
import numpy as np
import sys
import math

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
    """
    중심 (9.875, 9.875)을 기준으로 축을 그립니다.
    """
    origin = np.float32([[9.875, 9.875, 0]]).reshape(-1, 3)
    """
    원점은 (9.875, 9.875, 0)에 설정됩니다. 여기서 9.875는 사용자 지정 값이며, 보드의 중심 위치로 생각할 수 있습니다.
    """
    axis = np.float32([
        [9.875 + length, 9.875, 0],
        [9.875, 9.875 + length, 0],
        [9.875, 9.875, length]
    ]).reshape(-1, 3)
    """
    axis는 3개의 점을 포함하는 배열입니다. 이 점들은 각 축(X, Y, Z)의 끝점을 나타냅니다. 예를 들어:
    x-axis: (9.875 + length, 9.875, 0) => X축 방향.
    y-axis: (9.875, 9.875 + length, 0) => Y축 방향.
    z-axis: (9.875, 9.875, length) => Z축 방향.
    이 값들은 모두 length를 더해주어, 기본 원점에서 length만큼 떨어진 위치를 나타냅니다.
    """

    points, _ = cv2.projectPoints(np.vstack((origin, axis)), rvec, tvec, camera_matrix, dist_coeff)
    """
    np.vstack((origin, axis))는 원점과 축의 끝점들을 합쳐서 3D 공간에서의 모든 점을 하나의 배열로 만듭니다. (vertical stack)
    cv2.projectPoints()는 3D 점들을 카메라 좌표계를 기준으로 2D 이미지 좌표로 변환하는 함수입니다. 
    이 함수는 회전 벡터(rvec)와 변환 벡터(tvec)를 이용하여 카메라의 위치와 방향을 반영한 후, 결과적으로 frame의 2D 이미지에서 그려질 위치를 계산합니다.
    반환값 points는 2D 이미지 상의 점들입니다.
    """

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

def main():
    video_dev = 0
    cap = cv2.VideoCapture(video_dev)
    if not cap.isOpened():
        print("Failed to open camera.")
        sys.exit(1)

    # Calibration 프레임 캡처
    frames = []
    print("Press Space to capture calibration frames. Press ESC to exit.")
    while len(frames) < 10:
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

    #검출 후에 닫는 걸로 보완.
    
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
        cap.release()
        cv2.destroyAllWindows()
        sys.exit(1)

    ret, camera_matrix, dist_coeff, _, _ = cv2.aruco.calibrateCameraCharuco(
        all_corners, all_ids, board, imsize, None, None
    )
    print("> Calibration complete.")
    print("> Camera Matrix:\n", camera_matrix)
    print("> Distortion Coefficients:\n", dist_coeff)

    # 실시간 Pose Estimation
    prev_rvec, prev_tvec = None, None
    print("Press ESC to exit.")
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

