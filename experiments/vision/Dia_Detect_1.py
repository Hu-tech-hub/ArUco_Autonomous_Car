import cv2
import sys
from cv2 import aruco
import numpy as np
import math

# with ... as X: 파일을 열고 그 데이터를 X에 할당. with문을 사용하면 자동으로 파일을 닫아줌.
with np.load('param/calib_1109.npz') as X:
    camera_matrix = X['K']
    dist_coeffs = X['dists']
print(f'camera_matrix = {camera_matrix} \n')
print(f'dist_coeffs = {dist_coeffs}')

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

squareLength = 1.0  # Here, our measurement unit is centimetre.
markerLength = 0.5  # Here, our measurement unit is centimetre.

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

# 1: open video capture
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print('Error opening video')
    sys.exit()

while True:
    ret, frame = cap.read()  # Capture frame-by-frame
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)  # First, detect markers

        if ids is not None and len(ids) > 0:  # if there is at least one marker detected
            diamondCorners, diamondIds = aruco.detectCharucoDiamond(
                gray, corners, ids, squareLength / markerLength
            )  # Second, detect diamond markers

            # 다이아몬드 마커의 중심에 점 찍기 및 solvePnP 사용
            if len(diamondCorners) >= 1:  # if there is at least one diamond detected
                # Define the 3D points of the marker corners in the world coordinate system
                # Assume marker is a square in the XY plane at Z=0
                object_points = np.array([
                    [-markerLength / 2, -markerLength / 2, 0],
                    [ markerLength / 2, -markerLength / 2, 0],
                    [ markerLength / 2,  markerLength / 2, 0],
                    [-markerLength / 2,  markerLength / 2, 0]
                ], dtype=np.float32)

                for d_corners in diamondCorners:
                    # 다이아몬드 마커의 네 모서리 좌표로 solvePnP 계산
                    image_points = d_corners.reshape(-1, 2)

                    # Use solvePnP to estimate the rotation and translation vectors
                    success, rvec, tvec = cv2.solvePnP(
                        object_points, image_points, camera_matrix, dist_coeffs
                    )

                    if success:
                        calculate_camera_position_and_orientation(rvec, tvec)
                        
                    # 다이아몬드 중심에 파란 점 표시
                    cx = int(np.mean(d_corners[:, 0, 0]))
                    cy = int(np.mean(d_corners[:, 0, 1]))
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

        # 이미지 중심에 빨간 점 표시
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        cv2.imshow("diamondLeft", frame)  # display

        if cv2.waitKey(2) & 0xFF == ord('q'):  # press 'q' to quit
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()

