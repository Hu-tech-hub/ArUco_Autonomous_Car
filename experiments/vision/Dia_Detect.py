import cv2
import sys
from cv2 import aruco
import numpy as np
import math

# with ... as X: 파일을 열고 그 데이터를 X에 할당. with문을 사용하면 자동으로 파일을 닫아줌.
with np.load('param/calibration_param.npz') as X:
    camera_matrix = X['K']
    dist_coeffs = X['D']
print(f'camera_matrix = {camera_matrix} \n')
print(f'dist_coeffs = {dist_coeffs}')

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

squareLength = 1.0  # Here, our measurement unit is centimetre.
markerLength = 0.5  # Here, our measurement unit is centimetre.


# ArUco 마커 감지 및 이미지 중심으로부터의 가장 가까운 마커 ID 계산
def find_closest_marker_to_center(gray, diamondCorners, diamondIds):

    # 이미지 크기에서 중심 계산
    img_center = np.array([gray.shape[1] / 2, gray.shape[0] / 2])  # (x, y)

    # 각 마커의 중심 좌표 계산 및 거리 측정
    min_distance = float('inf')
    closest_id = None

    for marker_corners, marker_id in zip(diamondCorners, diamondIds):
        # 마커의 코너 좌표 평균으로 중심 계산
        marker_center = np.mean(marker_corners, axis=0)

        # 이미지 중심과의 거리 계산
        distance = np.linalg.norm(marker_center - img_center)

        # 최소 거리 갱신
        if distance < min_distance:
            min_distance = distance
            closest_id = marker_id

    return closest_id

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
        closest_id = None

        #1. gray scale로 이미지를 변환한다.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #2. 마커의 id, corner를 얻는다.
        #입력: 그레이 스케일 이미지, 아루코 마커 딕셔너리
        #출력: corners: (n (n개 마커), 4 (코너 4개), 2 (x, y 두 좌표))
        #       ids : (n, 1) 
        #ex) corners[0]과 ids[0]이 매칭
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)  # First, detect markers

        # 마커가 하나라도 감지된다면 if문 실행
        if ids is not None and len(ids) > 0:  # if there is at least one marker detected
            
            #3 감지된 다이아몬드(중앙의 검은 사각형)의 4개 코너 좌표와, id를 출력
            #입력: 그레이 스케일 이미지, 마커의 코너, 마커의 id, 마커와 다이아몬드의 길이 비 
            #출력: diamondCorners: (n, 4, 2) (n개의 마커, 4개의 코너, x, y 두 좌표)
            #     diamondIds: (N, 1) (N개의 다이아몬드, 4개의 마커 ids로 조합된 ID)
            #GPT에서는 1,2,3,4 => 1234 라고 하던데 일단 출력해보자.
            diamondCorners, diamondIds = aruco.detectCharucoDiamond(
                gray, corners, ids, squareLength / markerLength)  
            
            # Second, detect diamond markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            if len(diamondCorners) >= 1:  # 하나 이상의 다이아몬드 마커가 탐지된 경우
                # 월드 좌표계에서 다이아몬드의 3D 코너 점 정의 (XY 평면, Z=0)
                closest_id = find_closest_marker_to_center(gray, diamondCorners, diamondIds)
                print(closest_id)

                object_points = np.array([
                    [-markerLength / 2, -markerLength / 2, 0], #좌하단
                    [ markerLength / 2, -markerLength / 2, 0], #우하단
                    [ markerLength / 2,  markerLength / 2, 0], #우상단
                    [-markerLength / 2,  markerLength / 2, 0]  #좌하단
                ], dtype=np.float32)

                for d_corners, d_id in zip(diamondCorners, diamondIds):
                    # 다이아몬드 마커의 네 모서리 좌표로 solvePnP 계산
                    if d_id == closest_id:
                        image_points = d_corners.reshape(-1, 2)

                        # Use solvePnP to estimate the rotation and translation vectors
                        success, rvec, tvec = cv2.solvePnP(
                            object_points, image_points, camera_matrix, dist_coeffs
                        )

                        if success:
                            calculate_camera_position_and_orientation(rvec, tvec)
                            # 좌표축을 그리기 위해 월드 좌표계의 축 정의
                            axis_points = np.array([
                                [0, 0, 0],       # 원점
                                [markerLength, 0, 0],  # X축
                                [0, markerLength, 0],  # Y축
                                [0, 0, -markerLength]  # Z축
                            ], dtype=np.float32)

                            # 월드 좌표계 축을 이미지 좌표계로 투영
                            img_points, _ = cv2.projectPoints(
                                axis_points, rvec, tvec, camera_matrix, dist_coeffs
                            )

                            # 축 그리기 (빨강: X, 초록: Y, 파랑: Z)
                            origin = tuple(img_points[0].ravel().astype(int))  # 원점
                            cv2.line(frame, origin, tuple(img_points[1].ravel().astype(int)), (0, 0, 255), 3)  # X축
                            cv2.line(frame, origin, tuple(img_points[2].ravel().astype(int)), (0, 255, 0), 3)  # Y축
                            cv2.line(frame, origin, tuple(img_points[3].ravel().astype(int)), (255, 0, 0), 3)  # Z축


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
