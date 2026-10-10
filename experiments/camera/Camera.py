#!/usr/bin/env python3
# encoding:utf-8
import sys
import cv2
import time
import threading
import numpy as np
from CameraCalibration.CalibrationConfig import *

# Call USB camera
# 이 프로그램은 USB 카메라를 사용하여 실시간 영상을 캡처하고, 
# 필요에 따라 왜곡을 보정한 영상을 제공합니다.

# Python 2가 아닌 Python 3에서 실행해야 함
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

class Camera:
    """
    USB 카메라를 관리하고 왜곡 보정 기능을 포함한 카메라 클래스
    """
    def __init__(self, resolution=(640, 480)):
        """
        Camera 클래스의 초기화 함수
        :param resolution: 캡처할 영상의 해상도 (기본값: 640x480)
        """
        self.cap = None
        self.width = resolution[0]  # 영상의 가로 해상도
        self.height = resolution[1]  # 영상의 세로 해상도
        self.frame = None  # 현재 프레임 저장
        self.opened = False  # 카메라 상태 플래그

        # 보정에 사용할 카메라 매개변수 로드
        self.param_data = np.load(calibration_param_path + '.npz')
        dim = tuple(self.param_data['dim_array'])  # 입력 영상 크기
        k = np.array(self.param_data['k_array'].tolist())  # 카메라 행렬
        d = np.array(self.param_data['d_array'].tolist())  # 왜곡 계수

        # 보정 행렬 계산
        p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, dim, None).copy()
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            k, d, np.eye(3), p, dim, cv2.CV_16SC2
        )

        # 카메라 작업을 수행할 별도의 스레드 시작
        self.th = threading.Thread(target=self.camera_task, args=(), daemon=True)
        self.th.start()

    def camera_open(self, correction=False):
        """
        카메라를 열고 설정 초기화
        :param correction: 왜곡 보정 여부 (기본값: False)
        """
        try:
            # 기본 카메라 열기
            self.cap = cv2.VideoCapture(-1)
            # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))  # 포맷 설정
            # self.cap.set(cv2.CAP_PROP_FPS, 30)  # 프레임 속도 설정
            # self.cap.set(cv2.CAP_PROP_SATURATION, 40)  # 채도 설정
            self.correction = correction  # 왜곡 보정 설정
            self.opened = True  # 카메라 상태를 열림으로 변경
        except Exception as e:
            print('Fail to open camera:', e)

    def camera_close(self):
        """
        카메라를 닫고 자원을 해제
        """
        try:
            self.opened = False  # 카메라 상태를 닫힘으로 변경
            time.sleep(0.2)
            if self.cap is not None:
                self.cap.release()  # 카메라 리소스 해제
                time.sleep(0.05)
            self.cap = None  # 객체 제거
        except Exception as e:
            print('Fail to close camera:', e)

    def camera_task(self):
        """
        카메라 작업 스레드: 실시간으로 프레임을 캡처
        """
        while True:
            try:
                # 카메라가 열려 있고 활성화된 경우
                if self.opened and self.cap.isOpened():
                    ret, frame_tmp = self.cap.read()  # 프레임 캡처
                    if ret:
                        # 해상도에 맞게 프레임 크기 조정
                        frame_resize = cv2.resize(frame_tmp, (self.width, self.height), interpolation=cv2.INTER_NEAREST)

                        # 왜곡 보정 수행 여부에 따라 다른 처리
                        if self.correction:
                            self.frame = cv2.remap(
                                frame_resize, self.map1, self.map2,
                                interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
                            )
                        else:
                            self.frame = frame_resize
                        ret = False
                    else:
                        # 프레임 캡처 실패 시 재시작
                        self.frame = None
                        self.cap.release()
                        cap = cv2.VideoCapture(-1)
                        ret, _ = cap.read()
                        if ret:
                            self.cap = cap
                elif self.opened:  # 카메라가 열려 있지만 비활성화된 경우
                    self.cap.release()
                    cap = cv2.VideoCapture(-1)
                    ret, _ = cap.read()
                    if ret:
                        self.cap = cap              
                else:  # 카메라가 닫혀 있으면 대기
                    time.sleep(0.01)
            except Exception as e:
                print('Fail to capture:', e)
                time.sleep(0.01)

if __name__ == '__main__':
    """
    메인 함수: 카메라를 실행하고 실시간으로 영상을 디스플레이
    """
    camera = Camera()  # Camera 객체 생성
    camera.camera_open()  # 카메라 열기
    while True:
        img = camera.frame  # 현재 프레임 가져오기
        if img is not None:
            cv2.imshow('img', img)  # 프레임 표시
            key = cv2.waitKey(1)  # 키 입력 대기
            if key == 27:  # ESC 키 입력 시 종료
                break
    camera.camera_close()  # 카메라 닫기
    cv2.destroyAllWindows()  # 모든 창 닫기

