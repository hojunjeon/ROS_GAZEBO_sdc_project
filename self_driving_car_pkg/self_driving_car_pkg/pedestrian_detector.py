# pedestrian_detector.py
import cv2

class PedestrianDetector:
    def __init__(self):
        # HOG 기반 보행자 검출기 초기화
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def detect(self, cv_image):
        """
        cv_image: OpenCV BGR 이미지 (numpy array)
        return: bool (보행자 탐지 여부)
        """
        # 보행자 탐지 수행
        boxes, weights = self.hog.detectMultiScale(cv_image, winStride=(8,8))
        detected = len(boxes) > 0
        return detected
