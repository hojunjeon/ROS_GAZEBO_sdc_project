# pedestrian_detector.py
import cv2

class PedestrianDetector:
    def __init__(self):
        # HOG 기반 보행자 검출기 초기화
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def detect(self, cv_image, visualize=False):
        """
        cv_image: OpenCV BGR 이미지 (numpy array)
        visualize: True면 bbox 시각화
        return: bool (보행자 탐지 여부)
        """
        # 보행자 탐지 수행
        boxes, weights = self.hog.detectMultiScale(cv_image, winStride=(8, 8))
        detected = len(boxes) > 0

        if visualize:
            for (x, y, w, h) in boxes:
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(cv_image, f"Pedestrians: {len(boxes)}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.imshow("Pedestrian Detection", cv_image)
            cv2.waitKey(1)

        return detected
