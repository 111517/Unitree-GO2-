import cv2
from ultralytics import YOLO

class YoloDetector:
    def __init__(self, model_path="yolov8n.pt"):
        self.model = YOLO(model_path)

    def detect(self, frame):
        """
        输入: frame (BGR 图像)
        输出:
            results: YOLO 原始结果对象
            annotated_img: 画框后的图像
            detections(list): 解析后的检测信息
        """
        results = self.model(frame, stream=False)

        detections = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls = int(box.cls[0])

                detections.append({
                    "cls": cls,
                    "conf": conf,
                    "xyxy": xyxy
                })

        annotated = results[0].plot()  # 画框图

        return results, annotated, detections
