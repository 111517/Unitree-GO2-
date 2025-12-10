from .go2_video import get_go2_image
from .yolo_detector import YoloDetector
import cv2

def main():
    yolo = YoloDetector("/home/ayi/unitree_go2_ws/src/go2_camera/yolov8n.pt")

    
    frame = get_go2_image()  # 从 DDS 获取一帧

    _, annotated, dets = yolo.detect(frame)

        # 打印检测结果
    print(dets)

        # 显示画框后的画面
    cv2.imshow("YOLO Detection", annotated)
    cv2.imwrite("/home/ayi/unitree_go2_ws/src/go2_camera/runs/detect",annotated)
    cv2.waitKey(2) 

