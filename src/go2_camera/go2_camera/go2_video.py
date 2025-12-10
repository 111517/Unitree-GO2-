{"id":"92015","variant":"standard","title":"go2_video.py(Camera DDS Wrapper)"}
#!/usr/bin/env python3
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient
import numpy as np
import cv2
import time
import threading

# 全局单例对象（避免多次 init）
_global_video_client = None
_global_lock = threading.Lock()


class Go2VideoClient:
    def __init__(self, nic_name="ens33", timeout=3.0):
        """
        初始化 DDS + VideoClient
        """
        print(f"[Go2Video] 初始化 DDS 通道，绑定网卡: {nic_name}")
        ChannelFactoryInitialize(0, nic_name)

        self.client = VideoClient()
        self.client.SetTimeout(timeout)
        self.client.Init()

        self.reconnect_fail_count = 0

    def get_frame(self):
        """
        拉取一帧图像
        返回: OpenCV格式(BGR)的numpy图像 或 None
        """
        try:
            code, data = self.client.GetImageSample()
        except Exception as e:
            print("[Go2Video] DDS 调用异常:", e)
            return None

        if code != 0:
            print(f"[Go2Video] 获取图像失败 code={code}")
            self.reconnect_fail_count += 1
            time.sleep(0.05)

            # 如果多次失败尝试重连
            if self.reconnect_fail_count > 20:
                print("[Go2Video] 重连视频客户端...")
                try:
                    self.client.Init()
                    self.reconnect_fail_count = 0
                except Exception as e:
                    print("[Go2Video] 重连失败:", e)
            return None

        # 将数据转换为 numpy
        try:
            if isinstance(data, (bytes, bytearray, memoryview)):
                arr = np.frombuffer(data, dtype=np.uint8)
            else:
                arr = np.frombuffer(bytes(data), dtype=np.uint8)
        except Exception as e:
            print("[Go2Video] 转换图像缓冲区失败:", e)
            return None

        # 解码 JPEG 压缩图像
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            print("[Go2Video] 图像解码失败")
            return None

        return img


def get_go2_image():
    """
    全局函数：供其他节点调用，获取 GO2 摄像头图像
    """
    global _global_video_client, _global_lock

    # 单例初始化（线程安全）
    if _global_video_client is None:
        with _global_lock:
            if _global_video_client is None:
                _global_video_client = Go2VideoClient("ens33")

    return _global_video_client.get_frame()
