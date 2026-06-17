"""cv_bridge 우회 이미지 변환 유틸.

jazzy의 cv_bridge boost 모듈이 numpy 1.x로 컴파일돼 있어, numpy 2.x venv에서
imgmsg_to_cv2(desired_encoding=...)가 호출하는 cvtColor2(boost)에서 segfault가 난다.
이미지를 numpy로 직접 다뤄 cv_bridge(boost) 경로를 완전히 우회한다.

지원 인코딩: bgr8, rgb8 (3채널 8bit). yolo_seg_infer 는 bgr8 로 발행한다.
"""

import numpy as np
from sensor_msgs.msg import Image


def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    """sensor_msgs/Image(bgr8|rgb8) -> HxWx3 uint8 BGR(writable, contiguous)."""
    h, w = int(msg.height), int(msg.width)
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    step = int(msg.step) if msg.step else w * 3
    arr = buf.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
    if msg.encoding == 'rgb8':
        arr = arr[:, :, ::-1]
    # frombuffer는 read-only 라 cv2 in-place 연산이 거부될 수 있다. writable C-contiguous 복사.
    return np.array(arr, dtype=np.uint8, copy=True)


def bgr_to_imgmsg(arr: np.ndarray, header=None) -> Image:
    """HxWx3 uint8 BGR -> sensor_msgs/Image(bgr8)."""
    msg = Image()
    if header is not None:
        msg.header = header
    msg.height = int(arr.shape[0])
    msg.width = int(arr.shape[1])
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = int(arr.shape[1]) * 3
    msg.data = np.ascontiguousarray(arr, dtype=np.uint8).tobytes()
    return msg
