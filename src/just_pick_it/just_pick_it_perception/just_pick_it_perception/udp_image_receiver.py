"""
Standalone UDP image receiver — ROS2 불필요, python3로 직접 실행.

Usage:
    python3 udp_image_receiver.py [--port 9870]
    spacebar: 현재 프레임 캡처 (~/img_capture/)
    q 키로 종료
"""
import argparse
import socket
import struct
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

CAPTURE_DIR = Path.home() / 'img_capture'

HEADER_FMT = '>IHH'
HEADER_SIZE = struct.calcsize(HEADER_FMT)
RECV_BUF = 65536


def main():
    parser = argparse.ArgumentParser(description='UDP image stream receiver')
    parser.add_argument('--port', type=int, default=9870)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', args.port))
    sock.settimeout(1.0)
    print(f'Listening on UDP port {args.port} ... (q to quit)')

    frames: dict[int, dict[int, bytes]] = {}

    while True:
        try:
            packet, addr = sock.recvfrom(RECV_BUF)
        except socket.timeout:
            continue

        if len(packet) < HEADER_SIZE:
            continue

        frame_id, pkt_idx, total = struct.unpack(HEADER_FMT, packet[:HEADER_SIZE])
        chunk = packet[HEADER_SIZE:]

        frames.setdefault(frame_id, {})[pkt_idx] = chunk

        if len(frames[frame_id]) == total:
            data = b''.join(frames[frame_id][i] for i in range(total))
            del frames[frame_id]

            # 오래된 미완성 프레임 제거
            stale = [fid for fid in list(frames) if fid < frame_id - 30]
            for fid in stale:
                del frames[fid]

            img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            img_rot = cv2.rotate(img, cv2.ROTATE_180)
            img_rot_rgb = cv2.cvtColor(img_rot, cv2.COLOR_BGR2RGB)
            cv2.imshow('UDP Stream', img_rot_rgb)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):
                CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                save_path = CAPTURE_DIR / f'capture_{timestamp}.png'
                # PNG: 무손실 저장으로 UDP 디코딩 이후 픽셀 품질 유지
                cv2.imwrite(str(save_path), img_rot_rgb)
                print(f'캡처 저장: {save_path}')

    sock.close()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
