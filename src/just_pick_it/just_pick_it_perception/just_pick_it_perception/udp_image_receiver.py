"""
Standalone UDP image receiver — ROS2 불필요, python3로 직접 실행.

Usage:
    python3 udp_image_receiver.py [--port 9870] [--save-dir ~/img_capture]
    spacebar: 현재 프레임 캡처
    q 키로 종료
"""
import argparse
import socket
import struct
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

HEADER_FMT = '>IHH'
HEADER_SIZE = struct.calcsize(HEADER_FMT)
RECV_BUF = 65536


def main():
    parser = argparse.ArgumentParser(description='UDP image stream receiver')
    parser.add_argument('--port', type=int, default=9870)
    parser.add_argument('--save-dir', type=str, default=None,
                        help='Directory for captured images (default: ~/img_capture)')
    args = parser.parse_args()

    capture_dir = Path(args.save_dir).expanduser() if args.save_dir \
        else Path.home() / 'img_capture'

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', args.port))
    sock.settimeout(1.0)
    print(f'Listening on UDP port {args.port} ... (Space: capture, q: quit)')
    print(f'Save dir: {capture_dir}')

    frames: dict[int, dict[int, bytes]] = {}
    img_count = len(list(capture_dir.glob('*.png'))) if capture_dir.exists() else 0

    while True:
        try:
            packet, addr = sock.recvfrom(RECV_BUF)
        except socket.timeout:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
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

            img = cv2.flip(img, 0)

            overlay = img.copy()
            cv2.putText(overlay, f'captured: {img_count}  Space: save  q: quit',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow('UDP Stream', overlay)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):
                capture_dir.mkdir(parents=True, exist_ok=True)
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                save_path = capture_dir / f'capture_{timestamp}.png'
                cv2.imwrite(str(save_path), img)
                img_count += 1
                print(f'캡처 저장: {save_path}')

    sock.close()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
