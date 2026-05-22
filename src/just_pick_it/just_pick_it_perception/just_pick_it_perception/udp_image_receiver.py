"""
Standalone UDP image receiver — ROS2 불필요, python3로 직접 실행.

Usage:
    python3 udp_image_receiver.py [--port 9870]
    q 키로 종료
"""
import argparse
import socket
import struct

import cv2
import numpy as np

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
            if img is not None:
                cv2.imshow('UDP Stream', img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    sock.close()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
