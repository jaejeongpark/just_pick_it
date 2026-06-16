"""
Standalone UDP image → AVI recorder — ROS2 불필요, python3로 직접 실행.

Usage:
    python3 udp_image_video_gen.py [--port 9870] [--output-dir ~/videos] [--fps 30]
    r 키: 녹화 시작 / 정지
    q 키: 종료 (녹화 중이면 자동 저장)
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

FOURCC = cv2.VideoWriter_fourcc(*'MJPG')


def make_output_path(output_dir: Path) -> Path:
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return output_dir / f'record_{timestamp}.avi'


def main():
    parser = argparse.ArgumentParser(description='UDP image stream → AVI recorder (MJPEG)')
    parser.add_argument('--port', type=int, default=9870)
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Directory to save AVI files (default: ~/videos)')
    parser.add_argument('--fps', type=float, default=30.0,
                        help='Video FPS (default: 30)')
    parser.add_argument('--robot-name', type=str, default='UDP Recorder',
                        help='Window title (e.g. picky_1, picky_2)')
    args = parser.parse_args()

    output_dir = Path(args.output_dir).expanduser() if args.output_dir \
        else Path.home() / 'videos'

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', args.port))
    sock.settimeout(1.0)
    print(f'Listening on UDP port {args.port} ... (r: record start/stop, q: quit)')
    print(f'Output dir: {output_dir}')

    frames: dict[int, dict[int, bytes]] = {}

    writer: cv2.VideoWriter | None = None
    recording = False
    frame_count = 0
    video_path: Path | None = None

    while True:
        try:
            packet, _ = sock.recvfrom(RECV_BUF)
        except socket.timeout:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        if len(packet) < HEADER_SIZE:
            continue

        frame_id, pkt_idx, total = struct.unpack(HEADER_FMT, packet[:HEADER_SIZE])
        chunk = packet[HEADER_SIZE:]

        frames.setdefault(frame_id, {})[pkt_idx] = chunk

        if len(frames[frame_id]) != total:
            continue

        data = b''.join(frames[frame_id][i] for i in range(total))
        del frames[frame_id]

        stale = [fid for fid in list(frames) if fid < frame_id - 30]
        for fid in stale:
            del frames[fid]

        img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue

        # img = cv2.flip(img, -1)

        if recording and writer is not None:
            writer.write(img)
            frame_count += 1

        overlay = img.copy()
        if recording:
            status = f'[REC] {frame_count} frames  r: stop  q: quit'
            color = (0, 0, 255)
        else:
            status = 'r: start rec  q: quit'
            color = (0, 255, 0)
        cv2.putText(overlay, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.imshow(args.robot_name, overlay)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            if not recording:
                h, w = img.shape[:2]
                output_dir.mkdir(parents=True, exist_ok=True)
                video_path = make_output_path(output_dir)
                writer = cv2.VideoWriter(str(video_path), FOURCC, args.fps, (w, h))
                recording = True
                frame_count = 0
                print(f'녹화 시작: {video_path}')
            else:
                recording = False
                if writer is not None:
                    writer.release()
                    writer = None
                print(f'녹화 완료: {video_path} ({frame_count} frames)')

    if recording and writer is not None:
        writer.release()
        print(f'녹화 자동 저장: {video_path} ({frame_count} frames)')

    sock.close()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
