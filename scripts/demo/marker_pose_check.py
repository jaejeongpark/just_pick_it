#!/usr/bin/env python3
"""마커 pose 정적 진단 (reverse docking 부호 검증용).

로봇을 움직이지 않고, reverse_docking 과 똑같은 파이프라인
(Picamera2 -> cv2.flip(-1) -> AprilTag 36h11 -> solvePnP IPPE_SQUARE,
camera_calibration.yaml 직접 로드)으로 마커 pose 를 ~2Hz 로 출력한다.

목적: 로봇을 '아는 위치'(예: STANDBY_ZONE_1 x=0.11,y=0.40, 마커 바라봄)에 두고,
출력된 tvec/rvec 과 거기서 추정한 robot world (x,y)·yaw 가 실제와 맞는지 확인해
정렬 부호(tvec[0], rvec[1])를 확정한다.

보드에서 실행(카메라가 다른 노드에 안 잡혀 있어야 함):
  python3 scripts/demo/marker_pose_check.py
"""
import math
import os
import time

import cv2
import numpy as np
import yaml

# ── reverse_docking.yaml 과 동일한 값 ──────────────────────────────────
MARKER_SIZE = 0.05                       # AprilTag 36h11 한 변 (m)
CAM_FWD = 0.055                          # base_link 에서 카메라 전방 오프셋 (로봇0.40/카메라0.455)
MARKER_WORLD = {0: (0.07, 0.635), 1: (0.28, 0.635)}  # marker_id -> (world_x, world_y) 벽0.64/마커0.635
DOCK = {0: (0.11, 0.10), 1: (0.28, 0.10)}            # marker_id -> dock (x, y) 참고용
FLIP_180 = True
CAM_W, CAM_H = 1280, 720


def load_calib():
    try:
        from ament_index_python.packages import get_package_share_directory
        base = get_package_share_directory("just_pick_it_perception")
        path = os.path.join(base, "result", "camera_calibration.yaml")
    except Exception:
        path = os.path.expanduser(
            "~/just_pick_it/src/just_pick_it/just_pick_it_perception/result/camera_calibration.yaml"
        )
    with open(path) as f:
        d = yaml.safe_load(f)
    K = np.array(d["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    dist = np.array(d["distortion_coefficients"]["data"], dtype=np.float64)
    print(f"[calib] {path}\n  fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}")
    return K, dist


def main():
    K, dist = load_calib()
    h = MARKER_SIZE / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    from picamera2 import Picamera2
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": (CAM_W, CAM_H), "format": "RGB888"}))
    cam.start()
    time.sleep(0.5)
    print(f"[cam] started {CAM_W}x{CAM_H} flip_180={FLIP_180}")
    print("로봇을 아는 위치에 두고(마커 바라봄) 아래 값을 실제와 비교하세요. Ctrl-C 종료.\n")

    try:
        while True:
            frame = cam.capture_array()
            if FLIP_180:
                frame = cv2.flip(frame, -1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is None:
                print("  (마커 미검출)")
            else:
                for i, mid in enumerate(ids.flatten()):
                    mid = int(mid)
                    ok, rvec, tvec = cv2.solvePnP(
                        obj, corners[i][0].astype(np.float64), K, dist,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    )
                    if not ok:
                        continue
                    tx, ty, tz = (float(v) for v in tvec.flatten())
                    rx, ry, rz = (float(v) for v in rvec.flatten())
                    # 픽셀 중심 대비 마커 중심 위치(부호 직관 확인용)
                    cx_px = float(corners[i][0][:, 0].mean())
                    side = "왼쪽" if cx_px < CAM_W / 2 else "오른쪽"
                    line = (
                        f"id={mid} | tvec[x,y,z]=({tx:+.3f},{ty:+.3f},{tz:+.3f}) "
                        f"rvec=({rx:+.2f},{ry:+.2f},{rz:+.2f}) "
                        f"| 마커 화면 {side}(cx={cx_px:.0f})"
                    )
                    if mid in MARKER_WORLD:
                        mwx, mwy = MARKER_WORLD[mid]
                        robot_y = mwy - tz - CAM_FWD          # 현 코드 깊이 추정
                        rxm = mwx - tx                         # 횡 부호 가설 A
                        rxp = mwx + tx                         # 횡 부호 가설 B
                        line += (
                            f"\n      robot_y_est={robot_y:.3f}(dock_y {DOCK.get(mid,('?','?'))[1]}) "
                            f"robot_x_est: (mwx-tx)={rxm:.3f} / (mwx+tx)={rxp:.3f} "
                            f"(dock_x {DOCK.get(mid,('?',))[0]}) yaw(rvec_y)={math.degrees(ry):+.1f}deg"
                        )
                    print("  " + line)
            print("-" * 70)
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        cam.close()


if __name__ == "__main__":
    main()
