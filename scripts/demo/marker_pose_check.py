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

# ── reverse_docking.yaml 에서 직접 읽어옴(노드와 동일 값 유지) ───────────
DOCK = {0: (0.11, 0.10), 1: (0.28, 0.10)}            # marker_id -> dock (x, y) 참고용
CAM_W, CAM_H = 1280, 720


def load_dock_params():
    """reverse_docking.yaml 의 ros__parameters 를 읽어 노드와 같은 보정값 사용."""
    try:
        from ament_index_python.packages import get_package_share_directory
        base = get_package_share_directory("pinky_amr_1")
        path = os.path.join(base, "params", "reverse_docking.yaml")
    except Exception:
        path = os.path.expanduser(
            "~/just_pick_it/src/just_pick_it/pinky_amr_1/params/reverse_docking.yaml"
        )
    with open(path) as f:
        d = yaml.safe_load(f)
    # 최상위 키가 '/**/reverse_docking' 형태 → 그 안의 ros__parameters
    node = next(iter(d.values()))
    p = node["ros__parameters"]
    ids = p["marker_ids"]
    mwx = p["marker_world_x"]
    mwy = p["marker_world_y"]
    marker_world = {int(i): (float(x), float(y)) for i, x, y in zip(ids, mwx, mwy)}
    print(f"[params] {path}\n  yaw_offset={p['marker_yaw_offset_deg']}deg "
          f"lat_offset={p['marker_lat_offset_m']} depth_scale={p['depth_scale']} "
          f"lateral_scale={p['lateral_scale']}")
    return {
        "MARKER_SIZE": float(p["marker_size_m"]),
        "CAM_FWD": float(p["camera_forward_offset_m"]),
        "DEPTH_SCALE": float(p["depth_scale"]),
        "LATERAL_SCALE": float(p["lateral_scale"]),
        "LAT_OFFSET": float(p["marker_lat_offset_m"]),
        "YAW_OFFSET_DEG": float(p["marker_yaw_offset_deg"]),
        "FLIP_180": bool(p.get("flip_camera_180", True)),
        "MARKER_WORLD": marker_world,
    }


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
    P = load_dock_params()
    MARKER_SIZE = P["MARKER_SIZE"]
    CAM_FWD = P["CAM_FWD"]
    DEPTH_SCALE = P["DEPTH_SCALE"]
    LATERAL_SCALE = P["LATERAL_SCALE"]
    LAT_OFFSET = P["LAT_OFFSET"]
    YAW_OFFSET = math.radians(P["YAW_OFFSET_DEG"])
    FLIP_180 = P["FLIP_180"]
    MARKER_WORLD = P["MARKER_WORLD"]
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
                        # 헤딩 오차 psi (마커 법선 기준 상대 yaw). 정면 정렬 시 ~0.
                        Rm, _ = cv2.Rodrigues(rvec)
                        psi = math.atan2(float(Rm[0, 2]), -float(Rm[2, 2]))
                        robot_y = mwy - tz * DEPTH_SCALE - CAM_FWD
                        rx_simple = mwx - tx                                  # psi=0 가정(현재 코드)
                        rx_dec = mwx - tx * math.cos(psi) + tz * math.sin(psi)   # 헤딩보정 +sin
                        rx_dec2 = mwx - tx * math.cos(psi) - tz * math.sin(psi)  # 헤딩보정 -sin
                        line += (
                            f"\n      robot_y_est={robot_y:.3f}(dock_y {DOCK.get(mid,('?','?'))[1]}) "
                            f"psi={math.degrees(psi):+.1f}deg "
                            f"robot_x: simple={rx_simple:.3f} dec+={rx_dec:.3f} dec-={rx_dec2:.3f} "
                            f"(실제 dock_x {DOCK.get(mid,('?',))[0]})"
                        )
                        # ── 보정 후(reverse_docking 노드와 동일 식): psi_n=psi-offset, Δx=측정식 ──
                        psi_n = psi - YAW_OFFSET
                        dx = (-(tx * math.cos(psi_n) + tz * math.sin(psi_n)) * LATERAL_SCALE
                              + LAT_OFFSET)
                        line += (
                            f"\n      [보정후] psi_법선기준={math.degrees(psi_n):+.1f}deg(정면시 ~0) "
                            f"Δx(node)={dx:+.3f} robot_x_est={mwx + dx:.3f} (목표 {mwx:.3f})"
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
