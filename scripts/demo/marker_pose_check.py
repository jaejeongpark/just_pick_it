#!/usr/bin/env python3
"""마커 pose 정적 진단 (reverse docking 부호 검증 + Δx 깊이편향 진단용).

로봇을 움직이지 않고, reverse_docking 과 똑같은 파이프라인
(Picamera2 -> cv2.flip(-1) -> AprilTag 36h11 -> solvePnP IPPE_SQUARE,
camera_calibration.yaml 직접 로드)으로 마커 pose 를 윈도우(기본 2초) 단위로
평균내어 출력한다.

목적1 (부호 검증): 로봇을 '아는 위치'에 두고 tvec/rvec, robot world (x,y)·yaw 확인.
목적2 (Δx 깊이편향 진단): 로봇을 진짜 x(예 0.11)에 정렬해두고 y(깊이)만 옮기며
  각 깊이의 psi 평균을 비교한다. psi 가 깊이 무관 '상수 편향'이면
  Δx_full = -(tx·cosψ + tz·sinψ)·scale 의 tz·sinψ 항이 깊이에 비례하는 phantom 을
  만든다(= '같은 x 인데 깊이 따라 x 가 달라짐'의 원인). 윈도우 평균/표준편차로 확정.
목적3 (두-마커 가능성): 각 깊이에서 몇 개 마커가 동시 검출되는지, 둘 다 보이면
  translation 만으로(회전 rvec 안 씀) 계산한 두-마커 yaw 를 단일마커 psi 와 대조한다.

깊이 실험 절차: 로봇을 x=0.11 에 정렬 -> 한 윈도우 요약 읽기 -> y 만 이동(예
  0.40/0.30/0.20) -> 반복. 각 위치 요약의 psi 가 같은지(상수 편향) 확인.

보드에서 실행(카메라가 다른 노드에 안 잡혀 있어야 함):
  python3 scripts/demo/marker_pose_check.py            # 윈도우 2초
  python3 scripts/demo/marker_pose_check.py 3.0        # 윈도우 3초
"""
import math
import os
import sys
import time
from collections import defaultdict

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


def _mean_std(xs):
    if not xs:
        return 0.0, 0.0
    m = sum(xs) / len(xs)
    if len(xs) < 2:
        return m, 0.0
    var = sum((x - m) ** 2 for x in xs) / (len(xs) - 1)
    return m, math.sqrt(var)


def _median(xs):
    if not xs:
        return 0.0
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def _two_marker_pose(P0, P1, M0, M1):
    """두 마커의 카메라프레임 (tx,tz) 와 월드좌표로 카메라 월드 pose 강체정합.

    P_i=(tx,tz) 카메라프레임(z=전방=월드+y, x=우=월드+x at yaw0).
    회전 rvec 미사용. 반환 (Cx, Cy, yaw_deg). 깊이 underscale 가 있으면 결과도
    그만큼 어긋나므로 fx 보정 후 비교용.
    """
    (tx0, tz0), (tx1, tz1) = P0, P1
    (mx0, my0), (mx1, my1) = M0, M1
    # 마커선 기울기로 카메라 yaw: 카메라프레임 (Δtx,Δtz) vs 월드 (Δx,Δy)
    theta = math.atan2(tz1 - tz0, tx1 - tx0) - math.atan2(my1 - my0, mx1 - mx0)
    c, s = math.cos(theta), math.sin(theta)
    cxs, cys = [], []
    for (tx, tz), (mx, my) in ((P0, M0), (P1, M1)):
        vx = c * tx + s * tz      # 월드 x 변위
        vy = -s * tx + c * tz     # 월드 y 변위
        cxs.append(mx - vx)
        cys.append(my - vy)
    return sum(cxs) / 2, sum(cys) / 2, math.degrees(theta), abs(cxs[0] - cxs[1])


def main():
    window_sec = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    fx_scale = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
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
    # fx/fy 스케일(가설 검증용). 캘리브 fx 가 실제보다 작으면(영상모드 crop 불일치)
    # 거리·횡·회전이 전부 어긋난다. fx_scale>1 로 키워 tz 가 실제거리와 맞는지 본다.
    if fx_scale != 1.0:
        K = K.copy()
        K[0, 0] *= fx_scale
        K[1, 1] *= fx_scale
        print(f"[fx_scale] fx,fy x{fx_scale} -> fx={K[0,0]:.1f} fy={K[1,1]:.1f}")
    h = MARKER_SIZE / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    from picamera2 import Picamera2
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": (CAM_W, CAM_H), "format": "RGB888"}))
    cam.start()
    time.sleep(0.5)
    print(f"[cam] started {CAM_W}x{CAM_H} flip_180={FLIP_180} window={window_sec:.1f}s")
    print("로봇을 아는 위치에 두고(마커 바라봄) 윈도우 요약을 실제와 비교하세요. Ctrl-C 종료.\n")

    win_idx = 0
    try:
        while True:
            # ── 한 윈도우 동안 프레임을 모아 마커별 샘플 누적 ──────────────
            t_end = time.time() + window_sec
            frames = 0
            # id -> {tx, tz, ty, psi(채택), cx_px, both(set), e}
            samp = defaultdict(lambda: defaultdict(list))
            comb = defaultdict(int)   # 동시검출 조합(예 (0,1)) 빈도
            while time.time() < t_end:
                frame = cam.capture_array()
                if FLIP_180:
                    frame = cv2.flip(frame, -1)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = detector.detectMarkers(gray)
                frames += 1
                if ids is None:
                    time.sleep(0.01)
                    continue
                seen_ids = tuple(sorted(int(m) for m in ids.flatten()))
                comb[seen_ids] += 1
                for i, mid in enumerate(ids.flatten()):
                    mid = int(mid)
                    img_pts = corners[i][0].astype(np.float64)
                    n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
                        obj, img_pts, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    )
                    if n < 1:
                        continue
                    # ±π 평면 모호성: 두 해 중 재투영오차 작은 쪽 채택(노드 안정화와 동일 취지)
                    cand = []
                    for si in range(n):
                        Rs, _ = cv2.Rodrigues(rvecs[si])
                        psi_s = math.atan2(float(Rs[0, 2]), -float(Rs[2, 2]))
                        e = float(errs[si][0]) if errs is not None else 0.0
                        cand.append((e, psi_s, tvecs[si]))
                    cand.sort(key=lambda c: c[0])
                    e0, psi0, tvec0 = cand[0]
                    tx0, ty0, tz0 = (float(v) for v in tvec0.flatten())
                    d = samp[mid]
                    d["tx"].append(tx0)
                    d["ty"].append(ty0)
                    d["tz"].append(tz0)
                    d["psi"].append(psi0)
                    d["err"].append(e0)
                    d["cx"].append(float(corners[i][0][:, 0].mean()))
                time.sleep(0.01)

            # ── 윈도우 요약 출력 ────────────────────────────────────────
            win_idx += 1
            ndet = sorted(samp.keys())
            print(f"===== 윈도우 #{win_idx} | {frames}프레임/{window_sec:.1f}s | "
                  f"검출 마커 {len(ndet)}개 {ndet} =====")
            # 동시검출 조합 분포(두-마커 가능성 판단)
            if comb:
                combs = "  ".join(
                    f"{list(k)}:{v}f({100*v/max(frames,1):.0f}%)"
                    for k, v in sorted(comb.items(), key=lambda kv: -kv[1])
                )
                print(f"  동시검출 조합: {combs}")

            for mid in ndet:
                d = samp[mid]
                n = len(d["tx"])
                if n == 0:
                    continue
                tx_m, tx_s = _mean_std(d["tx"])
                tz_m, tz_s = _mean_std(d["tz"])
                psi_m, psi_s = _mean_std(d["psi"])
                psi_med = _median(d["psi"])
                cx_m, _ = _mean_std(d["cx"])
                side = "왼쪽" if cx_m < CAM_W / 2 else "오른쪽"
                print(f"  id={mid} (n={n}, 화면{side} cx={cx_m:.0f})")
                print(f"    tx={tx_m:+.4f}±{tx_s:.4f}  tz={tz_m:+.4f}±{tz_s:.4f}  "
                      f"psi={math.degrees(psi_m):+.2f}±{math.degrees(psi_s):.2f}deg "
                      f"(med {math.degrees(psi_med):+.2f})")
                if mid in MARKER_WORLD:
                    mwx, mwy = MARKER_WORLD[mid]
                    psi_n = psi_m - YAW_OFFSET
                    robot_y = mwy - tz_m * DEPTH_SCALE - CAM_FWD
                    # Δx 세 가지: full(노드 현행, tz·sinψ 포함) / tx만 / dec-
                    dx_full = -(tx_m * math.cos(psi_m) + tz_m * math.sin(psi_m)) \
                        * LATERAL_SCALE + LAT_OFFSET
                    dx_txonly = -tx_m * LATERAL_SCALE + LAT_OFFSET
                    # tz·sinψ phantom 단독 크기(깊이편향의 정체)
                    phantom = -tz_m * math.sin(psi_m) * LATERAL_SCALE
                    print(f"    psi_법선기준={math.degrees(psi_n):+.2f}deg(정면시~0)  "
                          f"robot_y_est={robot_y:.3f}(dock_y {DOCK.get(mid,('?','?'))[1]})")
                    print(f"    Δx_full={dx_full:+.4f}(x={mwx+dx_full:.4f})  "
                          f"Δx_tx만={dx_txonly:+.4f}(x={mwx+dx_txonly:.4f})  "
                          f"tz·sinψ_phantom={phantom:+.4f}  (목표 x={mwx:.3f})")

            # ── 두-마커 yaw (translation 만, 회전 rvec 미사용) ───────────────
            if 0 in samp and 1 in samp and samp[0]["tx"] and samp[1]["tx"]:
                tx0_m, _ = _mean_std(samp[0]["tx"])
                tz0_m, _ = _mean_std(samp[0]["tz"])
                tx1_m, _ = _mean_std(samp[1]["tx"])
                tz1_m, _ = _mean_std(samp[1]["tz"])
                dtx, dtz = (tx1_m - tx0_m), (tz1_m - tz0_m)
                # 두 마커 월드 벡터(0->1)는 +x 축. 카메라 프레임 벡터(dtx,dtz)의
                # 카메라 +x 대비 각도 = 벽 기준 카메라 yaw(부호는 실측으로 확정).
                psi_two = math.atan2(dtz, dtx)
                p0_m, _ = _mean_std(samp[0]["psi"])
                p1_m, _ = _mean_std(samp[1]["psi"])
                print(f"  [두-마커 yaw] psi_two={math.degrees(psi_two):+.2f}deg "
                      f"(translation만, pose-flip 없음)  vs 단일 psi: "
                      f"id0={math.degrees(p0_m):+.2f} id1={math.degrees(p1_m):+.2f}deg")
                # 강체정합 카메라 월드 pose (fx 가 맞으면 실제 카메라 위치와 일치해야 함).
                if 0 in MARKER_WORLD and 1 in MARKER_WORLD:
                    Cx, Cy, yaw_deg, resid = _two_marker_pose(
                        (tx0_m, tz0_m), (tx1_m, tz1_m),
                        MARKER_WORLD[0], MARKER_WORLD[1])
                    sep = math.hypot(dtx, dtz)
                    # 로봇중심 = 카메라 - 전방오프셋(헤딩방향). 카메라는 중심보다 0.06 앞이라
                    # 그대로 비교하면 yaw 만큼 옆으로 나가 보인다. 역산해 실제 로봇 x,y 로.
                    yr = math.radians(yaw_deg)
                    rcx = Cx - CAM_FWD * math.sin(yr)
                    rcy = Cy - CAM_FWD * math.cos(yr)
                    print(f"  [두-마커 pose] 카메라월드=({Cx:.3f},{Cy:.3f}) "
                          f"로봇중심=({rcx:.3f},{rcy:.3f}) yaw={yaw_deg:+.1f}deg "
                          f"| 두마커간격 측정={sep:.3f}(실제 "
                          f"{math.hypot(MARKER_WORLD[1][0]-MARKER_WORLD[0][0], MARKER_WORLD[1][1]-MARKER_WORLD[0][1]):.3f}) "
                          f"정합잔차={resid*1000:.0f}mm")
            print("-" * 70)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        cam.close()


if __name__ == "__main__":
    main()
