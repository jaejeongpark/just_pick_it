"""선반 빈자리 선정 노드 (AMR drift robust).

eye-in-hand 카메라로 본 선반 위 물체(YOLO-seg mask)를 팔 base 프레임 메트릭으로 투영하고,
Canny+Hough로 검출한 선반 앞 모서리로 선반 좌표계를 복원한 뒤, 빈 영역 중
거리·여유·가장자리 margin 가중치로 최적 place 자리를 선정해 PoseStamped(팔 base)로 발행한다.

좌표 파이프라인 (모두 팔 base 프레임 기준 -> AMR이 어디 서 있든 무관):
  T_base_cam = T_base_ee(status coords, SDK FK) . T_ee_cam(hand-eye param)
  픽셀 (u,v) -> undistort -> base 광선 -> 선반 평면(z=shelf_plane_z) ray-plane 교점

사용 흐름:
  1) 각 스캔 자세(left/center/right)에서 정착 후 /place/capture_view (Empty) 발행 -> 그 뷰 누적
  2) 스캔 완료 후 /place/plan (Empty) 발행 -> 최적 자리 계산 및 /place/target_pose 발행
  3) 새 에피소드는 /place/reset (Empty) 로 누적 초기화
"""

import math

import cv2
import numpy as np
import yaml

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Float64MultiArray, Header

from just_pick_it_interfaces.msg import TrackedObjectArray


# status Float64MultiArray 레이아웃 (jetcobot_joint_subscriber.publish_status)
STATUS_COORDS_SLICE = slice(20, 26)   # [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]


def _rot_axis(axis: str, ang_rad: float) -> np.ndarray:
    c, s = math.cos(ang_rad), math.sin(ang_rad)
    if axis == 'x':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)
    if axis == 'y':
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def euler_to_R(rx: float, ry: float, rz: float, order: str, degrees: bool) -> np.ndarray:
    """order 문자열 순서대로 좌->우 행렬곱(R = R[order[0]] @ R[order[1]] @ ...)."""
    if degrees:
        rx, ry, rz = math.radians(rx), math.radians(ry), math.radians(rz)
    ang = {'x': rx, 'y': ry, 'z': rz}
    R = np.eye(3)
    for ax in order.lower():
        R = R @ _rot_axis(ax, ang[ax])
    return R


def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def quaternion_from_euler(rx: float, ry: float, rz: float) -> Quaternion:
    """roll(x), pitch(y), yaw(z) [rad] -> Quaternion (R = Rz@Ry@Rx)."""
    cy, sy = math.cos(rz * 0.5), math.sin(rz * 0.5)
    cp, sp = math.cos(ry * 0.5), math.sin(ry * 0.5)
    cr, sr = math.cos(rx * 0.5), math.sin(rx * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class ShelfPlacePlannerNode(Node):

    def __init__(self):
        super().__init__('shelf_place_planner_node')
        self._declare_parameters()
        self._bridge = CvBridge()

        self._K = None
        self._D = None
        self._load_camera_calibration()

        self._T_ee_cam = self._load_hand_eye()

        # 동기화용 최신 캐시
        self._latest_image = None
        self._latest_objects = []
        self._latest_coords = None  # np.array len6 (mm, deg)

        # 누적된 뷰 데이터 (한 docking 세션 동안 base 프레임에서 불변)
        self._captures = []  # [{'objects': [{'class','poly_xy','center_xy'}], 'edge_xy': Nx2}]

        ns = self.get_parameter('robot_name').value
        self.create_subscription(Image, 'infer/image_raw', self._image_cb, 5)
        self.create_subscription(
            TrackedObjectArray, 'infer/tracked_objects', self._objects_cb, 10)
        self.create_subscription(
            Float64MultiArray, f'/{ns}/status', self._status_cb, 10)

        self._req_status_pub = self.create_publisher(Empty, f'/{ns}/request_status', 10)
        self._pub_pose = self.create_publisher(PoseStamped, '/place/target_pose', 10)
        self._pub_debug = self.create_publisher(Image, '/place/debug_image', 5)

        self.create_subscription(Empty, '/place/capture_view', self._capture_cb, 10)
        self.create_subscription(Empty, '/place/plan', self._plan_cb, 10)
        self.create_subscription(Empty, '/place/reset', self._reset_cb, 10)

        self._pending_capture = False
        self.get_logger().info(
            'ShelfPlacePlannerNode 준비 완료. '
            'capture_view -> plan 순서로 트리거하세요.')

    # ------------------------------------------------------------------ params

    def _declare_parameters(self):
        self.declare_parameter('robot_name', 'jetcobot1')
        self.declare_parameter('camera_calibration_file', '')
        self.declare_parameter('base_frame', 'jetcobot1_base_link')

        # hand-eye T_ee_cam: [x,y,z (m), rx,ry,rz (deg)], euler order
        self.declare_parameter('hand_eye_xyzrpy', [0.0, 0.0, 0.05, 0.0, 0.0, 0.0])
        self.declare_parameter('hand_eye_euler_order', 'zyx')
        # status coords -> T_base_ee 회전 변환 convention
        self.declare_parameter('coords_euler_order', 'zyx')

        # 선반 dimension (m)
        self.declare_parameter('shelf_width_m', 0.40)    # 앞 모서리 방향 (W)
        self.declare_parameter('shelf_depth_m', 0.20)    # 깊이 방향 (L)
        self.declare_parameter('shelf_plane_z', 0.15)    # base 기준 선반 표면 높이
        self.declare_parameter('shelf_center_offset_m', 0.0)  # 앞모서리 중앙 anchoring 보정

        # occupancy / 후보
        self.declare_parameter('grid_res_m', 0.01)
        self.declare_parameter('place_object_radius_m', 0.03)
        self.declare_parameter('place_clearance_m', 0.01)
        self.declare_parameter('edge_margin_m', 0.02)

        # 가중치 (점수 최대화)
        self.declare_parameter('w_dist', 1.0)
        self.declare_parameter('w_clear', 0.6)
        self.declare_parameter('w_edge', 0.3)
        self.declare_parameter('w_reach', 0.4)

        # 팔 도달 반경 (base 원점 기준 xy 거리, m)
        self.declare_parameter('reach_min_m', 0.10)
        self.declare_parameter('reach_max_m', 0.45)

        # place pose orientation 기본값 (top-down)
        self.declare_parameter('place_roll_deg', 180.0)
        self.declare_parameter('place_pitch_deg', 0.0)
        self.declare_parameter('place_yaw_offset_deg', 0.0)
        self.declare_parameter('place_approach_height_m', 0.05)

        # edge detection
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('hough_threshold', 50)
        self.declare_parameter('hough_min_line_len', 40)
        self.declare_parameter('hough_max_line_gap', 10)
        self.declare_parameter('edge_roi_top_frac', 0.35)  # 상단 비율 무시(앞 모서리는 하단)
        self.declare_parameter('edge_max_image_slope', 0.5)  # |dy/dx| 이하만 near-horizontal
        self.declare_parameter('use_white_mask', True)
        self.declare_parameter('capture_delay_sec', 0.4)

        self.declare_parameter('debug_px_per_m', 400.0)

    def _load_camera_calibration(self):
        path = self.get_parameter('camera_calibration_file').value
        if not path:
            self.get_logger().warn('camera_calibration_file 미지정. 3D 투영 불가.')
            return
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            self._K = np.array(
                data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
            dist = data.get('distortion_coefficients', {}).get('data', [0, 0, 0, 0, 0])
            self._D = np.array(dist, dtype=np.float64).reshape(1, -1)
            self.get_logger().info(f'카메라 캘리브레이션 로드: {path}')
        except Exception as e:
            self.get_logger().error(f'캘리브레이션 로드 실패: {e}')

    def _load_hand_eye(self) -> np.ndarray:
        v = list(self.get_parameter('hand_eye_xyzrpy').value)
        order = self.get_parameter('hand_eye_euler_order').value
        R = euler_to_R(v[3], v[4], v[5], order, degrees=True)
        return make_T(R, np.array(v[:3], dtype=np.float64))

    # --------------------------------------------------------------- callbacks

    def _image_cb(self, msg: Image):
        self._latest_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _objects_cb(self, msg: TrackedObjectArray):
        self._latest_objects = list(msg.objects)

    def _status_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 26:
            self._latest_coords = np.array(msg.data[STATUS_COORDS_SLICE], dtype=np.float64)

    def _reset_cb(self, _msg):
        self._captures.clear()
        self.get_logger().info('누적 뷰 초기화 (reset)')

    def _capture_cb(self, _msg):
        # 최신 status 요청 후 약간의 지연을 두고 캡처(신선한 coords 확보).
        self._req_status_pub.publish(Empty())
        delay = self.get_parameter('capture_delay_sec').value
        if self._pending_capture:
            return
        self._pending_capture = True
        self._capture_timer = self.create_timer(delay, self._do_capture)

    def _do_capture(self):
        self._capture_timer.cancel()
        self._pending_capture = False
        self._capture_current_view()

    def _plan_cb(self, _msg):
        self._plan_and_publish()

    # ------------------------------------------------------------ 좌표 변환

    def _current_T_base_cam(self):
        if self._latest_coords is None:
            return None
        c = self._latest_coords
        t_ee = c[:3] / 1000.0  # mm -> m
        R_ee = euler_to_R(c[3], c[4], c[5],
                          self.get_parameter('coords_euler_order').value, degrees=True)
        T_base_ee = make_T(R_ee, t_ee)
        return T_base_ee @ self._T_ee_cam

    def _pixels_to_base_plane(self, uv: np.ndarray, T_base_cam: np.ndarray):
        """uv: Nx2 픽셀 -> (Nx2 base XY on plane, valid bool N)."""
        if self._K is None or uv.shape[0] == 0:
            return np.empty((0, 2)), np.zeros((uv.shape[0],), dtype=bool)
        pts = uv.reshape(-1, 1, 2).astype(np.float64)
        norm = cv2.undistortPoints(pts, self._K, self._D).reshape(-1, 2)
        dirs_cam = np.hstack([norm, np.ones((norm.shape[0], 1))])
        R = T_base_cam[:3, :3]
        o = T_base_cam[:3, 3]
        dirs_base = (R @ dirs_cam.T).T
        plane_z = self.get_parameter('shelf_plane_z').value
        denom = dirs_base[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            tparam = (plane_z - o[2]) / denom
        valid = (np.abs(denom) > 1e-6) & (tparam > 0) & np.isfinite(tparam)
        pts3 = o[None, :] + tparam[:, None] * dirs_base
        return pts3[:, :2], valid

    # ------------------------------------------------------------ 캡처 처리

    def _capture_current_view(self):
        T_base_cam = self._current_T_base_cam()
        if T_base_cam is None:
            self.get_logger().warn('status coords 없음 - 캡처 무시.')
            return
        if self._K is None:
            self.get_logger().warn('카메라 K 없음 - 캡처 무시.')
            return

        objs = []
        for o in self._latest_objects:
            if not o.mask_polygon:
                continue
            poly_px = np.array(o.mask_polygon, dtype=np.float64).reshape(-1, 2)
            poly_xy, valid = self._pixels_to_base_plane(poly_px, T_base_cam)
            poly_xy = poly_xy[valid]
            if poly_xy.shape[0] < 3:
                continue
            objs.append({
                'class': o.class_label,
                'poly_xy': poly_xy,
                'center_xy': poly_xy.mean(axis=0),
            })

        edge_xy = self._detect_edge_points(T_base_cam)

        self._captures.append({'objects': objs, 'edge_xy': edge_xy})
        self.get_logger().info(
            f'뷰 캡처 #{len(self._captures)}: objects={len(objs)}, '
            f'edge_pts={edge_xy.shape[0]}')

    def _detect_edge_points(self, T_base_cam: np.ndarray) -> np.ndarray:
        img = self._latest_image
        if img is None:
            return np.empty((0, 2))
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if self.get_parameter('use_white_mask').value:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            white = cv2.inRange(hsv, (0, 0, 150), (180, 60, 255))
            white = cv2.dilate(white, np.ones((9, 9), np.uint8))
            gray = cv2.bitwise_and(gray, gray, mask=white)

        # 상단 ROI 제거(앞 모서리는 하단부)
        top = int(h * self.get_parameter('edge_roi_top_frac').value)
        gray[:top, :] = 0

        edges = cv2.Canny(gray,
                          int(self.get_parameter('canny_low').value),
                          int(self.get_parameter('canny_high').value))
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180,
            int(self.get_parameter('hough_threshold').value),
            minLineLength=int(self.get_parameter('hough_min_line_len').value),
            maxLineGap=int(self.get_parameter('hough_max_line_gap').value))
        if lines is None:
            return np.empty((0, 2))

        max_slope = self.get_parameter('edge_max_image_slope').value
        sample_px = []
        for ln in lines[:, 0, :]:
            x1, y1, x2, y2 = map(float, ln)
            dx, dy = x2 - x1, y2 - y1
            if abs(dx) < 1e-3:
                continue
            if abs(dy / dx) > max_slope:  # near-horizontal 만
                continue
            n = max(2, int(math.hypot(dx, dy) / 5))
            for k in range(n + 1):
                a = k / n
                sample_px.append([x1 + a * dx, y1 + a * dy])
        if not sample_px:
            return np.empty((0, 2))

        uv = np.array(sample_px, dtype=np.float64)
        xy, valid = self._pixels_to_base_plane(uv, T_base_cam)
        return xy[valid]

    # ------------------------------------------------------------ 선반 좌표계

    def _build_shelf_frame(self):
        """누적 edge 점에서 앞 모서리 직선을 PCA fit -> (corner_xy, R_s 2x2, e_x, e_y)."""
        all_edge = [c['edge_xy'] for c in self._captures if c['edge_xy'].shape[0] > 0]
        W = self.get_parameter('shelf_width_m').value
        offset = self.get_parameter('shelf_center_offset_m').value

        if not all_edge:
            return None
        pts = np.vstack(all_edge)
        if pts.shape[0] < 5:
            return None

        c = pts.mean(axis=0)
        u, s, vt = np.linalg.svd(pts - c, full_matrices=False)
        e_x = vt[0]
        e_x = e_x / np.linalg.norm(e_x)
        if e_x[0] < 0:        # base x 방향으로 부호 통일
            e_x = -e_x
        e_y = np.array([-e_x[1], e_x[0]])
        if np.dot(e_y, c) < 0:   # 깊이는 base 원점에서 멀어지는 방향
            e_y = -e_y

        # base 원점을 앞 모서리에 정사영 -> foot
        foot = c + np.dot(-c, e_x) * e_x
        corner = foot - (W / 2.0 + offset) * e_x
        R_s = np.column_stack([e_x, e_y])
        return corner, R_s, e_x, e_y

    def _base_to_uv(self, p_xy, corner, R_s):
        return (R_s.T @ (np.asarray(p_xy) - corner).T).T

    # ------------------------------------------------------------ plan

    def _plan_and_publish(self):
        frame = self._build_shelf_frame()
        if frame is None:
            self.get_logger().warn('선반 모서리 검출 실패 - plan 불가. capture_view를 먼저 충분히 수행하세요.')
            return
        corner, R_s, e_x, e_y = frame

        W = self.get_parameter('shelf_width_m').value
        L = self.get_parameter('shelf_depth_m').value
        res = self.get_parameter('grid_res_m').value
        nu = max(1, int(math.ceil(W / res)))
        nv = max(1, int(math.ceil(L / res)))

        # occupancy: 1 = 점유(객체+margin). grid[row=v, col=u]
        occ = np.zeros((nv, nu), dtype=np.uint8)
        for cap in self._captures:
            for obj in cap['objects']:
                uv = self._base_to_uv(obj['poly_xy'], corner, R_s)
                col = uv[:, 0] / res
                row = uv[:, 1] / res
                poly = np.column_stack([col, row]).astype(np.int32)
                cv2.fillPoly(occ, [poly], 1)

        # place 물체 footprint + clearance 만큼 객체 팽창
        place_r = self.get_parameter('place_object_radius_m').value
        clear = self.get_parameter('place_clearance_m').value
        dil_cells = max(1, int(round((place_r + clear) / res)))
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * dil_cells + 1, 2 * dil_cells + 1))
        occ_dil = cv2.dilate(occ, kernel)

        # 가장자리 margin -> forbidden
        edge_margin = self.get_parameter('edge_margin_m').value
        m_cells = max(0, int(round(edge_margin / res)))
        valid_region = np.zeros((nv, nu), dtype=np.uint8)
        valid_region[m_cells:nv - m_cells, m_cells:nu - m_cells] = 1

        free = ((occ_dil == 0) & (valid_region == 1)).astype(np.uint8) * 255
        # 거리 변환 -> 각 free cell의 여유(m)
        dt = cv2.distanceTransform(free, cv2.DIST_L2, 5) * res

        need = place_r + clear
        candidate = dt >= need
        if not np.any(candidate):
            self.get_logger().warn('빈 자리 후보 없음 (선반이 가득 찼거나 검출 부족).')
            self._publish_debug(occ_dil, valid_region, candidate, None, corner, R_s)
            return

        best = self._score_candidates(candidate, dt, res, corner, R_s, L, W)
        if best is None:
            self.get_logger().warn('도달 가능한 후보 없음 (reach 범위 밖).')
            self._publish_debug(occ_dil, valid_region, candidate, None, corner, R_s)
            return

        (brow, bcol), base_xy = best
        self._publish_pose(base_xy, e_x)
        self._publish_debug(occ_dil, valid_region, candidate, (brow, bcol), corner, R_s)

    def _score_candidates(self, candidate, dt, res, corner, R_s, L, W):
        rows, cols = np.where(candidate)
        if rows.size == 0:
            return None
        uv = np.column_stack([(cols + 0.5) * res, (rows + 0.5) * res])
        base_xy = (corner[None, :] + (R_s @ uv.T).T)

        base_dist = np.linalg.norm(base_xy, axis=1)
        reach_min = self.get_parameter('reach_min_m').value
        reach_max = self.get_parameter('reach_max_m').value
        reachable = (base_dist >= reach_min) & (base_dist <= reach_max)
        if not np.any(reachable):
            return None

        clear = dt[rows, cols]
        edge_dist = np.minimum.reduce([uv[:, 0], W - uv[:, 0], uv[:, 1], L - uv[:, 1]])

        # 정규화
        dist_norm = np.clip(1.0 - base_dist / max(reach_max, 1e-6), 0, 1)
        clear_norm = np.clip(clear / (0.5 * min(W, L) + 1e-6), 0, 1)
        edge_norm = np.clip(edge_dist / (0.5 * min(W, L) + 1e-6), 0, 1)
        reach_ideal = 0.5 * (reach_min + reach_max)
        reach_norm = np.clip(
            1.0 - np.abs(base_dist - reach_ideal) / (0.5 * (reach_max - reach_min) + 1e-6),
            0, 1)

        w_dist = self.get_parameter('w_dist').value
        w_clear = self.get_parameter('w_clear').value
        w_edge = self.get_parameter('w_edge').value
        w_reach = self.get_parameter('w_reach').value
        score = (w_dist * dist_norm + w_clear * clear_norm
                 + w_edge * edge_norm + w_reach * reach_norm)
        score[~reachable] = -1e9

        idx = int(np.argmax(score))
        return (int(rows[idx]), int(cols[idx])), base_xy[idx]

    def _publish_pose(self, base_xy, e_x):
        z = (self.get_parameter('shelf_plane_z').value
             + self.get_parameter('place_approach_height_m').value)
        yaw = math.atan2(e_x[1], e_x[0]) + math.radians(
            self.get_parameter('place_yaw_offset_deg').value)
        roll = math.radians(self.get_parameter('place_roll_deg').value)
        pitch = math.radians(self.get_parameter('place_pitch_deg').value)

        pose = PoseStamped()
        pose.header = Header(stamp=self.get_clock().now().to_msg(),
                             frame_id=self.get_parameter('base_frame').value)
        pose.pose.position = Point(x=float(base_xy[0]), y=float(base_xy[1]), z=float(z))
        pose.pose.orientation = quaternion_from_euler(roll, pitch, yaw)
        self._pub_pose.publish(pose)
        self.get_logger().info(
            f'place 자리 선정: base xy=({base_xy[0]:.3f}, {base_xy[1]:.3f}), z={z:.3f}, '
            f'yaw={math.degrees(yaw):.1f}deg')

    # ------------------------------------------------------------ debug viz

    def _publish_debug(self, occ_dil, valid_region, candidate, best_rc, corner, R_s):
        if self._pub_debug.get_subscription_count() == 0:
            return
        nv, nu = occ_dil.shape
        scale = self.get_parameter('debug_px_per_m').value * self.get_parameter('grid_res_m').value
        scale = max(1, int(round(scale)))
        img = np.zeros((nv, nu, 3), dtype=np.uint8)
        img[valid_region == 1] = (60, 60, 60)        # 선반 유효 영역
        img[(occ_dil > 0)] = (40, 40, 160)           # 점유(객체+margin)
        img[candidate] = (40, 160, 40)               # 후보 free
        if best_rc is not None:
            cv2.circle(img, (best_rc[1], best_rc[0]), max(2, nu // 40), (0, 255, 255), -1)
        # v(깊이)축이 위로 가도록 뒤집고 확대 (u 오른쪽, v 위)
        img = cv2.flip(img, 0)
        img = cv2.resize(img, (nu * scale, nv * scale), interpolation=cv2.INTER_NEAREST)
        cv2.putText(img, 'u(width)->  v(depth)^', (5, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self._pub_debug.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ShelfPlacePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
