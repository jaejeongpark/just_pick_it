"""배치 종단 처리(결정론적, C-1 폴백).

place release 의 NN(Phase D) 학습 전에도 DISPLAY_PLACE 전체 파이프라인을 검증할 수 있도록,
IBVS 수렴(ibvs_done) 후 선택적 하강 + gripper open(release)을 결정론적으로 수행한다.
NN 경로(place_nn_controller)가 준비되면 place_servo.launch.py 에서 이 노드 대신 NN 을 쓴다.

구독:
  /{robot_name}/ibvs_done (Empty)            : IBVS 가 빈자리 bbox 로 수렴 완료
  /{robot_name}/status    (Float64MultiArray): 현재 좌표(하강 목표 계산용)
발행:
  /{robot_name}/target_pose  (Float64MultiArray): [CMD_COORD, x,y,z,rx,ry,rz, speed, mode]
  /{robot_name}/set_gripper  (Float64MultiArray): [open_value, speed] (release)
  /{robot_name}/request_status (Empty)
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64MultiArray


CMD_COORD = 1
STATUS_COORDS_SLICE = slice(20, 26)  # [x,y,z,rx,ry,rz] mm/deg


class PlaceFinalizerNode(Node):

    def __init__(self):
        super().__init__('place_finalizer_node')
        self.declare_parameter('robot_name', 'jetcobot1')
        # 수렴 자세에서 추가 하강량(mm). 선반 표면까지 내려 놓을 만큼 실측 보정.
        # 0 이면 하강 없이 현재 높이에서 release(튜닝 출발점).
        self.declare_parameter('descent_mm', 0.0)
        self.declare_parameter('move_speed', 20)
        self.declare_parameter('descent_settle_sec', 2.0)
        self.declare_parameter('gripper_open_value', 100.0)
        self.declare_parameter('gripper_speed', 50)

        self.robot_name = str(self.get_parameter('robot_name').value)
        self.descent_mm = float(self.get_parameter('descent_mm').value)
        self.move_speed = int(self.get_parameter('move_speed').value)
        self.descent_settle_sec = float(self.get_parameter('descent_settle_sec').value)
        self.gripper_open_value = float(self.get_parameter('gripper_open_value').value)
        self.gripper_speed = int(self.get_parameter('gripper_speed').value)

        ns = f'/{self.robot_name}'
        self._latest_coords = None
        self._done = False

        self._target_pub = self.create_publisher(Float64MultiArray, f'{ns}/target_pose', 1)
        self._gripper_pub = self.create_publisher(Float64MultiArray, f'{ns}/set_gripper', 10)
        self._req_status_pub = self.create_publisher(Empty, f'{ns}/request_status', 10)

        self.create_subscription(
            Float64MultiArray, f'{ns}/status', self._status_cb, 10)
        self.create_subscription(Empty, f'{ns}/ibvs_done', self._ibvs_done_cb, 1)

        self.get_logger().info(
            f'PlaceFinalizerNode 준비 — robot={self.robot_name}, descent={self.descent_mm}mm. '
            'ibvs_done 대기.')

    def _status_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 26:
            self._latest_coords = list(msg.data[STATUS_COORDS_SLICE])

    def _ibvs_done_cb(self, _msg):
        if self._done:
            return
        self._done = True
        self.get_logger().info('ibvs_done 수신 — 배치 종단 시작.')

        if self.descent_mm > 0.0:
            coords = self._fetch_coords()
            if coords is not None:
                target = list(coords)
                target[2] = target[2] - self.descent_mm  # z 하강
                self._publish_coord(target)
                time.sleep(self.descent_settle_sec)
            else:
                self.get_logger().warn('status coords 없음 — 하강 생략하고 release.')

        # release: gripper open.
        self._publish_gripper_open()
        self.get_logger().info('gripper open(release) 발행 — 배치 완료.')

    def _fetch_coords(self, timeout: float = 1.5):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._req_status_pub.publish(Empty())
            time.sleep(0.15)
            if self._latest_coords is not None:
                return self._latest_coords
        return self._latest_coords

    def _publish_coord(self, coords):
        msg = Float64MultiArray()
        # [CMD_COORD, x,y,z,rx,ry,rz, speed, mode]. mode=0(직선/안전 보간은 드라이버 기본).
        msg.data = [float(CMD_COORD)] + [float(c) for c in coords] + [float(self.move_speed), 0.0]
        self._target_pub.publish(msg)

    def _publish_gripper_open(self):
        msg = Float64MultiArray()
        msg.data = [self.gripper_open_value, float(self.gripper_speed)]
        self._gripper_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlaceFinalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
