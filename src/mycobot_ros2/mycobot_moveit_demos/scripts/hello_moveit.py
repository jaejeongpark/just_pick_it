#!/usr/bin/env python3
"""
MoveIt2를 이용해 mycobot280 arm을 절대좌표(x,y,z,roll,pitch,yaw)로 이동시키는 스크립트.

hello_moveit.cpp의 Python 포팅 버전.
moveit_py 없이 moveit_msgs/action/MoveGroup 액션 클라이언트를 직접 사용한다.

사용법:
  ros2 run mycobot_moveit_demos hello_moveit.py
"""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive


# ── 사용자 설정 영역 ──────────────────────────────────────────────────
# base_link 기준 목표 위치 (미터)
TARGET_X = 0.061
TARGET_Y = -0.176
TARGET_Z = 0.168

# 목표 자세 (라디안). hello_moveit.cpp의 quaternion(1,0,0,0) = roll=π
TARGET_ROLL  = math.pi
TARGET_PITCH = 0.0
TARGET_YAW   = 0.0

# MoveIt 설정
PLANNING_GROUP      = 'arm'
END_EFFECTOR_LINK   = 'gripper_base'   # arm 체인의 tip link
FRAME_ID            = 'base_link'
PLANNER_ID          = 'RRTConnectkConfigDefault'
PIPELINE_ID         = 'ompl'
PLANNING_TIME       = 5.0              # 초
PLANNING_ATTEMPTS   = 10
VEL_SCALE           = 1.0
ACC_SCALE           = 1.0
# ─────────────────────────────────────────────────────────────────────


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
    """RPY(rad) → quaternion(x, y, z, w). 외부 라이브러리 없이 변환."""
    cr, sr = math.cos(roll / 2),  math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2),   math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    )


class HelloMoveitPy(Node):
    def __init__(self):
        super().__init__('hello_moveit_py')
        self._client = ActionClient(self, MoveGroup, '/move_action')

    def move_to_pose(
        self,
        x: float, y: float, z: float,
        roll: float, pitch: float, yaw: float,
    ) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup 액션 서버를 찾을 수 없습니다.')
            return False

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        self.get_logger().info(
            f'목표 위치: pos=({x:.3f}, {y:.3f}, {z:.3f})  '
            f'rpy=({roll:.3f}, {pitch:.3f}, {yaw:.3f})  '
            f'quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})'
        )

        goal_constraints = self._build_pose_constraints(x, y, z, qx, qy, qz, qw)

        request = MotionPlanRequest()
        request.group_name                    = PLANNING_GROUP
        request.num_planning_attempts         = PLANNING_ATTEMPTS
        request.allowed_planning_time         = PLANNING_TIME
        request.max_velocity_scaling_factor   = VEL_SCALE
        request.max_acceleration_scaling_factor = ACC_SCALE
        request.planner_id                    = PLANNER_ID
        request.pipeline_id                   = PIPELINE_ID
        request.goal_constraints              = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request                       = request
        goal.planning_options              = PlanningOptions()
        goal.planning_options.plan_only    = False  # plan + execute
        goal.planning_options.replan       = False

        self.get_logger().info('플래닝 요청 전송 중...')
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal이 MoveGroup에 의해 거부되었습니다.')
            return False

        self.get_logger().info('Goal 수락됨. 실행 결과 대기 중...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        error_val = result_future.result().result.error_code.val
        if error_val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('동작 실행 완료!')
            return True

        self.get_logger().error(f'동작 실패. MoveItErrorCodes: {error_val}')
        return False

    def _build_pose_constraints(
        self, x, y, z, qx, qy, qz, qw
    ) -> Constraints:
        now = self.get_clock().now().to_msg()

        # 위치 제약: 1mm 반지름 구 안에 END_EFFECTOR_LINK 의 원점이 들어와야 함
        target_pose = Pose()
        target_pose.position.x    = x
        target_pose.position.y    = y
        target_pose.position.z    = z
        target_pose.orientation.x = qx
        target_pose.orientation.y = qy
        target_pose.orientation.z = qz
        target_pose.orientation.w = qw

        sphere = SolidPrimitive()
        sphere.type       = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]   # 1 mm 허용 오차

        bv = BoundingVolume()
        bv.primitives       = [sphere]
        bv.primitive_poses  = [target_pose]

        pos = PositionConstraint()
        pos.header.frame_id     = FRAME_ID
        pos.header.stamp        = now
        pos.link_name           = END_EFFECTOR_LINK
        pos.constraint_region   = bv
        pos.weight              = 1.0

        # 자세 제약
        ori = OrientationConstraint()
        ori.header.frame_id              = FRAME_ID
        ori.header.stamp                 = now
        ori.link_name                    = END_EFFECTOR_LINK
        ori.orientation.x                = qx
        ori.orientation.y                = qy
        ori.orientation.z                = qz
        ori.orientation.w                = qw
        ori.absolute_x_axis_tolerance    = 0.01
        ori.absolute_y_axis_tolerance    = 0.01
        ori.absolute_z_axis_tolerance    = 0.01
        ori.weight                       = 1.0

        constraints = Constraints()
        constraints.position_constraints    = [pos]
        constraints.orientation_constraints = [ori]
        return constraints


def main(args=None):
    rclpy.init(args=args)
    node = HelloMoveitPy()

    success = node.move_to_pose(
        TARGET_X, TARGET_Y, TARGET_Z,
        TARGET_ROLL, TARGET_PITCH, TARGET_YAW,
    )

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
