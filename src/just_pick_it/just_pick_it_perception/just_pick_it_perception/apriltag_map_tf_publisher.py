import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from transforms3d.euler import euler2quat       # returns [w, x, y, z]
from transforms3d.quaternions import mat2quat   # returns [w, x, y, z]

_DEFAULT_POSES = os.path.join(
    get_package_share_directory('just_pick_it_perception'),
    'config',
    'apriltag_world_poses.yaml',
)


class AprilTagMapTfPublisher(Node):

    def __init__(self):
        super().__init__('apriltag_map_tf_publisher')

        self.declare_parameter('poses_file', _DEFAULT_POSES)
        poses_path = self.get_parameter('poses_file').get_parameter_value().string_value

        data = self._load_poses(poses_path)
        world_frame = data.get('world_frame', 'map')
        tags = data.get('tags', {})

        broadcaster = StaticTransformBroadcaster(self)
        tfs = []
        token_bool = 0

        for tag_id, tag_data in tags.items():
            p = tag_data['pose']

            if 'normal' in tag_data:
                R_mat = self._rotation_from_normal(tag_data['normal'])
                w, qx, qy, qz = mat2quat(R_mat)
            else:
                w, qx, qy, qz = euler2quat(p['R'], p['P'], p['Y'], axes='sxyz')

            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = world_frame
            tf.child_frame_id = f'apriltag_{tag_id}'
            tf.transform.translation.x = float(p['x'])
            tf.transform.translation.y = float(p['y'])
            tf.transform.translation.z = float(p['z'])
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = w
            tfs.append(tf)
            
            if token_bool == 0:
                self.get_logger().info(
                    f'[tag {tag_id}] static TF: {world_frame} -> apriltag_{tag_id} '
                    f'xyz=({p["x"]}, {p["y"]}, {p["z"]}) '
                    f'normal={tag_data.get("normal", "RPY fallback")}'
                )
                token_bool = 1

        broadcaster.sendTransform(tfs)

    @staticmethod
    def _rotation_from_normal(normal: list) -> np.ndarray:
        """
        AprilTag 좌표계 (x right, y up, z toward camera) 기준으로
        normal 벡터(z_tag 방향)와 world up([0,0,1])으로부터 회전행렬을 구성한다.

        R의 열(column): map 프레임에서 표현한 tag frame의 각 축
          col 0 = x_tag (right, 카메라 기준 오른쪽)
          col 1 = y_tag (up)
          col 2 = z_tag (toward camera)
        """
        z = np.array(normal, dtype=np.float64)
        z /= np.linalg.norm(z)

        world_up = np.array([0.0, 0.0, 1.0])
        if abs(np.dot(z, world_up)) > 0.999:
            world_up = np.array([1.0, 0.0, 0.0])

        y = world_up - np.dot(world_up, z) * z
        y /= np.linalg.norm(y)

        # 기존 x 방향이 좌우 반전처럼 보이면 x를 반대로 잡는다
        x = np.cross(z, y)
        x /= np.linalg.norm(x)

        # 오른손 좌표계 유지하려고 y를 다시 계산
        y = np.cross(z, x)
        y /= np.linalg.norm(y)

        return np.column_stack([x, y, z])

    def _load_poses(self, path: str):
        if not os.path.isfile(path):
            raise FileNotFoundError(f'poses_file not found: {path}')
        with open(path, 'r') as f:
            return yaml.safe_load(f)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagMapTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
