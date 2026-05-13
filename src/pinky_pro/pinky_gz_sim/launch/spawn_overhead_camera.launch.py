import math
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn(context, *args, **kwargs):
    height      = float(LaunchConfiguration('overhead_height').perform(context))
    fov_deg     = float(LaunchConfiguration('overhead_fov').perform(context))
    img_width   = int(LaunchConfiguration('overhead_img_width').perform(context))
    img_height  = int(LaunchConfiguration('overhead_img_height').perform(context))
    tilt_deg    = float(LaunchConfiguration('overhead_tilt').perform(context))

    # gz-sim (Ogre2) native convention: camera sensor looks along the link's -Z axis.
    # (Unlike URDF-sourced cameras where ros_gz_sim auto-applies a frame conversion.)
    #
    # To aim straight down (world -Z):  link -Z = world -Z  →  pitch = 0
    # To tilt toward world +X by θ:     pitch = -θ  (negative pitch rotates -Z toward +X)
    pitch_rad = -math.radians(tilt_deg)
    fov_rad   = math.radians(fov_deg)

    # Camera body: 80 mm(x) × 120 mm(y) × 80 mm(z)
    # Lens disc: on the -Z face (the face that points toward the arena floor).
    #   Cylinder default axis = link Z.  No rotation needed; the circular end-face
    #   is visible from the -Z direction (i.e., from below = from the arena).
    #
    # <inertial> is required even for static models: without it gz-sim's ECS
    # does not fully register the link and SceneBroadcaster skips its visuals.
    #
    # Pose is intentionally omitted from the model element; the spawn position
    # is passed explicitly via CLI args to ros_gz_sim create so it is always applied.
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="overhead_camera">
    <static>true</static>
    <link name="camera_link">

      <!-- Required for gz-sim ECS visual registration even on static models -->
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>1e-6</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-6</iyy><iyz>0</iyz>
          <izz>1e-6</izz>
        </inertia>
      </inertial>

      <!-- Camera body (visual only, no collision) -->
      <visual name="camera_body">
        <geometry>
          <box><size>0.080 0.120 0.080</size></box>
        </geometry>
        <material>
          <ambient>0.20 0.20 0.20 1</ambient>
          <diffuse>0.30 0.30 0.30 1</diffuse>
          <specular>0.40 0.40 0.40 1</specular>
        </material>
      </visual>

      <!-- Lens disc on the -Z face (faces the arena floor).
           Cylinder axis = link Z (default); circular end-face visible from below. -->
      <visual name="camera_lens">
        <pose>0 0 -0.043 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.024</radius>
            <length>0.006</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.00 0.25 0.80 1</ambient>
          <diffuse>0.00 0.35 1.00 1</diffuse>
          <specular>0.50 0.70 1.00 1</specular>
        </material>
      </visual>

      <!-- gz-sim native camera renders along sensor-frame +X.
           Rotating the sensor by pitch=pi/2 maps its +X onto the link's -Z,
           so the rendering direction matches the lens visual on the -Z face. -->
      <sensor name="overhead_camera" type="camera">
        <pose>0 0 0 0 1.5708 0</pose>
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <camera>
          <horizontal_fov>{fov_rad:.6f}</horizontal_fov>
          <image>
            <width>{img_width}</width>
            <height>{img_height}</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>20.0</far>
          </clip>
        </camera>
        <topic>overhead_camera/image_raw</topic>
        <gz_frame_id>overhead_camera_link</gz_frame_id>
      </sensor>

    </link>
  </model>
</sdf>"""

    # Write to a temp file and use -file to avoid multi-line string parsing issues.
    # Pose is passed as explicit CLI args; ros_gz_sim create CLI args always win
    # over any pose embedded in the SDF when both are present.
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.sdf', prefix='overhead_camera_', delete=False)
    tmp.write(sdf)
    tmp.close()

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_overhead_camera',
        output='screen',
        arguments=[
            '-file', tmp.name,
            '-x', '0', '-y', '0', '-z', f'{height:.4f}',
            '-R', '0', '-P', f'{pitch_rad:.6f}', '-Y', '0',
        ],
        parameters=[{'use_sim_time': True}],
    )

    bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='overhead_image_bridge',
        output='screen',
        arguments=['/overhead_camera/image_raw'],
    )

    return [spawn_node, bridge_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'overhead_height', default_value='1.5',
            description='Camera height above arena floor [m]'),
        DeclareLaunchArgument(
            'overhead_fov', default_value='75.0',
            description='Horizontal field of view [deg]  '
                        '(75 deg @ 1.5 m covers ~2.2 m × 1.6 m, arena 1.5 m × 1.0 m with margin)'),
        DeclareLaunchArgument(
            'overhead_img_width', default_value='640',
            description='Image width [px]'),
        DeclareLaunchArgument(
            'overhead_img_height', default_value='480',
            description='Image height [px]'),
        DeclareLaunchArgument(
            'overhead_tilt', default_value='0.0',
            description='Tilt from nadir (straight-down) toward +X [deg]  '
                        '0 = looking straight down, 90 = looking horizontal'),
        OpaqueFunction(function=_spawn),
    ])
