"""Fire-fighting robot simulation (step 1: robot + world + Arduino flame-sensor behaviour).

    ros2 launch fire_robot_bringup sim.launch.py                      # Gazebo window + everything
    ros2 launch fire_robot_bringup sim.launch.py gui:=false           # headless
    ros2 launch fire_robot_bringup sim.launch.py autonomous:=false    # no Arduino program, drive via /cmd_vel
    ros2 launch fire_robot_bringup sim.launch.py swap_motor_sides:=false   # run fire_sensor.ino's motor pins literally
"""
import math
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def fire_model(f):
    """Placeholder fire (step 1): emissive cones + glow light on a solid pit (the robot cannot drive through the fire).
    Step 2 replaces the cones with textured sprites + particles."""
    s = float(f["intensity"]) ** 0.5
    cone = lambda name, r, h, rgb, z: (
        f'<visual name="{name}"><pose>0 0 {z * s:.3f} 0 0 0</pose><geometry><cone><radius>{r * s:.3f}</radius>'
        f'<length>{h * s:.3f}</length></cone></geometry><material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse>'
        f'<emissive>{rgb} 1</emissive></material></visual>')
    return f'''    <model name="fire_{f["id"]}">
      <static>true</static>
      <pose>{f["x"]} {f["y"]} 0 0 0 0</pose>
      <link name="link">
        <collision name="pit"><pose>0 0 0.025 0 0 0</pose><geometry><cylinder><radius>{0.12 * s:.3f}</radius><length>0.05</length></cylinder></geometry></collision>
        <visual name="scorch"><pose>0 0 0.001 0 0 0</pose><geometry><cylinder><radius>{0.13 * s:.3f}</radius><length>0.002</length></cylinder></geometry>
          <material><diffuse>0.05 0.04 0.03 1</diffuse></material></visual>
        {cone("flame_outer", 0.08, 0.26, "1.0 0.32 0.04", 0.13)}
        {cone("flame_inner", 0.045, 0.17, "1.0 0.80 0.25", 0.085)}
        <light type="point" name="glow"><pose>0 0 {0.15 * s:.3f} 0 0 0</pose><diffuse>1 0.55 0.2 1</diffuse>
          <attenuation><range>2.0</range><constant>0.4</constant><linear>0.3</linear><quadratic>0.5</quadratic></attenuation></light>
      </link>
    </model>'''


SHOT_CAMERA = '''    <model name="shot_camera">
      <static>true</static>
      <pose>-0.55 -1.05 1.2 0 0.62 0.95</pose>
      <link name="link"><sensor name="camera" type="camera">
        <camera><horizontal_fov>1.0</horizontal_fov><image><width>960</width><height>600</height></image><clip><near>0.05</near><far>30</far></clip></camera>
        <always_on>1</always_on><update_rate>5</update_rate><topic>/shot/image</topic></sensor></link>
    </model>'''


def _setup(context):
    lc = context.launch_configurations
    gui = lc["gui"] == "true"
    bringup, desc = share("fire_robot_bringup"), share("fire_robot_description")
    fires_file = lc["fires"] or os.path.join(bringup, "config", "fires.yaml")
    robot_yaml = os.path.join(desc, "config", "robot.yaml")

    with open(fires_file) as f:
        fires = yaml.safe_load(f)["fires"]
    with open(os.path.join(bringup, "worlds", "fire_world.sdf.in")) as f:
        world = f.read()
    world = world.replace("<!--FIRES-->", "\n".join(fire_model(f) for f in fires))
    world = world.replace("<!--SHOT_CAMERA-->", SHOT_CAMERA if lc["shot_camera"] == "true" else "")
    fd, world_path = tempfile.mkstemp(prefix="fire_world_", suffix=".sdf")
    with os.fdopen(fd, "w") as f:
        f.write(world)

    urdf = xacro.process_file(os.path.join(desc, "urdf", "fire_bot.urdf.xacro")).toxml()
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v 2 {world_path}" + ("" if gui else " -s --headless-rendering"),
                          "on_exit_shutdown": "true"}.items())
    sim_time = {"use_sim_time": True}
    nodes = [
        gz,
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf, **sim_time}]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-topic", "robot_description", "-name", "fire_bot", "-z", "0.04"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": os.path.join(bringup, "config", "bridge.yaml"), **sim_time}]),
        Node(package="fire_robot_sim", executable="fire_sensor_sim", output="screen",
             parameters=[{"robot_yaml": robot_yaml, "fires_file": fires_file, **sim_time}]),
        Node(package="fire_robot_sim", executable="sim800l", output="screen", parameters=[sim_time]),
        Node(package="fire_robot_sim", executable="arduino_bot", output="screen",
             condition=IfCondition(LaunchConfiguration("autonomous")),
             parameters=[{"robot_yaml": robot_yaml, "phone": lc["phone"], "wheel_speed": float(lc["wheel_speed"]),
                          "swap_motor_sides": lc["swap_motor_sides"] == "true", **sim_time}]),
    ]
    if lc["shot_camera"] == "true":
        nodes.append(Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                          arguments=["/shot/image@sensor_msgs/msg/Image[gz.msgs.Image"]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo window"),
        DeclareLaunchArgument("autonomous", default_value="true", description="run the ported fire_sensor.ino"),
        DeclareLaunchArgument("fires", default_value="", description="fires yaml (default: config/fires.yaml)"),
        DeclareLaunchArgument("phone", default_value="+910000000000", description="PHONE of fire_sensor.ino"),
        DeclareLaunchArgument("wheel_speed", default_value="0.4", description="m/s of a driven side"),
        DeclareLaunchArgument("swap_motor_sides", default_value="true", description="see docs/design.md"),
        DeclareLaunchArgument("shot_camera", default_value="false", description="fixed camera on /shot/image (docs/tests)"),
        OpaqueFunction(function=_setup),
    ])
