from setuptools import find_packages, setup

package_name = "fire_robot_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="S. Mahadev",
    maintainer_email="mrsssok222@gmail.com",
    description="Fire robot simulation nodes.",
    license="MIT",
    entry_points={"console_scripts": [
        "fire_sensor_sim = fire_robot_sim.fire_sensor_sim:main",
        "arduino_bot = fire_robot_sim.arduino_bot:main",
        "sim800l = fire_robot_sim.sim800l_node:main",
        "fire_manager = fire_robot_sim.fire_manager:main",
        "water_system = fire_robot_sim.water_system:main",
        "cnn_detector = fire_robot_sim.cnn_detector:main",
        "fire_monitor = fire_robot_sim.fire_monitor:main",
        "mission = fire_robot_sim.mission_node:main",
    ]},
)
