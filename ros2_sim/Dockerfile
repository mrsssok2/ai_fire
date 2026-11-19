# Built on the existing gurdy2/ros:jazzy image (ROS 2 Jazzy, Gazebo Harmonic via ros_gz, ros2_control, RViz,
# numpy, OpenCV, onnxruntime, PyYAML): nothing extra to download. Runs as user `ubuntu` (UID 1000).
FROM gurdy2/ros:jazzy
COPY --chmod=755 docker/entrypoint.sh /fire_entrypoint.sh
WORKDIR /home/ubuntu/ros2_sim
ENTRYPOINT ["/fire_entrypoint.sh"]
CMD ["bash"]
