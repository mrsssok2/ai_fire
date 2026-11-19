cmake_minimum_required(VERSION 3.8)
project(fire_robot_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch worlds config DESTINATION share/${PROJECT_NAME})
# textures, particle ramps and the CNN live at the ros2_sim/ top level
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../assets/flames ${CMAKE_CURRENT_SOURCE_DIR}/../../assets/particles
        DESTINATION share/${PROJECT_NAME}/assets)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../models DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.onnx")
ament_package()
