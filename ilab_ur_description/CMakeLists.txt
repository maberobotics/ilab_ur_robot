cmake_minimum_required(VERSION 3.5)
project(ilab_ur_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config rviz urdf srdf moveit2 meshes gazebo
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
