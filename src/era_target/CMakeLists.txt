cmake_minimum_required(VERSION 3.0.2)
project(era_target)
add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  message_generation
  sensor_msgs
  std_msgs
)

find_package( OpenCV REQUIRED )
find_package( Eigen3 3.3 REQUIRED NO_MODULE)

add_message_files(
  DIRECTORY msg
  FILES
  era_camera.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
#  INCLUDE_DIRS include
#  LIBRARIES camrcv
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable( era_target_recognizer src/main.cpp src/circle_calculating.cpp src/era_target_recogn.cpp)
add_dependencies(era_target_recognizer ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries ( era_target_recognizer ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} Eigen3::Eigen)
