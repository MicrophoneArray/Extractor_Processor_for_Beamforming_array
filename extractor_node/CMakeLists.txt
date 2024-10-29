cmake_minimum_required(VERSION 3.8)
project(extractor_node)

# find the package we want to use 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cae_microphone_array REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(message_filters REQUIRED)

# generate the messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AvReader.msg"
  "msg/AvReaderCom.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# include the directories
include_directories(include ${PROJECT_NAME}/include)

# add the executable files
# add_executable(extractor src/extractor.cpp)
add_executable(extractor src/extractor_with_time_synchronizer.cpp)
add_executable(extractor_car src/extractor_on_the_car.cpp)

# link the libraries to the files
ament_target_dependencies(extractor rclcpp std_msgs sensor_msgs cae_microphone_array message_filters)
ament_target_dependencies(extractor_car  rclcpp std_msgs sensor_msgs cae_microphone_array message_filters)

install(TARGETS
  extractor
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_target_interfaces(extractor 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  extractor_car
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_target_interfaces(extractor_car 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(DIRECTORY
  msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()
