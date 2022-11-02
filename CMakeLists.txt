cmake_minimum_required(VERSION 2.8.3)

project(pycoral_ros)

find_package(
    catkin REQUIRED COMPONENTS
    message_generation 
    image_proc
    image_view
    rospy 
    roscpp 
    std_msgs 
    sensor_msgs 
    vision_msgs
    cv_bridge
    pcl_conversions
    pcl_ros
    tf2
    tf2_ros
    tf2_msgs
    tf2_sensor_msgs
)
find_package(Boost REQUIRED python3)

add_service_files(
    FILES
    DetectImage.srv
    DetectCompressedImage.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    vision_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES add_detection
    CATKIN_DEPENDS pcl_ros pcl_conversions cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
)

catkin_install_python(PROGRAMS scripts/detect_coral
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/topic2video.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(add_detection src/add_detection.cpp)
target_link_libraries(add_detection ${catkin_LIBRARIES})

