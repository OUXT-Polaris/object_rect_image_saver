cmake_minimum_required(VERSION 2.8.3)
project(object_rect_image_saver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  jsk_recognition_msgs
  message_filters
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES object_rect_image_saver
  CATKIN_DEPENDS cv_bridge image_transport jsk_recognition_msgs message_filters roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_rect_image_saver_node src/object_rect_image_saver_node.cpp src/object_rect_image_saver.cpp)
target_link_libraries(object_rect_image_saver_node ${catkin_LIBRARIES})

# install executables
install(TARGETS object_rect_image_saver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install headers
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install launch files
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)

set(PYTHON_SCRIPTS 
  scripts/generate_dataset.py
)
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(scripts)