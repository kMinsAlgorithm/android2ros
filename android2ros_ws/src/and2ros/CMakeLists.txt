cmake_minimum_required(VERSION 3.0.2)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf tf2 tf2_ros actionlib_msgs)
include(InstallRequiredSystemLibraries)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs actionlib_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(talker src/rn_publisher.cpp)
add_executable(listener src/rn_subscriber.cpp)
add_executable(loc_talker src/tcp_cpp.cpp)
add_executable(base_link_coordinate_listener src/base_link_coordinate_listener.cpp)

target_link_libraries(base_link_coordinate_listener ${catkin_LIBRARIES} ${tf_LIBRARIES})
target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(listener ${catkin_LIBRARIES})
target_link_libraries(loc_talker ${catkin_LIBRARIES})

install(TARGETS talker listener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
