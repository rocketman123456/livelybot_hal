cmake_minimum_required(VERSION 3.0.2)
project(livelybot_hardware)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    livelybot_msg
    livelybot_serial
    serial
)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
)

file(GLOB_RECURSE humanoid_SRCS "src/*/**.cpp")

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp roslib std_msgs livelybot_msg livelybot_serial serial
    # DEPENDS rt pthread
)

add_library(${PROJECT_NAME} ${humanoid_SRCS})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${pinocchio_LIBRARIES}
    Eigen3::Eigen
    rt pthread
)

add_executable(pino_test src/pino_test.cpp)
target_link_libraries(pino_test
    ${catkin_LIBRARIES}
    ${pinocchio_LIBRARIES}
    Eigen3::Eigen
)

add_executable(hardware_node src/hardware_node.cpp)
target_link_libraries(hardware_node
    ${catkin_LIBRARIES}
    ${pinocchio_LIBRARIES}
    ${PROJECT_NAME}
    Eigen3::Eigen
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS pino_test
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS hardware_node
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install headers
install(FILES include/*.h include/*/*.h
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/livelybot_serial
)

install(FILES
    launch/hardware.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
    launch/rviz.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
