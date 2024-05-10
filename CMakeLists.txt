cmake_minimum_required(VERSION 3.0.2)
project(livelybot_hardware)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    livelybot_msg
    livelybot_serial
    modern_robotics_cpp
    serial
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(humanoid_SRCS src/*/*.cpp)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    DEPENDS rt pthread
    CATKIN_DEPENDS roscpp std_msgs livelybot_msg livelybot_serial modern_robotics_cpp serial
)

add_library(${PROJECT_NAME} ${humanoid_SRCS})
target_link_libraries(${PROJECT_NAME} rt pthread)

add_executable(test_motion src/test_motor.cpp ${humanoid_SRCS})
target_link_libraries(test_motion ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install headers
install(FILES include/*.h include/*/*.h
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/livelybot_serial
)
