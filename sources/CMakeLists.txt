cmake_minimum_required(VERSION 3.0.0)
project(robodog_v2 VERSION 2.0.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

#add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

find_package(Boost COMPONENTS filesystem thread)

include(CTest)
enable_testing()

catkin_package()

include_directories(~/catkin_ws/src/serial/include "${PROJECT_SOURCE_DIR}/Include")
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(~/catkin_ws/devel/lib)

#message(STATUS "Include: " ${INCLUDE_DIRECTORIES})

set(DOG_LIB_SOURCES
    IK.cpp
    Leg.cpp
    LegV2.cpp
    Trajectory/GCodeController.cpp
    Trajectory/MovePrimitives.cpp
    Trajectory/MoveParams.cpp
    Trajectory/RobotState.cpp
    LX16a.cpp Utils.cpp)

# robodog_lib
add_library(${PROJECT_NAME}_lib STATIC ${DOG_LIB_SOURCES})

# robodog_gcode_service
add_executable(${PROJECT_NAME}_hw
  Services/RobotHW.cpp)
target_link_libraries(${PROJECT_NAME}_hw
    ${PROJECT_NAME}_lib
    libserial.so
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_hw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
