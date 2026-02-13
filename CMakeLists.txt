cmake_minimum_required(VERSION 3.0.2)
project(icir_tutorial_pinocchio)

add_compile_options(-std=c++17 -Ofast)
add_definitions(-D_OS_UNIX)

INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ide.cmake)
INCLUDE(cmake/apple.cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  geometry_msgs
  mujoco_ros
  hardware_interface
  dynamic_reconfigure    
  pluginlib
  realtime_tools    
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp  
    geometry_msgs
    mujoco_ros    
    hardware_interface
    dynamic_reconfigure
    pluginlib
    realtime_tools
    controller_interface                
  #DEPENDS system_lib
)

###########
## Build ##
###########
link_directories(${Boost_LIBRARY_DIR})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

### library
SET(${PROJECT_NAME}_HEADERS   
  utility/urdf_to_pin.hpp
  utility/math_functions.hpp
)
SET(${PROJECT_NAME}_SOURCES   
  utility/urdf_to_pin.cpp
  utility/math_functions.cpp
)

ADD_SOURCE_GROUP(${PROJECT_NAME}_HEADERS)
ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC pinocchio::pinocchio ${pinocchio_LIBRARIES} -lpthread)

### exe 
add_executable(${PROJECT_NAME}_simulation src/icir_tutorial_pinocchio_sim.cpp)
target_link_libraries(${PROJECT_NAME}_simulation ${PROJECT_NAME} ${catkin_LIBRARIES} )
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}_simulation SYSTEM PUBLIC $<INSTALL_INTERFACE:include>)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


