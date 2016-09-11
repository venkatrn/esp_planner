cmake_minimum_required(VERSION 2.8.3)
project(esp_planner)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS sbpl sbpl_utils)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system timer)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES esp_planner
 CATKIN_DEPENDS sbpl sbpl_utils 
 DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(esp_planner
  src/esp_structs.cpp
  src/esp_utils.cpp
  src/edge_selector_ssp.cpp
  src/lao_planner.cpp
  src/esp_mha_planner.cpp
  src/esp_environment.cpp
  src/environments/environment_nav2D.cpp
)
target_link_libraries(esp_planner ${catkin_LIBRARIES} ${Boost_TIMER_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(esp_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(esp_test src/test_esp.cpp)
# add_dependencies(esp_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(esp_test
  esp_planner)

add_executable(2dnav_experiments src/2dnav_experiments.cpp)
# add_dependencies(esp_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(2dnav_experiments
  esp_planner)
