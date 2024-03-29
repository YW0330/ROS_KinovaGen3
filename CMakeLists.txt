cmake_minimum_required(VERSION 3.0.2)
project(kinova_test)

# Activate C++ 17
set (CMAKE_CXX_STANDARD 17)
add_definitions(-D_OS_UNIX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# 自定義 msg file
add_message_files(FILES
  kinovaMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
    message_runtime)

## 加入標頭檔
# include_directories 官方文件 https://cmake.org/cmake/help/latest/command/include_directories.html
# (推薦) target_include_directories 官方文件 https://cmake.org/cmake/help/latest/command/target_include_directories.html
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/include/client
  ${PROJECT_SOURCE_DIR}/include/common
  ${PROJECT_SOURCE_DIR}/include/messages
  ${PROJECT_SOURCE_DIR}/include/client_stubs
  ${PROJECT_SOURCE_DIR}/include/cxxopts
  ${catkin_INCLUDE_DIRS}
)

# 建立mylib函式庫
add_library(mylibary "")
target_sources(mylibary
  PRIVATE
  ${PROJECT_SOURCE_DIR}/src/Matrix.cpp
  ${PROJECT_SOURCE_DIR}/src/KinovaConfig.cpp
  ${PROJECT_SOURCE_DIR}/src/KinovaGen3Model.cpp
  ${PROJECT_SOURCE_DIR}/src/controller.cpp
  ${PROJECT_SOURCE_DIR}/src/conio.c
  ${PROJECT_SOURCE_DIR}/src/HumanState.cpp
)

## 連接函式庫
# include_directories 官方文件 https://cmake.org/cmake/help/latest/command/link_libraries.html
# (推薦) target_link_libraries 官方文件 https://cmake.org/cmake/help/latest/command/target_link_libraries.html#command:target_link_libraries
link_libraries(
  mylibary
  ${PROJECT_SOURCE_DIR}/lib/libKinovaGen3Model.a
  ${PROJECT_SOURCE_DIR}/lib/libkinova_FK_axisAngle.a
  ${PROJECT_SOURCE_DIR}/lib/release/libKortexApiCpp.a
  ${catkin_LIBRARIES}
)

# 生成執行檔
# ---------------- gravity_compensation ------------------
add_executable(gravity_compensation src/gravity_compensation.cpp src/utilities.cpp)

# ---------------- chang_tracking ------------------
add_executable(chang_tracking src/chang_tracking.cpp src/utilities.cpp)

# ---------------- following_human_control ------------------
add_executable(following_human_control src/following_human_control.cpp src/utilities.cpp)

# ---------------- following_human_manipulator ------------------
add_executable(following_human_manipulator src/following_human_manipulator.cpp src/utilities.cpp)