cmake_minimum_required(VERSION 3.0.2)
project(kinova_test)

# Activate C++ 11
set (CMAKE_CXX_STANDARD 11)
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

# 建立mylib函式庫
add_library(mylibary "")
target_include_directories(mylibary PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/include/client
  ${PROJECT_SOURCE_DIR}/include/common
  ${PROJECT_SOURCE_DIR}/include/messages
  ${PROJECT_SOURCE_DIR}/include/client_stubs
  ${PROJECT_SOURCE_DIR}/include/cxxopts
)
target_sources(mylibary
  PRIVATE
  ${PROJECT_SOURCE_DIR}/src/Matrix.cpp
  ${PROJECT_SOURCE_DIR}/src/mylib.cpp
)


# 生成執行檔
add_executable(${PROJECT_NAME}  src/${PROJECT_NAME}.cpp src/utilities.cpp)

# 加入標頭檔
target_include_directories(${PROJECT_NAME}  PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/include/client
  ${PROJECT_SOURCE_DIR}/include/common
  ${PROJECT_SOURCE_DIR}/include/messages
  ${PROJECT_SOURCE_DIR}/include/client_stubs
  ${PROJECT_SOURCE_DIR}/include/cxxopts
  ${catkin_INCLUDE_DIRS}
)

# 執行檔連接函式庫
target_link_libraries(${PROJECT_NAME} 
  ${PROJECT_SOURCE_DIR}/lib/release/libKortexApiCpp.a
  ${catkin_LIBRARIES}
)

# ---------------- 官方 kinova_torque 範例 ------------------
add_executable(kinova_torque src/kinova_torque.cpp src/utilities.cpp)

target_include_directories(kinova_torque PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/include/client
  ${PROJECT_SOURCE_DIR}/include/common
  ${PROJECT_SOURCE_DIR}/include/messages
  ${PROJECT_SOURCE_DIR}/include/client_stubs
  ${PROJECT_SOURCE_DIR}/include/cxxopts
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(kinova_torque
  mylibary
  ${PROJECT_SOURCE_DIR}/lib/libKinovaGen3Model.a
  ${PROJECT_SOURCE_DIR}/lib/release/libKortexApiCpp.a
  ${catkin_LIBRARIES}
)

# ---------------- 重力補償 ------------------
add_executable(gravity_compensation src/gravity_compensation.cpp src/utilities.cpp)

target_include_directories(gravity_compensation PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/include/client
  ${PROJECT_SOURCE_DIR}/include/common
  ${PROJECT_SOURCE_DIR}/include/messages
  ${PROJECT_SOURCE_DIR}/include/client_stubs
  ${PROJECT_SOURCE_DIR}/include/cxxopts
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(gravity_compensation
  mylibary
  ${PROJECT_SOURCE_DIR}/lib/libKinovaGen3Model.a
  ${PROJECT_SOURCE_DIR}/lib/release/libKortexApiCpp.a
  ${catkin_LIBRARIES}
)


# ---------------- Matlab ------------------
add_executable(matlab src/matlab.cpp)

target_include_directories(matlab PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/include/client
  ${PROJECT_SOURCE_DIR}/include/common
  ${PROJECT_SOURCE_DIR}/include/messages
  ${PROJECT_SOURCE_DIR}/include/client_stubs
  ${PROJECT_SOURCE_DIR}/include/cxxopts
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(matlab
  mylibary
  ${PROJECT_SOURCE_DIR}/lib/libKinovaGen3Model.a
  ${PROJECT_SOURCE_DIR}/lib/release/libKortexApiCpp.a
  ${catkin_LIBRARIES}
)