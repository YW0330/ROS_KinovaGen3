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
add_library(mylib "")
target_sources(mylib
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src/mylib.cpp
)
target_include_directories(mylib PRIVATE
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)


# 生成執行檔
add_executable(kinova_torque src/kinova_torque.cpp src/utilities.cpp)

# 執行檔加入標頭檔
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

# 執行檔連接函式庫
target_link_libraries(kinova_torque
  mylib
  ${PROJECT_SOURCE_DIR}/lib/KinovaGen3Model.a
  ${PROJECT_SOURCE_DIR}/lib/release/libKortexApiCpp.a
  ${catkin_LIBRARIES}
)

# ------------------------------------------------------
add_executable(${PROJECT_NAME}  src/${PROJECT_NAME}.cpp src/utilities.cpp)

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

target_link_libraries(${PROJECT_NAME} 
  ${PROJECT_SOURCE_DIR}/lib/release/libKortexApiCpp.a
  ${catkin_LIBRARIES}
)