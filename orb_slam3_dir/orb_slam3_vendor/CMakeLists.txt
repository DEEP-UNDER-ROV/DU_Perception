cmake_minimum_required(VERSION 3.16)
project(orb_slam3_vendor LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

get_filename_component(_orb_root "${CMAKE_CURRENT_SOURCE_DIR}/../ORB-SLAM3" REALPATH)
set(ORB_SLAM3_SOURCE_DIR "${_orb_root}" CACHE PATH "ORB-SLAM3 source root")
set(THIRDPARTY_DIR ${ORB_SLAM3_SOURCE_DIR}/Thirdparty)
set(INSTALL_INCLUDE_DIR include/${PROJECT_NAME})

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)

file(GLOB DBOW2_SOURCES
  ${THIRDPARTY_DIR}/DBoW2/DBoW2/*.cpp
  ${THIRDPARTY_DIR}/DBoW2/DUtils/*.cpp)
add_library(DBoW2 SHARED ${DBOW2_SOURCES})
set_target_properties(DBoW2 PROPERTIES OUTPUT_NAME orb_slam3_dbow2)
target_include_directories(DBoW2 PUBLIC
  $<BUILD_INTERFACE:${THIRDPARTY_DIR}/DBoW2>
  $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}/Thirdparty/DBoW2>)
target_link_libraries(DBoW2 PUBLIC ${OpenCV_LIBS})

configure_file(${THIRDPARTY_DIR}/g2o/config.h.in ${THIRDPARTY_DIR}/g2o/config.h)
file(GLOB G2O_SOURCES
  ${THIRDPARTY_DIR}/g2o/g2o/core/*.cpp
  ${THIRDPARTY_DIR}/g2o/g2o/types/*.cpp
  ${THIRDPARTY_DIR}/g2o/g2o/stuff/*.cpp
  ${THIRDPARTY_DIR}/g2o/g2o/stuff/*.c)
add_library(g2o SHARED ${G2O_SOURCES})
set_target_properties(g2o PROPERTIES OUTPUT_NAME orb_slam3_g2o)
target_include_directories(g2o PUBLIC
  $<BUILD_INTERFACE:${THIRDPARTY_DIR}/g2o>
  $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}/Thirdparty/g2o>)
target_link_libraries(g2o PUBLIC Eigen3::Eigen)

file(GLOB ORB_SLAM3_SOURCES
  ${ORB_SLAM3_SOURCE_DIR}/src/*.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/*.cpp
  ${ORB_SLAM3_SOURCE_DIR}/src/CameraModels/*.cpp)
add_library(ORB_SLAM3 SHARED ${ORB_SLAM3_SOURCES})
target_compile_definitions(ORB_SLAM3 PUBLIC COMPILEDWITHC11)
target_include_directories(ORB_SLAM3 PUBLIC
  $<BUILD_INTERFACE:${ORB_SLAM3_SOURCE_DIR}>
  $<BUILD_INTERFACE:${ORB_SLAM3_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${ORB_SLAM3_SOURCE_DIR}/include/CameraModels>
  $<BUILD_INTERFACE:${THIRDPARTY_DIR}/Sophus>
  $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}/include>
  $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}/include/CameraModels>
  $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}/Thirdparty/Sophus>)
target_link_libraries(ORB_SLAM3
  PUBLIC DBoW2 g2o Eigen3::Eigen ${OpenCV_LIBS} ${Pangolin_LIBRARIES}
  PRIVATE Boost::serialization OpenSSL::Crypto stdc++fs)
target_link_options(ORB_SLAM3 PRIVATE -Wl,--no-undefined)

install(DIRECTORY ${ORB_SLAM3_SOURCE_DIR}/include
  DESTINATION ${INSTALL_INCLUDE_DIR})
install(DIRECTORY ${THIRDPARTY_DIR}
  DESTINATION ${INSTALL_INCLUDE_DIR}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN "build" EXCLUDE
  PATTERN "lib" EXCLUDE)

install(TARGETS DBoW2 g2o ORB_SLAM3
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 OpenCV Pangolin)
ament_package()
