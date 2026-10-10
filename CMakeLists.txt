cmake_minimum_required(VERSION 3.16)
project(orbslam3_dense_ros2 VERSION 1.5.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d highgui)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)

add_subdirectory(orb_slam3/Thirdparty/g2o)
add_subdirectory(orb_slam3/Thirdparty/DBoW2)

add_library(orb_slam3_lib SHARED
  orb_slam3/src/System.cc
  orb_slam3/src/Tracking.cc
  orb_slam3/src/LocalMapping.cc
  orb_slam3/src/LoopClosing.cc
  orb_slam3/src/ORBextractor.cc
  orb_slam3/src/AgastCornerDetector.cc
  orb_slam3/src/ORBmatcher.cc
  orb_slam3/src/FrameDrawer.cc
  orb_slam3/src/Converter.cc
  orb_slam3/src/MapPoint.cc
  orb_slam3/src/KeyFrame.cc
  orb_slam3/src/Atlas.cc
  orb_slam3/src/Map.cc
  orb_slam3/src/MapDrawer.cc
  orb_slam3/src/Optimizer.cc
  orb_slam3/src/Frame.cc
  orb_slam3/src/KeyFrameDatabase.cc
  orb_slam3/src/Sim3Solver.cc
  orb_slam3/src/Viewer.cc
  orb_slam3/src/ImuTypes.cc
  orb_slam3/src/G2oTypes.cc
  orb_slam3/src/CameraModels/Pinhole.cpp
  orb_slam3/src/CameraModels/KannalaBrandt8.cpp
  orb_slam3/src/OptimizableTypes.cpp
  orb_slam3/src/MLPnPsolver.cpp
  orb_slam3/src/GeometricTools.cc
  orb_slam3/src/TwoViewReconstruction.cc
  orb_slam3/src/Config.cc
  orb_slam3/src/Settings.cc
  orb_slam3/src/SlamDataProcess.cc
  orb_slam3/src/PointClouds.cc
)
target_include_directories(orb_slam3_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3
  ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/include
  ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/include/CameraModels
  ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/Thirdparty
  ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(orb_slam3_lib PUBLIC
  DBoW2 g2o Eigen3::Eigen ${OpenCV_LIBS} ${PCL_LIBRARIES}
  ${Pangolin_LIBRARIES} Boost::serialization OpenSSL::Crypto Threads::Threads
)

add_library(ros2_slam_publisher_lib SHARED src/ros2_slam_publisher.cpp)
target_include_directories(ros2_slam_publisher_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(ros2_slam_publisher_lib
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs cv_bridge image_transport tf2_ros pcl_conversions
)
target_link_libraries(ros2_slam_publisher_lib orb_slam3_lib)

# Capture build-time provenance without requiring Git in the deployed environment.
find_package(Git QUIET)
set(ORB_SLAM3_BUILD_REVISION "unknown")
if(GIT_FOUND)
  execute_process(COMMAND ${GIT_EXECUTABLE} describe --always --dirty --broken
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE ORB_SLAM3_GIT_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET
    RESULT_VARIABLE ORB_SLAM3_GIT_RESULT)
  if(ORB_SLAM3_GIT_RESULT EQUAL 0)
    set(ORB_SLAM3_BUILD_REVISION "${ORB_SLAM3_GIT_REVISION}")
  endif()
endif()
# A source fingerprint also identifies working trees with no Git commits/untracked files.
file(GLOB_RECURSE ORB_SLAM3_PROVENANCE_FILES CONFIGURE_DEPENDS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/*.cpp" "include/*.h" "orb_slam3/src/*.cc" "orb_slam3/src/*.cpp"
  "orb_slam3/include/*.h" "orb_slam3/Thirdparty/*.h" "orb_slam3/Thirdparty/*.hpp"
  "orb_slam3/Thirdparty/*.cpp" "orb_slam3/Thirdparty/CMakeLists.txt"
  "launch/*.py" "scripts/*.py")
list(APPEND ORB_SLAM3_PROVENANCE_FILES CMakeLists.txt package.xml)
list(SORT ORB_SLAM3_PROVENANCE_FILES)
set(ORB_SLAM3_SOURCE_MANIFEST "")
foreach(ORB_SLAM3_SOURCE_FILE IN LISTS ORB_SLAM3_PROVENANCE_FILES)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/${ORB_SLAM3_SOURCE_FILE}")
  file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/${ORB_SLAM3_SOURCE_FILE}" ORB_SLAM3_FILE_SHA256)
  string(APPEND ORB_SLAM3_SOURCE_MANIFEST "${ORB_SLAM3_SOURCE_FILE}:${ORB_SLAM3_FILE_SHA256}\n")
endforeach()
string(SHA256 ORB_SLAM3_SOURCE_SHA256 "${ORB_SLAM3_SOURCE_MANIFEST}")
add_executable(orb_slam3_main src/main.cpp)
target_compile_definitions(orb_slam3_main PRIVATE
  ORBSLAM3_BUILD_REVISION="${ORB_SLAM3_BUILD_REVISION}"
  ORBSLAM3_SOURCE_SHA256="${ORB_SLAM3_SOURCE_SHA256}")
ament_target_dependencies(orb_slam3_main ament_index_cpp rclcpp sensor_msgs cv_bridge message_filters)
target_link_libraries(orb_slam3_main ros2_slam_publisher_lib)

# Resolve bundled libraries from the install tree, including after relocation.
set_target_properties(orb_slam3_lib ros2_slam_publisher_lib DBoW2 g2o
  PROPERTIES INSTALL_RPATH "$ORIGIN" INSTALL_RPATH_USE_LINK_PATH TRUE)
set_target_properties(orb_slam3_main PROPERTIES
  INSTALL_RPATH "$ORIGIN/.." INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS orb_slam3_main orb_slam3_lib ros2_slam_publisher_lib DBoW2 g2o
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/kitti_player.py DESTINATION lib/${PROJECT_NAME} RENAME kitti_player)
install(DIRECTORY orb_slam3/config orb_slam3/Vocabulary DESTINATION share/${PROJECT_NAME})
install(FILES README.md README_CN.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(build_environment test/test_build_env.py TIMEOUT 60)
  ament_add_pytest_test(rgbd_smoke test/test_rgbd_smoke.py TIMEOUT 180
    ENV "ORB_SLAM3_EXECUTABLE=$<TARGET_FILE:orb_slam3_main>"
      "ORB_SLAM3_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_pytest_test(kitti_player test/test_kitti_player.py TIMEOUT 180
    ENV "ORB_SLAM3_EXECUTABLE=$<TARGET_FILE:orb_slam3_main>")
endif()

ament_package()
