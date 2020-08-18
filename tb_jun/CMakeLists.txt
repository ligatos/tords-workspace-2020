cmake_minimum_required(VERSION 2.8.3)
project(tb_jun)


find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  #serial
  move_base_msgs
  frontier_exploration
  visualization_msgs
  message_generation
  dynamic_reconfigure
  diagnostic_updater
  octomap_ros
  octomap_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
  dynamixel_msgs
  tb_msgsrv
)


include_directories(
include
${catkin_INCLUDE_DIRS}
)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

#OCTOMAP
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

#EDT
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_libraries(${DYNAMICEDT3D_LIBRARIES})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
#INCLUDE_DIRS include
 LIBRARIES tb_mai
 )
 add_executable(tb_odom_node src/odom.cpp)
 target_link_libraries(tb_odom_node
     ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
add_executable(tb_targeter_node src/targeter.cpp)
target_link_libraries(tb_targeter_node
    ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES})
add_executable(tb_cmdaltarm_node src/cmdaltarm.cpp)
target_link_libraries(tb_cmdaltarm_node
     ${catkin_LIBRARIES}
	${OpenCV_LIBRARIES})
add_executable(tb_fsm_node src/fsm.cpp)
target_link_libraries(tb_fsm_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})
  add_executable(tb_cmdmb_node src/cmdmb.cpp)
  target_link_libraries(tb_cmdmb_node ${catkin_LIBRARIES} )
add_executable(tb_cmdodomtf_node src/cmdodomtf.cpp)
target_link_libraries(tb_cmdodomtf_node ${catkin_LIBRARIES} )
add_executable(tb_mbmapper_node src/mbmapper.cpp)
target_link_libraries(tb_mbmapper_node
     ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
add_executable(tb_pathclusters_node src/pathclusters.cpp)
target_link_libraries(tb_pathclusters_node
     ${catkin_LIBRARIES}
     ${OpenCV_LIBRARIES})
 add_executable(tb_scanproc_node src/scanproc.cpp)
 target_link_libraries(tb_scanproc_node
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES})
 add_executable(tb_draw_node src/draw.cpp)
 target_link_libraries(tb_draw_node
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES})
add_executable(tb_assembly_node src/assembly.cpp)
target_link_libraries(tb_assembly_node
    ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})
