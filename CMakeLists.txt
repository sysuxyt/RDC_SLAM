cmake_minimum_required(VERSION 3.0.0)
project(rdc_slam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  geodesy
  message_generation
)

add_service_files(
  FILES
  OutputMap.srv
  OutputTraj.srv
  OutputSepNodePcd.srv
)

add_message_files(
  FILES 
  cloud_info.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
)

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(libnabo REQUIRED)

find_package(GTSAM REQUIRED QUIET)
find_package(DistributedMapper)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
  ${libnabo_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
	${DistributedMapper_INCLUDE_DIR}
)

# add_library(msgFactory_lib src/msgFactory.cpp)
# target_link_libraries(msgFactory_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam ${DistributedMapper_LIBRARIES})

# add_library(mapOptimization_lib src/mapOptimization.cpp)
# add_dependencies(mapOptimization_lib ${PROJECT_NAME}_gencpp)
# target_link_libraries(mapOptimization_lib gtsam msgFactory_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${libnabo_LIBRARIES})

# add_library(graphComm_lib src/graphComm.cpp)
# target_link_libraries(graphComm_lib ${catkin_LIBRARIES} mapOptimization_lib)

add_executable(imageProjection app/imageProjection.cpp)
add_dependencies(imageProjection ${catkin_EXPORTED_TARGETS})
target_link_libraries(imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(featureAssociation app/featureAssociation.cpp)
add_dependencies(featureAssociation ${catkin_EXPORTED_TARGETS})
target_link_libraries(featureAssociation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(transformFusion app/transformFusion.cpp)
add_dependencies(transformFusion ${catkin_EXPORTED_TARGETS})
target_link_libraries(transformFusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(cslam app/cslam.cpp)
# target_link_libraries(cslam mapOptimization_lib graphComm_lib )
target_link_libraries(cslam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} 
                            ${libnabo_LIBRARIES} ${DistributedMapper_LIBRARIES} ${GTSAM_LIBRARIES} )