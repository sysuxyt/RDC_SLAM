cmake_minimum_required(VERSION 2.8.3)
project(ping_msg)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   message_generation
)

add_message_files(
  FILES
  Ping.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)
catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS message_runtime
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

link_directories(
	include)

add_executable(ping src/ping.cpp)
add_dependencies(ping ${catkin_EXPORTED_TARGETS})
target_link_libraries(ping ${catkin_LIBRARIES})

install(TARGETS ping
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
