cmake_minimum_required(VERSION 2.8.3)
project(video_grabber)

# Debug mode (Be careful. Some process become extremely slow in debug mode.)
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
# set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        cv_bridge
        image_transport
        )

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS roscpp sensor_msgs image_transport
        DEPENDS cv_bridge
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_library(video_grabber nodes/video_grabber.cpp)
target_link_libraries(video_grabber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(video_grabber ${catkin_EXPORTED_TARGETS})

add_executable(video_grabber_node nodes/video_grabber_node.cpp)
target_link_libraries(video_grabber_node ${catkin_LIBRARIES} video_grabber)
add_dependencies(video_grabber_node ${catkin_EXPORTED_TARGETS})

install(TARGETS video_grabber_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
