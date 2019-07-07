cmake_minimum_required(VERSION 2.8.3)
project(data_gen)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        roscpp
        rospy
        std_msgs
        pcl_conversions
        image_transport
        )
        


catkin_package(
        # INCLUDE_DIRS include
        LIBRARIES data_gen
        CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions
        #DEPENDS system_lib
)
find_package(OpenCV)
find_package(PCL 1.5)
message(WARNING "PCL 1.5 found")


if (NOT ${PCL_FOUND})
  message(WARNING "PCL 1.8 not found, attempting 1.7...")
  find_package(PCL 1.7 REQUIRED)
endif()

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})


###########
## Build ##
###########
include_directories(
        ${catkin_INCLUDE_DIRS}
        # ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS})

set(kitti2pcd_SRC
        src/kitti2pcd.cpp
        )


add_executable(kitti2pcd ${kitti2pcd_SRC})
target_link_libraries(kitti2pcd ${catkin_LIBRARIES} 
                                ${PCL_LIBRARIES} 
                                ${OpenCV_LIBRARIES})


set(imagetest_SRC
        src/image.cpp
        )


add_executable(imagetest ${imagetest_SRC})
target_link_libraries(imagetest ${catkin_LIBRARIES} 
                                ${PCL_LIBRARIES} 
                                ${OpenCV_LIBRARIES})

add_executable(pcb2kitti src/pcb2kitti.cpp)

target_link_libraries(pcb2kitti 
                        ${catkin_LIBRARIES} 
                        ${PCL_LIBRARIES} 
                        ${OpenCV_LIBRARIES})
