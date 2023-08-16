cmake_minimum_required(VERSION 3.11)
project(liorf)


set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# ccache setup
if(USE_CCACHE)
    find_program(CCACHE_PATH ccache)
    if(CCACHE_PATH)
        set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
        set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
        message(STATUS "Using ccache: ${CCACHE_PATH}")
    endif()
endif()

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib") # This is a hack to find GeographicLib, see https://bugs.launchpad.net/ubuntu/+source/geographiclib/+bug/1805173
find_package(GeographicLib REQUIRED)


# build messages
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/CloudInfo.msg"
        "srv/SaveMap.srv"
        DEPENDENCIES
        std_msgs
        sensor_msgs
)

add_subdirectory(kiss_icp)




# build common library
add_library(common lib/common_lib.cpp)
target_include_directories(common PUBLIC lib)
target_link_libraries(common PUBLIC ${PCL_LIBRARIES})


include_directories(include)


add_library(scancontext src/Scancontext.cpp)
target_link_libraries(scancontext PUBLIC ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})




###########
## Build ##
###########
# <!-- liorf_yjz_lucky_boy -->
# Range Image Projection
ament_auto_add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
rosidl_target_interfaces(${PROJECT_NAME}_imageProjection ${PROJECT_NAME} "rosidl_typesupport_cpp")

## IMU Preintegration
ament_auto_add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
rosidl_target_interfaces(${PROJECT_NAME}_imuPreintegration ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_imuPreintegration Boost::timer ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)


### TransformFusion
ament_auto_add_executable(${PROJECT_NAME}_transformFusion src/transformFusion.cpp)
rosidl_target_interfaces(${PROJECT_NAME}_transformFusion ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_transformFusion Boost::timer ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)


#
## Mapping Optimization
ament_auto_add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
rosidl_target_interfaces(${PROJECT_NAME}_mapOptmization ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization
        Boost::timer
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OpenMP_CXX_FLAGS}
        ${GeographicLib_LIBRARIES}
        gtsam
        scancontext)

ament_auto_add_executable(kiss_icp_odom kiss_icp_odom/OdometryServer.cpp kiss_icp_odom/OdometryServerNode.cpp)
target_link_libraries(kiss_icp_odom kiss_icp::pipeline gtsam ${GeographicLib_LIBRARIES})


ament_auto_package(INSTALL_TO_SHARE
        launch
        config
)
