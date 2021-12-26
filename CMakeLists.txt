project(db_carto)

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)
set(cartographer_DIR /home/moi/include/cartographer/share/cartographer)

# 日志相关
#list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(USE_STACK_TRACE_LOGGER ON CACHE BOOL "Enable automatic stack trace logger of google-glog")
set(USE_STACK_TRACE_LOGGER ON)
if (USE_STACK_TRACE_LOGGER)
    message(STATUS "Stack trace logger: ENABLED")
    # Glog
    find_package(Glog REQUIRED)
    include_directories(${GLOG_INCLUDE_DIR})
    add_definitions(-D USE_STACK_TRACE_LOGGER)
    message(STATUS "GLOG_INCLUDE_DIR: ${GLOG_INCLUDE_DIR}")
    message(STATUS "GLOG_LIBRARIES: ${GLOG_LIBRARIES}")
else ()
    message(STATUS "Stack trace logger: DISABLED")
endif ()

set(PACKAGE_DEPENDENCIES
        eigen_conversions
        message_runtime
        roscpp
        roslib
        rviz
        dbrobot_msg
        )

# 依赖库 
# 1 cartographer
# 2 Boost 
# 3 OpenCV
find_package(cartographer REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

# 包含 CARTOGRAPHER 所需要的cmake文件
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
#google_initialize_cartographer_project()

# 包含的文件

set(MAP_TEST_SRC
        src/db_carto.cpp
        src/sensor_ros.cpp)
# gcc编译选项
add_compile_options(-g)
add_compile_options(-w)
catkin_package()

include_directories(
        ${INCLUDE_CARTO}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(db_carto src/main.cpp ${MAP_TEST_SRC})
target_link_libraries(db_carto  glog ${PROTOBUF_LIBRARY} ${Boost_LIBRARIES} cartographer ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(SlamManager ${MAP_TEST_SRC} src/SlamManager.cpp)
target_link_libraries(SlamManager  glog ${PROTOBUF_LIBRARY} ${Boost_LIBRARIES} cartographer ${OpenCV_LIBS} ${catkin_LIBRARIES})
