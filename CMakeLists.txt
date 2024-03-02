cmake_minimum_required(VERSION 3.10)

project(lite_sfm)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DCV_IMSHOW_DEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DPCL_VIEWER_DEBUG")


include_directories(${PROJECT_SOURCE_DIR}/include)

# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# # FMT
find_package(FMT REQUIRED)
# include_directories(${FMT_INCLUDE_DIRS})

# Sophus 
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS} )
# Boost
find_package(Boost REQUIRED COMPONENTS system filesystem chrono)
include_directories(${Boost_INCLUDE_DIR})
# PCL
find_package(PCL REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        fmt::fmt
        ${Sophus_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        )

aux_source_directory(${PROJECT_SOURCE_DIR}/src/ SRCS)

add_executable(lite_sfm ${SRCS})
target_link_libraries(lite_sfm ${THIRD_PARTY_LIBS})
target_link_libraries(lite_sfm -lX11)  # PCL可视化支持(仍存在窗口启动失败的情况)

