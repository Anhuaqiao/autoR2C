cmake_minimum_required( VERSION 2.8 )
project( rcc )

set( CMAKE_BUILD_TYPE "Release" )
set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable( rcc src/main.cpp  )
target_link_libraries( rcc ${OpenCV_LIBS} 
                            ${EIGEN3_LIBARIES}
                            ${CERES_LIBRARIES} )

