# cmake_minimum_required( VERSION 2.8 )
# project( rcc )

# set( CMAKE_BUILD_TYPE "Release" )
# set(CMAKE_CXX_STANDARD 14)

# find_package( OpenCV 3.1 REQUIRED )
# include_directories( ${OpenCV_INCLUDE_DIRS} )

# find_package(Ceres REQUIRED)
# include_directories(${CERES_INCLUDE_DIRS})

# find_package(Eigen3)
# include_directories(${EIGEN3_INCLUDE_DIRS})

# add_executable( rcc src/main.cpp  )
# target_link_libraries( rcc ${OpenCV_LIBS} 
#                             ${EIGEN3_LIBARIES}
#                             ${CERES_LIBRARIES} )
cmake_minimum_required(VERSION 3.0)
project(rcc)

###############################################################################
## file globbing ##############################################################
###############################################################################

# these instructions search the directory tree when cmake is
# invoked and put all files that match the pattern in the variables 
# `sources` and `data`
file(GLOB_RECURSE sources      src/*.cpp)
# you can use set(sources src/main.cpp) etc if you don't want to
# use globing to find files automatically
###############################################################################
## target definitions #########################################################
###############################################################################

# add the data to the target, so it becomes visible in some IDE

find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(rcc ${sources})

target_link_libraries( rcc ${OpenCV_LIBS} 
                            ${EIGEN3_LIBARIES}
                            ${CERES_LIBRARIES} )