cmake_minimum_required(VERSION 3.0.2)
project(cad2slam)

find_package(catkin REQUIRED
        srrg_cmake_modules
        srrg2_core
        srrg2_solver
        srrg2_navigation_2d
)

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
message("CMAKE FLAGS SET TO ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Cholmod REQUIRED)
find_package(QGLViewer REQUIRED)
find_package(GLUT REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()

include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
        ${PROJECT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIR}
        ${CHOLMOD_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${QGLViewer_INCLUDE_DIR}
        ${SRRG_QT_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        ${YAMLCPP_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS
        src
        LIBRARIES
        cad2slam_library
        CATKIN_DEPENDS
        srrg_cmake_modules
        srrg2_core
        srrg2_solver
        srrg2_navigation_2d
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${YAMLCPP_LIBRARY_DIRS})

add_subdirectory(src)
