cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(kd_tree)
ADD_DEFINITIONS(-std=c++14)

include_directories (/usr/local/include)
link_directories (/usr/local/lib)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable (kd_tree kd_tree.cpp)
target_link_libraries (kd_tree Eigen3::Eigen)
