cmake_minimum_required(VERSION 3.13)
project(Geonoke)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

#Eigen
include_directories(/usr/include/eigen3/Eigen)

set(SOURCE_FILES Geonoke/Vector.h Geonoke/Vector.cpp)
set(SOURCE_FILES ${SOURCE_FILES} Geonoke/Curve.h Geonoke/Curve.cpp)
set(SOURCE_FILES ${SOURCE_FILES} Geonoke/Tetra.h Geonoke/Tetra.cpp)

add_library(Geonoke STATIC ${SOURCE_FILES})

target_link_libraries(Geonoke ${OpenCV_LIBS})