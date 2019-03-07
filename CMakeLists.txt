cmake_minimum_required(VERSION 3.13)
project(Geonoke)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

#Eigen
include_directories(/usr/include/eigen3/Eigen)

set(SOURCE_FILES src/Vector.h src/Vector.cpp)
set(SOURCE_FILES ${SOURCE_FILES} src/Curve.h src/Curve.cpp)
set(SOURCE_FILES ${SOURCE_FILES} src/Tetra.h src/Tetra.cpp)

add_library(Geonoke STATIC ${SOURCE_FILES})

target_link_libraries(Geonoke ${OpenCV_LIBS})