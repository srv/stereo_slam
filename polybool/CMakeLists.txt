cmake_minimum_required(VERSION 2.8)
project( polybool )

add_library(polybool 
pbgeom.cpp 
pbio.cpp 
pbpolys.cpp 
pbsweep.cpp 
pbunit.cpp 
polybool.cpp 
triacons.cpp 
triamono.cpp)
add_executable(pbunit pbunit.cpp)
target_link_libraries(pbunit polybool)
