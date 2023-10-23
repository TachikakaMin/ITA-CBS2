cmake_minimum_required(VERSION 3.21)
project(ITACBS_remake)

set(CMAKE_OSX_ARCHITECTURES "arm64")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3 -g")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
find_package(yaml-cpp REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
file(GLOB THIRD_PARTY "third_party/*.cpp" "third_party/*.h")

add_executable(ITACBS_remake main_ITACBS.cpp include/common.hpp include/ITACBS/ITACBS.hpp include/ITACBS/ITACBS.cpp include/ITACBS/ITACBSNode.hpp include/ITACBS/ITACBSNode.cpp include/dynamic_hungarian_assignment.hpp include/common.cpp)
target_link_libraries(ITACBS_remake
        ${Boost_LIBRARIES}
        yaml-cpp
        )


add_executable(CBSTA_remake main_CBSTA.cpp include/common.hpp include/CBSTA/CBSTANode.hpp include/CBSTA/CBSTANode.cpp include/dynamic_hungarian_assignment.hpp include/CBSTA/CBSTA.cpp include/CBSTA/CBSTA.hpp include/CBSTA/TANode.cpp include/CBSTA/TANode.hpp include/common.cpp)
target_link_libraries(CBSTA_remake
        ${Boost_LIBRARIES}
        yaml-cpp
        )


#option(SERIAL "Option for serial version" OFF)
#if(SERIAL)
#    target_compile_definitions(ITACBS_parallel PRIVATE PARLAY_SEQUENTIAL)
#endif()