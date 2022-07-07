cmake_minimum_required(VERSION 3.0)

project(3dpc_eq_if LANGUAGES CXX)

set(PROJECT_AUTHOR "SINTEF Digital")
set(PROJECT_URL "http://i3ds-h2020.eu/")
set(PROJECT_LICENSE "MPLv2")
set(PROJECT_VERSION "1.0")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost COMPONENTS program_options log REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("include/")

add_executable(3dpc_eq_if main.cpp src/disparity_generator.cpp src/stereo_reconstruction.cpp)

set (LIBS
    zmq
    i3ds
    i3ds_asn1
    i3ds_clients
    i3ds_emulators
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
  )

target_link_libraries(3dpc_eq_if ${LIBS})
