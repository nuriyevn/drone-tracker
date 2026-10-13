
cmake_minimum_required(VERSION 3.24)

project(drone_tracker_cpp
    VERSION 0.1.0
    DESCRIPTION "Real-time drone tracking using C++ and OpenCV"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV CONFIG REQUIRED)

add_library(drone_tracker_lib STATIC
    src/models/ObjectPosition.cpp
    src/sources/CameraVideoSource.cpp
    src/sources/FileVideoSource.cpp
    src/tracking/CSRTTracker.cpp
    src/application/TrackingApplication.cpp
)

target_include_directories(drone_tracker_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(drone_tracker_lib
    PUBLIC
        ${OpenCV_LIBS}
)

if(WIN32)
    target_link_libraries(drone_tracker_lib PRIVATE strmiids ole32)
endif()

add_executable(drone-tracker
    src/main.cpp
)

target_link_libraries(drone-tracker
    PRIVATE
        drone_tracker_lib
)

if(MSVC)
    target_compile_options(drone_tracker_lib PRIVATE /W4 /permissive- /EHsc)
    target_compile_options(drone-tracker PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(drone_tracker_lib PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(drone-tracker PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()

add_executable(drone_tracker_tests
    tests/ObjectPositionTests.cpp
    src/models/ObjectPosition.cpp
)

target_include_directories(drone_tracker_tests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(drone_tracker_tests
    PRIVATE
        ${OpenCV_LIBS}
)

add_test(NAME ObjectPositionTests COMMAND drone_tracker_tests)
