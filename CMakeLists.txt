cmake_minimum_required(VERSION 3.14)
project(SCServo)

# Set modern C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# SCServo Library

# Collect all source files from src/
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/*.cpp)

# Collect all header files from include/ (optional — not required for compilation)
file(GLOB_RECURSE HEADER_FILES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/include/*.h*)

# Define the static library
add_library(SCServo STATIC ${SRC_FILES} ${HEADER_FILES})

# Make headers available to anything linking against the library
target_include_directories(SCServo PUBLIC ${CMAKE_SOURCE_DIR}/include)

# Examples
add_subdirectory(examples/SMS_STS/Ping)
add_subdirectory(examples/SMS_STS/FeedBack)
add_subdirectory(examples/SMS_STS/SyncRead)
add_subdirectory(examples/SMS_STS/SyncWritePos)