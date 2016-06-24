cmake_minimum_required(VERSION 3.3)
project(openzed)

# Hide warning on policy CMP0042
SET(CMAKE_MACOSX_RPATH TRUE)

# Build library usb
add_subdirectory(submodule/libusb)

# Build library uvc
add_subdirectory(submodule/libuvc)

# Build openzed
include_directories(submodule/libuvc/libuvc/include ${libusb_BINARY_DIR}/include)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SOURCE_FILES main.cpp)
add_executable(openzed ${SOURCE_FILES})
target_link_libraries(openzed uvc)