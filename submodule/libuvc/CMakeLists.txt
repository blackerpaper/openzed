CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(libuvc)

INCLUDE_DIRECTORIES(
        libuvc/include
        ${PROJECT_BINARY_DIR}/include
        ${libusb_BINARY_DIR}/include
)

SET(
        SOURCES
        libuvc/src/ctrl.c
        libuvc/src/ctrl-gen.c
        libuvc/src/device.c
        libuvc/src/diag.c
        libuvc/src/frame.c
        libuvc/src/init.c
        libuvc/src/stream.c
        libuvc/src/misc.c
)

ADD_LIBRARY(uvc SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(uvc usb)
