CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(libusb)

IF (APPLE)
    # header needed for osx
    INCLUDE_DIRECTORIES(libusb/Xcode/)

    # source needed for osx
    SET(source ${source}
            libusb/libusb/os/darwin_usb.c
            libusb/libusb/os/poll_posix.c
            libusb/libusb/os/threads_posix.c
            )

    # Library needed in osx
    FIND_LIBRARY(COREFOUNDATION_LIBRARY CoreFoundation)
    FIND_LIBRARY(IOKIT_LIBRARY IOKit)
    SET(library ${library} ${COREFOUNDATION_LIBRARY} ${IOKIT_LIBRARY} objc)
ENDIF ()

CONFIGURE_FILE(libusb/libusb/libusb.h ${PROJECT_BINARY_DIR}/include/libusb-1.0/libusb.h @ONLY)

INCLUDE_DIRECTORIES(libusb/libusb/)

SET(source ${source}
        libusb/libusb/core.c
        libusb/libusb/descriptor.c
        libusb/libusb/hotplug.c
        libusb/libusb/io.c
        libusb/libusb/sync.c
        )

ADD_LIBRARY(usb SHARED ${source})

TARGET_LINK_LIBRARIES(usb ${library})
