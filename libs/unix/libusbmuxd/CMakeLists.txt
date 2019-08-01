project(usbmuxd)
cmake_minimum_required(VERSION 3.12)

set(libplist_DIR libs/libplist)
add_subdirectory(${libplist_DIR})

include(CheckSymbolExists)
include(CheckIncludeFile)
include(CheckIncludeFiles)

check_symbol_exists(stpncpy string.h HAVE_STPNCPY)
check_symbol_exists(sleep string.h HAVE_SLEEP)
check_symbol_exists(pselect sys/select.h HAVE_PSELECT)
check_include_file(sys/inotify.h HAVE_INOTIFY_H)

if (HAVE_STPNCPY)
    add_compile_definitions(HAVE_STPNCPY)
endif()
if (HAVE_INOTIFY_H)
    add_compile_definitions(HAVE_INOTIFY)
endif()
if (HAVE_PSELECT)
    add_compile_definitions(HAVE_PSELECT)
endif()

include_directories(${libplist_DIR}/include)

add_library(usbmuxd STATIC
        src/libusbmuxd.c
        src/collection.c
        src/socket.c
        src/thread.c)

if(UNIX AND NOT APPLE)
    target_link_libraries(usbmuxd pthread)
endif()

target_link_libraries(usbmuxd plist)