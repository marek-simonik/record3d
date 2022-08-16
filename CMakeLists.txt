cmake_minimum_required(VERSION 3.13)
project(record3d)
set(CMAKE_CXX_STANDARD 14)

if (APPLE)
    set(CMAKE_OSX_ARCHITECTURES "x86_64;arm64" CACHE INTERNAL "" FORCE)
endif()

if(UNIX AND NOT APPLE)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

# Add LZFSE
if(WIN32)
    set(LZFSE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/win/lzfse-lzfse-1.0)
else()
    set(LZFSE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/unix/lzfse-lzfse-1.0)
endif()
add_subdirectory(${LZFSE_DIR})
include_directories(${LZFSE_DIR}/src/lzfse)

# Add libusbmuxd
if(WIN32)
    set(libusbmuxd_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/win/libusbmuxd)
else()
    set(libusbmuxd_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/unix/libusbmuxd)
endif()
add_subdirectory(${libusbmuxd_DIR})
include_directories(${libusbmuxd_DIR}/include)

# Add OpenCV if available
find_package(OpenCV)
if (OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIR})
    add_compile_definitions(HAS_OPENCV)
endif ()


########################################################################
### Create C++ library
########################################################################
include(GNUInstallDirs)
add_library(record3d_cpp STATIC src/Record3DStream.cpp src/JPEGDecoder.cpp)
include_directories(include)
target_link_libraries(record3d_cpp PUBLIC lzfse usbmuxd)
install(TARGETS record3d_cpp lzfse usbmuxd plist EXPORT Record3DConfig
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT Record3DConfig DESTINATION share/Record3D/cmake)


########################################################################
### Create Python module (using pybind11)
########################################################################
if(BUILD_PYTHON_BINDINGS)
    include(python-bindings/pybind11/tools/FindPythonLibsNew.cmake)
    include_directories(python-bindings/pybind11/include)

    include_directories(${PYTHON_INCLUDE_DIRS})
    add_library(record3d_py STATIC src/Record3DStream.cpp src/JPEGDecoder.cpp)
    target_compile_definitions(record3d_py PRIVATE PYTHON_BINDINGS_BUILD)
    target_link_libraries(record3d_py lzfse usbmuxd)

    add_subdirectory(python-bindings/pybind11)
    pybind11_add_module(record3d python-bindings/src/PythonBindings.cpp)
    target_compile_definitions(record3d PRIVATE PYTHON_BINDINGS_BUILD)
    target_link_libraries(record3d PRIVATE record3d_py)
endif()


########################################################################
### Create demo project that uses the library
########################################################################
add_executable(demo src/DemoMain.cpp)
target_link_libraries(demo record3d_cpp)
if (OpenCV_FOUND)
    target_link_libraries(demo ${OpenCV_LIBS})
endif ()

set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY true)
