project(plist)
cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 14)

include_directories(include)
include_directories(libcnary/include)
include_directories(src)

add_library(plist STATIC
        src/Array.cpp
        src/base64.c
        src/Boolean.cpp
        src/bplist.c
        src/bytearray.c
        src/Data.cpp
        src/Date.cpp
        src/Dictionary.cpp
        src/hashtable.c
        src/Integer.cpp
        src/Key.cpp
        src/Node.cpp
        src/plist.c
        src/ptrarray.c
        src/Real.cpp
        src/String.cpp
        src/Structure.cpp
        src/time64.c
        src/Uid.cpp
        src/xplist.c
        libcnary/node.c
        libcnary/node_list.c
        )