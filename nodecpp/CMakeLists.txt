## CMake file for building an openBuildNet node in C++.
## - Place this file in the same directory as the main C++ source files for your node.
## - Adapt this file to point the header and source files of the node.cpp framework: specifically the OBN_MAIN_DIR.
## - Edit this file to include your source files (and header files if there are any).
## - Create a new /build directory, change to that directory and run
##		cmake ..
##   which will create the Makefile file to build your node. You may also generate projects for XCode, MS Visual Studio, etc.

CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0 FATAL_ERROR)

## Here comes the name of your project:
SET(PROJECT_NAME nodecpp_mynode)

SET(PROJECT_VERSION "0.1"  ## "hg-tip"
    CACHE STRING "${PROJECT_NAME} version string")

## Do not edit this.
PROJECT(${PROJECT_NAME})

## Change OBN_MAIN_DIR to the path to the main directory of openBuildNet
set (OBN_MAIN_DIR ../)

## This will set the header and source directories for the node.cpp framework
set (OBN_NODECPP_INCLUDE_DIR ${OBN_MAIN_DIR}/nodecpp/include)
set (OBN_NODECPP_SOURCE_DIR ${OBN_MAIN_DIR}/nodecpp/src)


## You can set your include and source directories below
set (PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR})
set (PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})


## Include directories for the general OBNSim project
set (OBNSIM_INCLUDE_DIR ${OBN_MAIN_DIR}/include)


IF(UNIX)
  SET(INSTALL_BIN_DIR "bin" CACHE STRING
    "Subdir for installing the binaries")
ELSE(UNIX)
  SET(INSTALL_BIN_DIR "." CACHE STRING
                  "Subdir for installing the binaries")
ENDIF(UNIX)


# guard against bad build-type strings
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()


## Generate code for the message formats using Google ProtoBuf, see http://www.cmake.org/cmake/help/v3.0/module/FindProtobuf.html
## The source files and header files are listed in PROTO_SRCS and PROTO_HDRS respectively
## The libraries and header files of the ProtoBuf framework (not the generated files) are in PROTOBUF_LITE_LIBRARIES and PROTOBUF_INCLUDE_DIRS respectively, because we will generate lite code instead of the full ProtoBuf code.

find_package(Protobuf REQUIRED)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${OBN_MAIN_DIR}/msg/obnsim_msg.proto ${OBN_MAIN_DIR}/msg/obnsim_io.proto)



## Find communication libraries and add appropriate source files
set(OBN_COMM_SRC "")
set(OBN_COMM_HDR "")

## To use YARP (optionally)
find_package(YARP REQUIRED)
if(YARP_FOUND)
  include_directories(${YARP_INCLUDE_DIRS})
  link_libraries(${YARP_LIBRARIES})
  set(OBN_COMM_SRC ${OBN_COMM_SRC}
	${OBN_NODECPP_SOURCE_DIR}/obnnode_yarpnode.cpp
	${OBN_NODECPP_SOURCE_DIR}/obnnode_yarpport.cpp
  )
  set(OBN_COMM_HDR ${OBN_COMM_HDR}
	${OBN_NODECPP_INCLUDE_DIR}/obnnode_yarpnode.h
	${OBN_NODECPP_INCLUDE_DIR}/obnnode_yarpportbase.h
	${OBN_NODECPP_INCLUDE_DIR}/obnnode_yarpport.h
	${OBN_NODECPP_INCLUDE_DIR}/sharedqueue_yarp.h
  )
  add_definitions(-DOBN_COMM_YARP)
  message("Found YARP library; will build node with YARP support.")
endif()


## These are the include directories used by the compiler.

INCLUDE_DIRECTORIES(
  ${OBN_NODECPP_INCLUDE_DIR}
  ${PROJECT_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}
  ${OBNSIM_INCLUDE_DIR}
)

IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

## Sometimes MSVC overwhelms you with compiler warnings which are impossible to
## avoid. Then comment out these sections.

#IF(MSVC)
#  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}
#      /wd4250 /wd4355 /wd4503 /wd4800 /wd4996")
# # Suppressed warnings:
# # C4250: 'class1' : inherits 'class2::member' via dominance
# # C4355: 'this' : used in base member initializer list
# # C4503: 'function' : decorated name length exceeded, name was truncated
# # C4800: 'type' : forcing value to bool 'true' or 'false'
# #        (performance warning)
# # C4996: 'function': was declared deprecated
# ENDIF(MSVC)


INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}
  ${PROTOBUF_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)


set(OBN_CORE_SRCFILES
	${PROTO_SRCS}
	${OBN_COMM_SRC}
)

set(OBN_CORE_HDRFILES
	${OBN_NODECPP_INCLUDE_DIR}/obnnode.h
	${OBN_NODECPP_INCLUDE_DIR}/obnnode_basic.h
	${OBNSIM_INCLUDE_DIR}/obnsim_basic.h
	${PROTO_HDRS}
	${OBN_COMM_HDR}
)

## When we generate Xcode projects, we need to include both the C++ and H files so that they will all be included in the projects.
## It's unnecessary for Makefile.

if(CMAKE_GENERATOR STREQUAL Xcode)
    set(OBN_CORE_SRCFILES ${OBN_CORE_SRCFILES} ${OBN_CORE_HDRFILES})
endif()


ADD_EXECUTABLE(nodecpp-mynode
	nodemain.cpp
	${OBN_CORE_SRCFILES}
)

## Make sure that C++ 11 is used (for thread, mutex...)
if(APPLE)
  list( APPEND CMAKE_CXX_FLAGS "-stdlib=libc++ -std=c++11 ${CMAKE_CXX_FLAGS}")
else()
  set_property(TARGET nodecpp-mynode PROPERTY CXX_STANDARD 11)
  set_property(TARGET nodecpp-mynode PROPERTY CXX_STANDARD_REQUIRED ON)
endif()


TARGET_LINK_LIBRARIES(nodecpp-mynode
  ${PROTOBUF_LITE_LIBRARIES}
)


INSTALL(
  TARGETS nodecpp-mynode
  RUNTIME DESTINATION ${INSTALL_BIN_DIR}
  COMPONENT bin
)
