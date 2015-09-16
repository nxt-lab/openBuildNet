CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0 FATAL_ERROR)

## Here comes the name of your project:
SET(PROJECT_NAME "busnode")

PROJECT(${PROJECT_NAME})

## Change OBN_MAIN_DIR to the path to the main directory of openBuildNet
set (OBN_MAIN_DIR ../../../)

## This will set the header and source directories for the node.cpp framework
set (OBN_NODECPP_INCLUDE_DIR ${OBN_MAIN_DIR}/nodecpp/include)
set (OBN_NODECPP_SOURCE_DIR ${OBN_MAIN_DIR}/nodecpp/src)

## Include directories for the general OBNSim project
set (OBNSIM_INCLUDE_DIR ${OBN_MAIN_DIR}/include)


## Generate code for the message formats using Google ProtoBuf, see http://www.cmake.org/cmake/help/v3.0/module/FindProtobuf.html
## The source files and header files are listed in PROTO_SRCS and PROTO_HDRS respectively
## The libraries and header files of the ProtoBuf framework (not the generated files) are in PROTOBUF_LITE_LIBRARIES and PROTOBUF_INCLUDE_DIRS respectively, because we will generate lite code instead of the full ProtoBuf code.

find_package(Protobuf REQUIRED)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${OBN_MAIN_DIR}/msg/obnsim_msg.proto ${OBN_MAIN_DIR}/msg/obnsim_io.proto)


## Find communication libraries and add appropriate source files
set(OBNNODE_COMM_SRC "")
set(OBNNODE_COMM_HDR "")

## To use YARP (optionally)
option(WITH_YARP "Build with YARP support for communication." ON)
if(WITH_YARP)
  find_package(YARP REQUIRED)
  if(YARP_FOUND)
    include_directories(${YARP_INCLUDE_DIRS})
    link_libraries(${YARP_LIBRARIES})
    set(OBNNODE_COMM_SRC ${OBNNODE_COMM_SRC}
      ${OBN_NODECPP_SOURCE_DIR}/obnnode_yarpnode.cpp
      ${OBN_NODECPP_SOURCE_DIR}/obnnode_yarpport.cpp
    )
    set(OBNNODE_COMM_HDR ${OBNNODE_COMM_HDR}
      ${OBN_NODECPP_INCLUDE_DIR}/obnnode_yarpnode.h
      ${OBN_NODECPP_INCLUDE_DIR}/obnnode_yarpportbase.h
      ${OBN_NODECPP_INCLUDE_DIR}/obnnode_yarpport.h
      ${OBN_NODECPP_INCLUDE_DIR}/sharedqueue_yarp.h
    )
    add_definitions(-DOBNNODE_COMM_YARP)
    message("Found YARP library; will build node with YARP support.")
  endif()
endif(WITH_YARP)


## To use MQTT (optionally)
option(WITH_PAHOMQTT3 "Build with Paho MQTT3 support for communication." OFF)
if(WITH_PAHOMQTT3)
  find_library(PAHOMQTT3A NAMES paho-mqtt3a)
  if(PAHOMQTT3A_NOTFOUND)
    # Use the local source files (submodule)
    message("Did not find Paho MQTT3 library in the system; will use the local source files. This may bloat your program, so it's best to install Paho into your system before building SMNChai.")

    find_package(OpenSSL REQUIRED)
    AUX_SOURCE_DIRECTORY(${OBN_MAIN_DIR}/thirdparties/paho/src PAHOMQTT3A_SRC)
    INCLUDE_DIRECTORIES(${OBN_MAIN_DIR}/thirdparties/paho/src)
    set(OBNNODE_COMM_SRC ${OBNNODE_COMM_SRC} ${PAHOMQTT3A_SRC})
  else()
    message("Found Paho MQTT3 library in the system; will use this library.")
    link_libraries(${PAHOMQTT3A})
  endif()
  add_definitions(-DOBNNODE_COMM_MQTT)
  #set(OBNNODE_COMM_SRC ${OBNNODE_COMM_SRC} ${OBN_NODECPP_SOURCE_DIR}/.cpp)
  #set(OBNNODE_COMM_HDR ${OBNNODE_COMM_HDR} ${OBN_NODECPP_INCLUDE_DIR}/.h)  
endif(WITH_PAHOMQTT3)


## These are the include directories used by the compiler.

INCLUDE_DIRECTORIES(
  ${PROJECT_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}
  ${OBN_NODECPP_INCLUDE_DIR}
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
  ${CMAKE_CURRENT_BINARY_DIR}
)


set(OBNNODE_CORE_SRCFILES
	${OBNSIM_INCLUDE_DIR}/obnsim_basic.cpp
	${OBN_NODECPP_SOURCE_DIR}/obnnode_basic.cpp
	${PROTO_SRCS}
	${OBNNODE_COMM_SRC}
)

set(OBNNODE_CORE_HDRFILES
	${OBN_NODECPP_INCLUDE_DIR}/obnnode.h
	${OBN_NODECPP_INCLUDE_DIR}/obnnode_basic.h
	${OBN_NODECPP_INCLUDE_DIR}/obnnode_exceptions.h
	${OBNSIM_INCLUDE_DIR}/obnsim_basic.h
	${PROTO_HDRS}
	${OBNNODE_COMM_HDR}
)


## When we generate Xcode projects, we need to include both the C++ and H files so that they will all be included in the projects.
## It's unnecessary for Makefile.
if(CMAKE_GENERATOR STREQUAL Xcode)
    set(OBNNODE_CORE_SRCFILES ${OBNNODE_CORE_SRCFILES} ${OBNNODE_CORE_HDRFILES})
endif()


ADD_EXECUTABLE(obn_powernet_bus
	busnode.cpp
	${OBNNODE_CORE_SRCFILES}
)


## Make sure that C++ 11 is used (for thread, mutex...)
if(APPLE)
  list( APPEND CMAKE_CXX_FLAGS "-stdlib=libc++ -std=c++11 ${CMAKE_CXX_FLAGS}")
else()
  set_property(TARGET obn_powernet_bus PROPERTY CXX_STANDARD 11)
  set_property(TARGET obn_powernet_bus PROPERTY CXX_STANDARD_REQUIRED ON)
endif()


TARGET_LINK_LIBRARIES(obn_powernet_bus
  ${PROTOBUF_LITE_LIBRARIES}
)

