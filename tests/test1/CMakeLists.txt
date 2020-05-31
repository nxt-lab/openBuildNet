## This builds the test project, including source files from the SMN and the node frameworks.

CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0 FATAL_ERROR)

## Here comes the name of your project:
SET(PROJECT_NAME "test1")

PROJECT(${PROJECT_NAME})

## Change OBN_MAIN_DIR to the path to the main directory of openBuildNet
set (OBN_MAIN_DIR ${PROJECT_SOURCE_DIR}/../../)

## Directories of the SMN source
set(OBNSMN_SRC_DIR "${OBN_MAIN_DIR}/smn/src")
set(OBNSMN_INCLUDE_DIR "${OBN_MAIN_DIR}/smn/include")


# Include the common CMake code for node.C++
INCLUDE(${OBN_MAIN_DIR}/nodecpp/CMakeCommon.txt)


## Find communication libraries and add appropriate source files
set(OBNSMN_COMM_SRC "")
set(OBNSMN_COMM_HDR "")

## To use YARP (optionally)
if(WITH_YARP)
  find_package(YARP REQUIRED)
  include_directories(${YARP_INCLUDE_DIRS})
  link_libraries(${YARP_LIBRARIES})
  set(OBNSMN_COMM_SRC ${OBNSMN_COMM_SRC} ${OBNSMN_SRC_DIR}/obnsmn_comm_yarp.cpp)
  set(OBNSMN_COMM_HDR ${OBNSMN_COMM_HDR} ${OBNSMN_INCLUDE_DIR}/obnsmn_comm_yarp.h)
  add_definitions(-DOBNSIM_COMM_YARP)
  message("Found YARP library; will build SMN with YARP support.")
endif(WITH_YARP)


## To use MQTT (optionally)
if(WITH_MQTT)
  find_library(PAHOMQTT3A NAMES paho-mqtt3a)
  if(PAHOMQTT3A)
    message("Found Paho MQTT3 library in the system; will use this library: ${PAHOMQTT3A}")
    link_libraries(${PAHOMQTT3A})
  else()
    message(FATAL_ERROR "Did not find Paho MQTT3 library in the system.")
  endif()

  add_definitions(-DOBNSIM_COMM_MQTT)
  set(OBNSMN_COMM_SRC ${OBNSMN_COMM_SRC} ${OBNSMN_SRC_DIR}/obnsmn_comm_mqtt.cpp)
  set(OBNSMN_COMM_HDR ${OBNSMN_COMM_HDR} ${OBNSMN_INCLUDE_DIR}/obnsmn_comm_mqtt.h)
endif(WITH_MQTT)

## These are the include directories used by the compiler.

INCLUDE_DIRECTORIES(
  ${OBNSMN_INCLUDE_DIR}
)

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


set(OBNSMN_CORE_SRCFILES
	${OBNSMN_SRC_DIR}/obnsmn_event.cpp
	${OBNSMN_SRC_DIR}/obnsmn_node.cpp
	${OBNSMN_SRC_DIR}/obnsmn_nodegraph.cpp
    	${OBNSMN_SRC_DIR}/obnsmn_gc.cpp
	${OBNSIM_INCLUDE_DIR}/obnsim_basic.cpp
	${OBNSMN_COMM_SRC}
	${PROTO_SRCS}
)


set(OBNSMN_CORE_HDRFILES
	${OBNSMN_INCLUDE_DIR}/obnsmn_basic.h
	${OBNSMN_INCLUDE_DIR}/obnsmn_event.h
	${OBNSMN_INCLUDE_DIR}/obnsmn_gc.h
	${OBNSMN_INCLUDE_DIR}/obnsmn_gc_inline.h
	${OBNSMN_INCLUDE_DIR}/obnsmn_node.h
	${OBNSMN_INCLUDE_DIR}/obnsmn_nodegraph.h
	${OBNSMN_INCLUDE_DIR}/sharedqueue.h
	${OBNSMN_INCLUDE_DIR}/obnsmn_report.h
	${OBNSIM_INCLUDE_DIR}/obnsim_basic.h
	${OBNSMN_COMM_HDR}
	${PROTO_HDRS}
)


## When we generate Xcode projects, we need to include both the C++ and H files so that they will all be included in the projects.
## It's unnecessary for Makefile.
if(CMAKE_GENERATOR STREQUAL Xcode)
    set(OBNSMN_CORE_SRCFILES ${OBNSMN_CORE_SRCFILES} ${OBNSMN_CORE_HDRFILES})
endif()


ADD_EXECUTABLE(smntest1
	smntest1.cpp
	${OBNSMN_CORE_SRCFILES}
)

ADD_EXECUTABLE(nodetest1
	nodetest1.cpp
	${OBNNODE_CORE_SRCFILES}
)


## Make sure that C++ 14 is used (for thread, mutex...)
if(APPLE)
  list( APPEND CMAKE_CXX_FLAGS "-stdlib=libc++ -std=c++14 ${CMAKE_CXX_FLAGS}")
else()
  set_property(TARGET smntest1 nodetest1 PROPERTY CXX_STANDARD 14)
  set_property(TARGET smntest1 nodetest1 PROPERTY CXX_STANDARD_REQUIRED ON)
endif()

## Set compiler options
if(MSVC)
  # Force to always compile with W4
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
  list(APPEND CMAKE_CXX_FLAGS "/w44640 ${CMAKE_CXX_FLAGS}")
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  # Update if necessary
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wshadow -Wnon-virtual-dtor -pedantic")  # -Wextra
endif()

