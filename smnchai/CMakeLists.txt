## This builds the SMNChai project

CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0 FATAL_ERROR)

## Here comes the name of your project:
SET(PROJECT_NAME "smnchai")

PROJECT(${PROJECT_NAME})


## Change OBN_MAIN_DIR to the path to the main directory of openBuildNet
set (OBN_MAIN_DIR ../)


## Directories of the SMN source
set(OBNSMN_SRC_DIR "${OBN_MAIN_DIR}/smn/src")
set(OBNSMN_INCLUDE_DIR "${OBN_MAIN_DIR}/smn/include")


## Include directories for the general OBNSim project
set (OBNSIM_INCLUDE_DIR ${OBN_MAIN_DIR}/include)


option(BUILD_STATIC_CHAI "Build SMNChai with static ChaiScript standard library instead of dynamic one." OFF)

## Generate code for the message formats using Google ProtoBuf, see http://www.cmake.org/cmake/help/v3.0/module/FindProtobuf.html
## The source files and header files are listed in PROTO_SRCS and PROTO_HDRS respectively
## The libraries and header files of the ProtoBuf framework (not the generated files) are in PROTOBUF_LITE_LIBRARIES and PROTOBUF_INCLUDE_DIRS respectively, because we will generate lite code instead of the full ProtoBuf code.

find_package(Protobuf REQUIRED)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${OBN_MAIN_DIR}/msg/obnsim_msg.proto ${OBN_MAIN_DIR}/msg/obnsim_io.proto)


## To use Boost library
# see http://stackoverflow.com/questions/6646405/how-do-you-add-boost-libraries-in-cmakelists-txt
# and http://www.cmake.org/cmake/help/v3.0/module/FindBoost.html
# Because Boost is required, it will be an error if Boost is not found

# set(Boost_USE_STATIC_LIBS OFF) 
# set(Boost_USE_MULTITHREADED ON)  
# set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.55.0 REQUIRED COMPONENTS graph filesystem system) 


## Find communication libraries and add appropriate source files
set(OBNSMN_COMM_SRC "")
set(OBNSMN_COMM_HDR "")


## To use YARP (optionally)
find_package(YARP)
if(YARP_FOUND)
  include_directories(${YARP_INCLUDE_DIRS})
  link_libraries(${YARP_LIBRARIES})
  set(OBNSMN_COMM_SRC ${OBNSMN_COMM_SRC} ${OBNSMN_SRC_DIR}/obnsmn_comm_yarp.cpp)
  set(OBNSMN_COMM_HDR ${OBNSMN_COMM_HDR} ${OBNSMN_INCLUDE_DIR}/obnsmn_comm_yarp.h)
  add_definitions(-DOBNSIM_COMM_YARP)
  message("Found YARP library; will build SMN with YARP support.")
endif()


## These are the include directories used by the compiler.

INCLUDE_DIRECTORIES(
  ${PROJECT_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}
  include
  ${OBNSMN_INCLUDE_DIR}
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


set(SMNCHAI_SRCFILES
	src/smnchai_api.cpp
	src/smnchai_export.cpp
	src/csvparser.c
	src/smnchai_utils.cpp
	src/main.cpp
)


set(SMNCHAI_HDRFILES
	include/smnchai_api.h
	include/csvparser.h
	include/smnchai_utils.h
)


## When we generate Xcode projects, we need to include both the C++ and H files so that they will all be included in the projects.
## It's unnecessary for Makefile.
if(CMAKE_GENERATOR STREQUAL Xcode)
    set(OBNSMN_CORE_SRCFILES ${OBNSMN_CORE_SRCFILES} ${OBNSMN_CORE_HDRFILES} ${SMNCHAI_SRCFILES} ${SMNCHAI_HDRFILES})
endif()


## Compile with static Chaiscript or dynamic
if(BUILD_STATIC_CHAI)
    add_definitions(-DSMNCHAI_CHAISCRIPT_STATIC)
endif(BUILD_STATIC_CHAI)

ADD_EXECUTABLE(smnchai
	${SMNCHAI_SRCFILES}
	${OBNSMN_CORE_SRCFILES}
)


## Make sure that C++ 11 is used (for thread, mutex...)
if(APPLE)
  list( APPEND CMAKE_CXX_FLAGS "-stdlib=libc++ -std=c++11 ${CMAKE_CXX_FLAGS}")
else()
  set_property(TARGET smntest2 nodemotor nodectrl nodesetpoint PROPERTY CXX_STANDARD 11)
  set_property(TARGET smntest2 nodemotor nodectrl nodesetpoint PROPERTY CXX_STANDARD_REQUIRED ON)
endif()


TARGET_LINK_LIBRARIES(smnchai
  ${PROTOBUF_LITE_LIBRARIES}
  ${Boost_LIBRARIES}
)


## The auxiliary doxygen files (.dox) should be placed in the 'doc'
## subdirectory. The next line includes the CMAKE config of that directory.

# ADD_SUBDIRECTORY(doc)

# Select the desired build type
set(CMAKE_BUILD_TYPE Release)
