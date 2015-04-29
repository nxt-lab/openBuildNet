## IMPORTANT NOTE:
## - All source files are in /src directory; to include them, prefix with ${PROJECT_SOURCE_DIR}/
## - All include files are in /include directory; normally, that directory is already included; however, if you want to specify a .h file explicitly, prefix it with ${PROJECT_INCLUDE_DIR}/

CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0 FATAL_ERROR)

## Here comes the name of your project:
SET(PROJECT_NAME "obnsmn_smn")

SET(PROJECT_VERSION "0.1"  ## "hg-tip"
    CACHE STRING "${PROJECT_NAME} version string")

## Do not edit this.
PROJECT(${PROJECT_NAME})

set (PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set (PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


IF(UNIX)
  SET(INSTALL_BIN_DIR "bin" CACHE STRING
    "Subdir for installing the binaries")
ELSE(UNIX)
  SET(INSTALL_BIN_DIR "." CACHE STRING
                  "Subdir for installing the binaries")
ENDIF(UNIX)


## Generate code for the message formats using Google ProtoBuf, see http://www.cmake.org/cmake/help/v3.0/module/FindProtobuf.html
## The source files and header files are listed in PROTO_SRCS and PROTO_HDRS respectively
## The libraries and header files of the ProtoBuf framework (not the generated files) are in PROTOBUF_LITE_LIBRARIES and PROTOBUF_INCLUDE_DIRS respectively, because we will generate lite code instead of the full ProtoBuf code.

find_package(Protobuf REQUIRED)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ../msg/obnsim_msg.proto)


## To use Boost library
# see http://stackoverflow.com/questions/6646405/how-do-you-add-boost-libraries-in-cmakelists-txt
# and http://www.cmake.org/cmake/help/v3.0/module/FindBoost.html
# Because Boost is required, it will be an error if Boost is not found

# set(Boost_USE_STATIC_LIBS OFF) 
# set(Boost_USE_MULTITHREADED ON)  
# set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.55.0 REQUIRED COMPONENTS graph) 


## This line finds doxygen (for document creation)
# FIND_PACKAGE(Doxygen)


## Find communication libraries and add appropriate source files
set(OBNSMN_COMM_SRC "")
set(OBNSMN_COMM_HDR "")

## To use YARP (optionally)
find_package(YARP)
if(YARP_FOUND)
  include_directories(${YARP_INCLUDE_DIRS})
  link_libraries(${YARP_LIBRARIES})
  set(OBNSMN_COMM_SRC ${OBNSMN_COMM_SRC} ${PROJECT_SOURCE_DIR}/obnsmn_comm_yarp.cpp)
  set(OBNSMN_COMM_HDR ${OBNSMN_COMM_HDR} ${PROJECT_INCLUDE_DIR}/obnsmn_comm_yarp.h)
  add_definitions(-DOBNSIM_COMM_YARP)
  message("Found YARP library; will build SMN with YARP support.")
endif()


## These are the include directories used by the compiler.

INCLUDE_DIRECTORIES(
  ${PROJECT_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}
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
	${PROJECT_SOURCE_DIR}/obnsmn_event.cpp
	${PROJECT_SOURCE_DIR}/obnsmn_node.cpp
	${PROJECT_SOURCE_DIR}/obnsmn_nodegraph.cpp
    	${PROJECT_SOURCE_DIR}/obnsmn_gc.cpp
	${PROTO_SRCS}
	${OBNSMN_COMM_SRC}
)

set(OBNSMN_CORE_HDRFILES
	${PROJECT_INCLUDE_DIR}/obnsmn_basic.h
	${PROJECT_INCLUDE_DIR}/obnsmn_event.h
	${PROJECT_INCLUDE_DIR}/obnsmn_gc.h
	${PROJECT_INCLUDE_DIR}/obnsmn_node.h
	${PROJECT_INCLUDE_DIR}/obnsmn_nodegraph.h
	${PROJECT_INCLUDE_DIR}/sharedqueue.h
	${PROJECT_INCLUDE_DIR}/obnsmn_report.h
	${PROTO_HDRS}
	${OBNSMN_COMM_HDR}
)

## When we generate Xcode projects, we need to include both the C++ and H files so that they will all be included in the projects.
## It's unnecessary for Makefile.

if(CMAKE_GENERATOR STREQUAL Xcode)
    set(OBNSMN_CORE_SRCFILES ${OBNSMN_CORE_SRCFILES} ${OBNSMN_CORE_HDRFILES})
endif()


ADD_EXECUTABLE(obnsim-smn
	${PROJECT_SOURCE_DIR}/main.cpp
	${OBNSMN_CORE_SRCFILES}
)

## Make sure that C++ 11 is used (for thread, mutex...)
if(APPLE)
  list( APPEND CMAKE_CXX_FLAGS "-stdlib=libc++ -std=c++11 ${CMAKE_CXX_FLAGS}")
else()
  set_property(TARGET obnsim-smn PROPERTY CXX_STANDARD 11)
  set_property(TARGET obnsim-smn PROPERTY CXX_STANDARD_REQUIRED ON)
endif()


TARGET_LINK_LIBRARIES(obnsim-smn
  ${PROTOBUF_LITE_LIBRARIES}
  ${Boost_LIBRARIES}
)


INSTALL(
  TARGETS obnsim-smn
  RUNTIME DESTINATION ${INSTALL_BIN_DIR}
  COMPONENT bin
)


## The auxiliary doxygen files (.dox) should be placed in the 'doc'
## subdirectory. The next line includes the CMAKE config of that directory.

# ADD_SUBDIRECTORY(doc)

set(CMAKE_BUILD_TYPE Release)
