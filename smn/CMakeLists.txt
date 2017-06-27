## IMPORTANT NOTE:
## - All source files are in /src directory; to include them, prefix with ${PROJECT_SOURCE_DIR}/
## - All include files are in /include directory; normally, that directory is already included; however, if you want to specify a .h file explicitly, prefix it with ${PROJECT_INCLUDE_DIR}/

CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0 FATAL_ERROR)

## Here comes the name of your project:
SET(PROJECT_NAME "obnsmn")

SET(PROJECT_VERSION "0.1"  ## "hg-tip"
    CACHE STRING "${PROJECT_NAME} version string")

## Do not edit this.
PROJECT(${PROJECT_NAME} C CXX)

## Installation directory
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX /usr/local/obn CACHE PATH "OBN install directory" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

# Select the desired build type
# set(CMAKE_BUILD_TYPE Release)


## Set OBN_MAIN_DIR to the path to the main directory of openBuildNet
set (OBN_MAIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../)

set (PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set (PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

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
option(WITH_YARP "Build SMNChai with YARP support for communication." OFF)
if(WITH_YARP)
  find_package(YARP REQUIRED)
  include_directories(${YARP_INCLUDE_DIRS})
  link_libraries(${YARP_LIBRARIES})
  set(OBNSMN_COMM_SRC ${OBNSMN_COMM_SRC} ${PROJECT_SOURCE_DIR}/obnsmn_comm_yarp.cpp)
  set(OBNSMN_COMM_HDR ${OBNSMN_COMM_HDR} ${PROJECT_INCLUDE_DIR}/obnsmn_comm_yarp.h)
  add_definitions(-DOBNSIM_COMM_YARP)
  message("Found YARP library; will build SMN with YARP support.")
endif(WITH_YARP)


## To use MQTT (optionally)
option(WITH_PAHOMQTT "Build SMNChai with Paho MQTT support for communication." ON)
if(WITH_PAHOMQTT)
  find_library(PAHOMQTT3A NAMES paho-mqtt3a)
  if(PAHOMQTT3A_NOTFOUND)
    # Use the local source files (submodule)
    message("Did not find Paho MQTT3 library in the system; will use the local source files. This may bloat your program, so it's best to install Paho into your system before building SMNChai.")

    find_package(OpenSSL REQUIRED)
    AUX_SOURCE_DIRECTORY(${OBN_MAIN_DIR}/thirdparties/paho/src PAHOMQTT3A_SRC)
    INCLUDE_DIRECTORIES(${OBN_MAIN_DIR}/thirdparties/paho/src)
    set(OBNSMN_COMM_SRC ${OBNSMN_COMM_SRC} ${PAHOMQTT3A_SRC})
  else()
    message("Found Paho MQTT3 library in the system; will use this library.")
    link_libraries(${PAHOMQTT3A})
  endif()
  add_definitions(-DOBNSIM_COMM_MQTT)
  set(OBNSMN_COMM_SRC ${OBNSMN_COMM_SRC} ${PROJECT_SOURCE_DIR}/obnsmn_comm_mqtt.cpp)
  set(OBNSMN_COMM_HDR ${OBNSMN_COMM_HDR} ${PROJECT_INCLUDE_DIR}/obnsmn_comm_mqtt.h)
endif(WITH_PAHOMQTT)


## These are the include directories used by the compiler.

INCLUDE_DIRECTORIES(
  ${PROJECT_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}
  ${OBNSIM_INCLUDE_DIR}
)


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
	${OBNSIM_INCLUDE_DIR}/obnsim_basic.cpp	
	${PROTO_SRCS}
	${OBNSMN_COMM_SRC}
)

set(OBNSMN_CORE_HDRFILES
	${PROJECT_INCLUDE_DIR}/obnsmn_basic.h
	${PROJECT_INCLUDE_DIR}/obnsmn_event.h
	${PROJECT_INCLUDE_DIR}/obnsmn_gc.h
	${PROJECT_INCLUDE_DIR}/obnsmn_gc_inline.h
	${PROJECT_INCLUDE_DIR}/obnsmn_node.h
	${PROJECT_INCLUDE_DIR}/obnsmn_nodegraph.h
	${PROJECT_INCLUDE_DIR}/sharedqueue.h
	${PROJECT_INCLUDE_DIR}/obnsmn_report.h
	${PROJECT_INCLUDE_DIR}/obnsmnlib.h
	${PROJECT_INCLUDE_DIR}/obnsmnlib_system.h
	${OBNSIM_INCLUDE_DIR}/obnsim_basic.h
	${PROTO_HDRS}
	${OBNSMN_COMM_HDR}
)

## When we generate Xcode projects, we need to include both the C++ and H files so that they will all be included in the projects.
## It's unnecessary for Makefile.

if(CMAKE_GENERATOR STREQUAL Xcode)
    set(OBNSMN_CORE_SRCFILES ${OBNSMN_CORE_SRCFILES} ${OBNSMN_CORE_HDRFILES})
endif()


# ADD_EXECUTABLE(obnsim-smn
# 	${PROJECT_SOURCE_DIR}/main.cpp
# 	${OBNSMN_CORE_SRCFILES}
# )

ADD_LIBRARY(obnsmn SHARED
	${PROJECT_SOURCE_DIR}/obnsmnlib.cpp
	${PROJECT_SOURCE_DIR}/obnsmnlib_system.cpp
	${OBNSMN_CORE_SRCFILES}
)


## Make sure that C++ 11 is used (for thread, mutex...)
if(APPLE)
  list( APPEND CMAKE_CXX_FLAGS "-stdlib=libc++ -std=c++11 ${CMAKE_CXX_FLAGS}")
else()
  set_target_properties(obnsmn PROPERTIES
      CXX_STANDARD 11
	  CXX_STANDARD_REQUIRED ON)
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
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wshadow -Wnon-virtual-dtor -pedantic")  # -Wextra
endif()


TARGET_LINK_LIBRARIES(obnsmn
  ${PROTOBUF_LITE_LIBRARIES}
  ${Boost_LIBRARIES}
)
#  ${CMAKE_THREAD_LIBS_INIT}
#  ${CMAKE_DL_LIBS}



# INSTALL(
#   TARGETS obnsim-smn
#   RUNTIME DESTINATION ${INSTALL_BIN_DIR}
#   COMPONENT bin
# )


## Installation rules
install(TARGETS obnsmn
		RUNTIME DESTINATION ${INSTALL_BIN_DIR}
        LIBRARY DESTINATION lib)
install(FILES include/obnsmnlib.h DESTINATION include)


## The auxiliary doxygen files (.dox) should be placed in the 'doc'
## subdirectory. The next line includes the CMAKE config of that directory.

# ADD_SUBDIRECTORY(doc)
