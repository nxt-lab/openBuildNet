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
set (OBN_MAIN_DIR ${PROJECT_SOURCE_DIR}/../)

# Include the common CMake code for node.C++
INCLUDE(${OBN_MAIN_DIR}/nodecpp/CMakeCommon.txt)


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


ADD_EXECUTABLE(nodecpp-mynode
	nodemain.cpp
	${OBNNODE_CORE_SRCFILES}
)


# TARGET_LINK_LIBRARIES(nodecpp-mynode
# 	extra libraries here if necessary
# )


INSTALL(
  TARGETS nodecpp-mynode
  RUNTIME DESTINATION ${INSTALL_BIN_DIR}
  COMPONENT bin
)


## Make sure that C++ 11 is used (for thread, mutex...)
if(APPLE)
  list( APPEND CMAKE_CXX_FLAGS "-stdlib=libc++ -std=c++11 ${CMAKE_CXX_FLAGS}")
else()
  set_property(TARGET nodecpp-mynode PROPERTY CXX_STANDARD 11)
  set_property(TARGET nodecpp-mynode PROPERTY CXX_STANDARD_REQUIRED ON)
endif()
