## CMake file for building a Chaisript-scriptable openBuildNet node.
## - Create a new /build directory, change to that directory and run
##		cmake ..
##   which will create the Makefile file to build your node. You may also generate projects for XCode, MS Visual Studio, etc.

CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0 FATAL_ERROR)

SET(PROJECT_VERSION "0.1"  ## "hg-tip"
    CACHE STRING "${PROJECT_NAME} version string")

## Here comes the name of your project:
SET(PROJECT_NAME nodechai)

## Do not edit this.
PROJECT(${PROJECT_NAME} C CXX)

## Installation directory
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX /usr/local CACHE PATH "OBN install directory" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

## Change OBN_MAIN_DIR to the path to the main directory of openBuildNet
set (OBN_MAIN_DIR ${PROJECT_SOURCE_DIR}/../)

# Include the common CMake code for node.C++
INCLUDE(${OBN_MAIN_DIR}/nodecpp/CMakeCommon.txt)

## To use Boost library
# see http://stackoverflow.com/questions/6646405/how-do-you-add-boost-libraries-in-cmakelists-txt
# and http://www.cmake.org/cmake/help/v3.0/module/FindBoost.html
# Because Boost is required, it will be an error if Boost is not found

# set(Boost_USE_STATIC_LIBS OFF) 
# set(Boost_USE_MULTITHREADED ON)  
# set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.55.0 REQUIRED COMPONENTS filesystem system) 

## Compile with static Chaiscript or dynamic
option(BUILD_STATIC_CHAI "Build with static ChaiScript standard library instead of dynamic one." ON)
if(BUILD_STATIC_CHAI)
    add_definitions(-DNODECHAI_CHAISCRIPT_STATIC)
endif(BUILD_STATIC_CHAI)


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

set(NODECHAI_SRCFILES
	chaiscript_stdlib.cpp
	chaiscript_bindings.cpp
	chaiscript_bindings_eigen.cpp
	chaiscript_io.cpp
	nodechai.cpp
	../thirdparties/csvparser/csvparser.c
	main.cpp
)

set(NODECHAI_HDRFILES
	chaiscript_stdlib.h
	chaiscript_bindings.h
	chaiscript_io.h
	nodechai.h
)

if (WITH_MQTT)
  set(NODECHAI_SRCFILES
	${NODECHAI_SRCFILES}
	nodechai_mqtt.cpp
  )
  set(NODECHAI_HDRFILES
    ${NODECHAI_HDRFILES}
	nodechai_mqtt.h
  )
endif()

if (WITH_YARP)
endif()


if(CMAKE_GENERATOR STREQUAL Xcode)
    set(NODECHAI_SRCFILES ${NODECHAI_SRCFILES} ${NODECHAI_HDRFILES})
endif()

INCLUDE_DIRECTORIES(
  ${Boost_INCLUDE_DIRS}
  ${OBN_MAIN_DIR}/thirdparties
  ${OBN_MAIN_DIR}/thirdparties/chaiscript/include
)


ADD_EXECUTABLE(obnchai
	${OBNNODE_CORE_SRCFILES}
	${NODECHAI_SRCFILES}
)


TARGET_LINK_LIBRARIES(obnchai
	${Boost_LIBRARIES}
	${CMAKE_DL_LIBS}
)


## Make sure that C++ 14 is used (for thread, mutex...)
if(APPLE)
  list( APPEND CMAKE_CXX_FLAGS "-stdlib=libc++ -std=c++14 ${CMAKE_CXX_FLAGS}")
else()
  set_target_properties(obnchai PROPERTIES
      CXX_STANDARD 14
	  CXX_STANDARD_REQUIRED ON)
endif()


## Installation rules
install(TARGETS obnchai
		RUNTIME DESTINATION ${INSTALL_BIN_DIR})
