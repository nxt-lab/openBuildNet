## This builds the test project, including source files from the SMN and the node frameworks.

CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0 FATAL_ERROR)

## Here comes the name of your project:
SET(PROJECT_NAME "test4")

PROJECT(${PROJECT_NAME})

## Change OBN_MAIN_DIR to the path to the main directory of openBuildNet
set (OBN_MAIN_DIR ${PROJECT_SOURCE_DIR}/../../)

# Include the common CMake code for node.C++
INCLUDE(${OBN_MAIN_DIR}/nodecpp/CMakeCommon.txt)

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


ADD_EXECUTABLE(source1
	source1.cpp
	${OBNNODE_CORE_SRCFILES}
)

ADD_EXECUTABLE(source2
	source2.cpp
	${OBNNODE_CORE_SRCFILES}
)

ADD_EXECUTABLE(receiver
	receiver.cpp
	${OBNNODE_CORE_SRCFILES}
)


## Make sure that C++ 11 is used (for thread, mutex...)
if(APPLE)
  list( APPEND CMAKE_CXX_FLAGS "-stdlib=libc++ -std=c++11 ${CMAKE_CXX_FLAGS}")
else()
  set_property(TARGET source1 source2 receiver PROPERTY CXX_STANDARD 11)
  set_property(TARGET source1 source2 receiver PROPERTY CXX_STANDARD_REQUIRED ON)
endif()
