## This is the root CMake file for the entire obnsim project.
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0 FATAL_ERROR)

# ON if the root OBNSIM project is being built, so that subprojects can avoid certain duplicate commands
set(OBNSIM_BUILD_ROOT ON)

# Options to build certain parts of the OBNSIM project
option(BUILD_SMNCHAI "Build the Chaiscript-enabled SMN server." ON)
option(BUILD_NODECHAI "Build the Chaiscript-enabled node program." ON)
option(BUILD_APPLICATIONS "Build application programs (e.g., nodes for power networks)." ON)
option(BUILD_TESTS "Build all tests." OFF)

SET(PROJECT_NAME "obnsim")

## Change 'hg-tip' to the current version number of your project if you wish.
## Optionally, you can leave it as is and set PROJECT_VERSION from
## the cmake-gui when you make a release.
## The last parameter is a help string displayed by CMAKE.

SET(PROJECT_VERSION "0.1"  ## "hg-tip"
    CACHE STRING "${PROJECT_NAME} version string")

PROJECT(${PROJECT_NAME})

## This line finds doxygen (for document creation)
# FIND_PACKAGE(Doxygen)


## Common Compiler settings

if(MSVC)
  # Force to always compile with W4
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
  list(APPEND CMAKE_CXX_FLAGS "/w44640 ${CMAKE_CXX_FLAGS}")
## Sometimes MSVC overwhelms you with compiler warnings which are impossible to
## avoid. Then comment out these sections.  
#  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}
#      /wd4250 /wd4355 /wd4503 /wd4800 /wd4996")
# # Suppressed warnings:
# # C4250: 'class1' : inherits 'class2::member' via dominance
# # C4355: 'this' : used in base member initializer list
# # C4503: 'function' : decorated name length exceeded, name was truncated
# # C4800: 'type' : forcing value to bool 'true' or 'false'
# #        (performance warning)
# # C4996: 'function': was declared deprecated
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  # Update if necessary
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wshadow -Wnon-virtual-dtor -pedantic")  # -Wextra
endif()


# Select the desired build type
set(CMAKE_BUILD_TYPE Release)


## Add the subprojects

# ADD_SUBDIRECTORY(smn)

# ADD_SUBDIRECTORY(nodecpp)
ADD_SUBDIRECTORY(tests/test1)
ADD_SUBDIRECTORY(tests/test2)
ADD_SUBDIRECTORY(tests/test3)


