cmake_minimum_required(VERSION 2.8.3)
project(unipi_decentralized)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  turtlesim
  rospy
  rviz
  tf
  urdf
  xacro
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)














# Try to find the CPLEX, Concert, IloCplex and CP Optimizer libraries.
#
# Once done this will add the following imported targets:
#
#  cplex-library - the CPLEX library
#  cplex-concert - the Concert library
#  ilocplex - the IloCplex library
#  cplex-cp - the CP Optimizer library

include(FindPackageHandleStandardArgs)

# Find the path to CPLEX Studio.
# CPLEX Studio 12.4 can be installed in the following default locations:
#   /opt/ibm/ILOG/CPLEX_Studio<edition>124 - Linux
#   /opt/IBM/ILOG/CPLEX_Studio<edition>124 - UNIX
#   ~/Applications/IBM/ILOG/CPLEX_Studio<edition>124 - Mac OS X
#   C:\Program Files\IBM\ILOG\CPLEX_Studio<edition>124 - Windows

  set(CPLEX_ILOG_DIRS /opt/ibm/ILOG /opt/IBM/ILOG)
  if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(CPLEX_ARCH x86-64)
  else ()
    set(CPLEX_ARCH x86)
  endif ()

    set(CPLEX_LIB_PATH_SUFFIXES
      lib/${CPLEX_ARCH}_sles10_4.1/static_pic lib/${CPLEX_ARCH}_linux/static_pic)

if (NOT CPLEX_STUDIO_DIR)
  foreach (dir ${CPLEX_ILOG_DIRS})
    file(GLOB CPLEX_STUDIO_DIRS "${dir}/CPLEX_Studio*")
    list(SORT CPLEX_STUDIO_DIRS)
    list(REVERSE CPLEX_STUDIO_DIRS)
    if (CPLEX_STUDIO_DIRS)
      list(GET CPLEX_STUDIO_DIRS 0 CPLEX_STUDIO_DIR_)
      message(STATUS "Found CPLEX Studio: ${CPLEX_STUDIO_DIR_}")
      break ()
    endif ()
  endforeach ()
  if (NOT CPLEX_STUDIO_DIR_)
    set(CPLEX_STUDIO_DIR_ CPLEX_STUDIO_DIR-NOTFOUND)
  endif ()
  set(CPLEX_STUDIO_DIR ${CPLEX_STUDIO_DIR_} CACHE PATH
    "Path to the CPLEX Studio directory")
endif ()

find_package(Threads)

# ----------------------------------------------------------------------------
# CPLEX

set(CPLEX_DIR ${CPLEX_STUDIO_DIR}/cplex)

# Find the CPLEX include directory.
find_path(CPLEX_INCLUDE_DIR ilcplex/cplex.h PATHS ${CPLEX_DIR}/include)

macro(find_win_cplex_library var path_suffixes)
  foreach (s ${path_suffixes})
    file(GLOB CPLEX_LIBRARY_CANDIDATES "${CPLEX_DIR}/${s}/cplex*.lib")
    if (CPLEX_LIBRARY_CANDIDATES)
      list(GET CPLEX_LIBRARY_CANDIDATES 0 ${var})
      break ()
    endif ()
  endforeach ()
  if (NOT ${var})
    set(${var} NOTFOUND)
  endif ()
endmacro()

# Find the CPLEX library.
find_library(CPLEX_LIBRARY NAMES cplex
    PATHS ${CPLEX_DIR} PATH_SUFFIXES ${CPLEX_LIB_PATH_SUFFIXES})
  set(CPLEX_LIBRARY_DEBUG ${CPLEX_LIBRARY})

# Handle the QUIETLY and REQUIRED arguments and set CPLEX_FOUND to TRUE
# if all listed variables are TRUE.
find_package_handle_standard_args(
  CPLEX DEFAULT_MSG CPLEX_LIBRARY CPLEX_LIBRARY_DEBUG CPLEX_INCLUDE_DIR)

mark_as_advanced(CPLEX_LIBRARY CPLEX_LIBRARY_DEBUG CPLEX_INCLUDE_DIR)

if (CPLEX_FOUND AND NOT TARGET cplex-library)
  set(CPLEX_LINK_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})
  check_library_exists(m floor "" HAVE_LIBM)
  if (HAVE_LIBM)
    set(CPLEX_LINK_LIBRARIES ${CPLEX_LINK_LIBRARIES} m)
  endif ()
  add_library(cplex-library STATIC IMPORTED GLOBAL)
  set_target_properties(cplex-library PROPERTIES
    IMPORTED_LOCATION "${CPLEX_LIBRARY}"
    IMPORTED_LOCATION_DEBUG "${CPLEX_LIBRARY_DEBUG}"
    INTERFACE_INCLUDE_DIRECTORIES "${CPLEX_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${CPLEX_LINK_LIBRARIES}")
endif ()

# ----------------------------------------------------------------------------
# Concert

macro(find_cplex_library var name paths)
  find_library(${var} NAMES ${name}
    PATHS ${paths} PATH_SUFFIXES ${CPLEX_LIB_PATH_SUFFIXES})
  if (UNIX)
    set(${var}_DEBUG ${${var}})
  else ()
    find_library(${var}_DEBUG NAMES ${name}
      PATHS ${paths} PATH_SUFFIXES ${CPLEX_LIB_PATH_SUFFIXES_DEBUG})
  endif ()
endmacro()

set(CPLEX_CONCERT_DIR ${CPLEX_STUDIO_DIR}/concert)

# Find the Concert include directory.
find_path(CPLEX_CONCERT_INCLUDE_DIR ilconcert/ilosys.h
  PATHS ${CPLEX_CONCERT_DIR}/include)

# Find the Concert library.
find_cplex_library(CPLEX_CONCERT_LIBRARY concert ${CPLEX_CONCERT_DIR})

# Handle the QUIETLY and REQUIRED arguments and set CPLEX_CONCERT_FOUND to
# TRUE if all listed variables are TRUE.
find_package_handle_standard_args(
  CPLEX_CONCERT DEFAULT_MSG CPLEX_CONCERT_LIBRARY CPLEX_CONCERT_LIBRARY_DEBUG
  CPLEX_CONCERT_INCLUDE_DIR)

mark_as_advanced(CPLEX_CONCERT_LIBRARY CPLEX_CONCERT_LIBRARY_DEBUG
  CPLEX_CONCERT_INCLUDE_DIR)

if (CPLEX_CONCERT_FOUND AND NOT TARGET cplex-concert)
  add_library(cplex-concert STATIC IMPORTED GLOBAL)
  set_target_properties(cplex-concert PROPERTIES
    IMPORTED_LOCATION "${CPLEX_CONCERT_LIBRARY}"
    IMPORTED_LOCATION_DEBUG "${CPLEX_CONCERT_LIBRARY_DEBUG}"
    INTERFACE_COMPILE_DEFINITIONS IL_STD # Require standard compliance.
    INTERFACE_INCLUDE_DIRECTORIES "${CPLEX_CONCERT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${CMAKE_THREAD_LIBS_INIT}")
endif ()

# ----------------------------------------------------------------------------
# IloCplex - depends on CPLEX and Concert

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-Wno-long-long HAVE_WNO_LONG_LONG_FLAG)
if (HAVE_WNO_LONG_LONG_FLAG)
  # Required if -pedantic is used.
  set(CPLEX_ILOCPLEX_DEFINITIONS -Wno-long-long)
endif ()

# Find the IloCplex include directory - normally the same as the one for CPLEX
# but check if ilocplex.h is there anyway.
find_path(CPLEX_ILOCPLEX_INCLUDE_DIR ilcplex/ilocplex.h
  PATHS ${CPLEX_INCLUDE_DIR})

# Find the IloCplex library.
find_cplex_library(CPLEX_ILOCPLEX_LIBRARY ilocplex ${CPLEX_DIR})

# Handle the QUIETLY and REQUIRED arguments and set CPLEX_ILOCPLEX_FOUND to
# TRUE if all listed variables are TRUE.
find_package_handle_standard_args(
  CPLEX_ILOCPLEX DEFAULT_MSG
  CPLEX_ILOCPLEX_LIBRARY CPLEX_ILOCPLEX_LIBRARY_DEBUG
  CPLEX_ILOCPLEX_INCLUDE_DIR CPLEX_FOUND CPLEX_CONCERT_FOUND)

mark_as_advanced(CPLEX_ILOCPLEX_LIBRARY CPLEX_ILOCPLEX_LIBRARY_DEBUG
  CPLEX_ILOCPLEX_INCLUDE_DIR)

if (CPLEX_ILOCPLEX_FOUND AND NOT TARGET ilocplex)
  add_library(ilocplex STATIC IMPORTED GLOBAL)
  set_target_properties(ilocplex PROPERTIES
    IMPORTED_LOCATION "${CPLEX_ILOCPLEX_LIBRARY}"
    IMPORTED_LOCATION_DEBUG "${CPLEX_ILOCPLEX_LIBRARY_DEBUG}"
    INTERFACE_INCLUDE_DIRECTORIES "${CPLEX_ILOCPLEX_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "cplex-concert;cplex-library")
endif ()



















## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES shadow_algorithm
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${CPLEX_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/shadow_algorithm.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/shadow_algorithm.cpp)
add_executable(kill_turtle_node src/kill_turtle_node.cpp)
# add_executable(yaml_creator src/yaml_creator.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

  target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} 
  ${CPLEX_CONCERT_LIBRARY}
   ${CPLEX_LINK_LIBRARIES}    
   ${CMAKE_THREAD_LIBS_INIT} 
   ${CPLEX_ILOCPLEX_LIBRARY}
   cplex-concert
   cplex-library
   )
target_link_libraries(kill_turtle_node ${catkin_LIBRARIES})
# target_link_libraries(yaml_creator ${catkin_LIBRARIES})

