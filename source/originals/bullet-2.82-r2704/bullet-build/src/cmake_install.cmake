# Install script for directory: /home/delpart/ba/bullet-2.82-r2704/src

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "0")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "/usr/local/include/bullet" TYPE FILE FILES
    "/home/delpart/ba/bullet-2.82-r2704/src/btBulletCollisionCommon.h"
    "/home/delpart/ba/bullet-2.82-r2704/src/btBulletDynamicsCommon.h"
    "/home/delpart/ba/bullet-2.82-r2704/src/Bullet-C-Api.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "/usr/local/include/bullet/vectormath" TYPE FILE FILES "/home/delpart/ba/bullet-2.82-r2704/src/vectormath/vmInclude.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "/usr/local/include/bullet/vectormath/scalar" TYPE FILE FILES
    "/home/delpart/ba/bullet-2.82-r2704/src/vectormath/scalar/boolInVec.h"
    "/home/delpart/ba/bullet-2.82-r2704/src/vectormath/scalar/floatInVec.h"
    "/home/delpart/ba/bullet-2.82-r2704/src/vectormath/scalar/mat_aos.h"
    "/home/delpart/ba/bullet-2.82-r2704/src/vectormath/scalar/quat_aos.h"
    "/home/delpart/ba/bullet-2.82-r2704/src/vectormath/scalar/vec_aos.h"
    "/home/delpart/ba/bullet-2.82-r2704/src/vectormath/scalar/vectormath_aos.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "/usr/local/include/bullet/vectormath/sse" TYPE FILE FILES
    "/home/delpart/ba/bullet-2.82-r2704/src/vectormath/sse/boolInVec.h"
    "/home/delpart/ba/bullet-2.82-r2704/src/vectormath/sse/floatInVec.h"
    "/home/delpart/ba/bullet-2.82-r2704/src/vectormath/sse/mat_aos.h"
    "/home/delpart/ba/bullet-2.82-r2704/src/vectormath/sse/quat_aos.h"
    "/home/delpart/ba/bullet-2.82-r2704/src/vectormath/sse/vec_aos.h"
    "/home/delpart/ba/bullet-2.82-r2704/src/vectormath/sse/vecidx_aos.h"
    "/home/delpart/ba/bullet-2.82-r2704/src/vectormath/sse/vectormath_aos.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/delpart/ba/bullet-2.82-r2704/bullet-build/src/BulletSoftBody/cmake_install.cmake")
  INCLUDE("/home/delpart/ba/bullet-2.82-r2704/bullet-build/src/BulletCollision/cmake_install.cmake")
  INCLUDE("/home/delpart/ba/bullet-2.82-r2704/bullet-build/src/BulletDynamics/cmake_install.cmake")
  INCLUDE("/home/delpart/ba/bullet-2.82-r2704/bullet-build/src/LinearMath/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

