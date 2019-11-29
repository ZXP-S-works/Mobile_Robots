# Install script for directory: /home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/src/car_model/cadillac

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/build/car_model/cadillac/catkin_generated/installspace/cadillac.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cadillac/cmake" TYPE FILE FILES
    "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/build/car_model/cadillac/catkin_generated/installspace/cadillacConfig.cmake"
    "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/build/car_model/cadillac/catkin_generated/installspace/cadillacConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cadillac" TYPE FILE FILES "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/src/car_model/cadillac/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cadillac/config" TYPE DIRECTORY FILES "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/src/car_model/cadillac/config/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cadillac/launch" TYPE DIRECTORY FILES "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/src/car_model/cadillac/launch/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cadillac/meshes" TYPE DIRECTORY FILES "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/src/car_model/cadillac/meshes/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cadillac/urdf" TYPE DIRECTORY FILES "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/src/car_model/cadillac/urdf/")
endif()

