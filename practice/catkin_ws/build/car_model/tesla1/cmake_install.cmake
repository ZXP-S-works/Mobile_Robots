# Install script for directory: /home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/src/car_model/tesla1

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/build/car_model/tesla1/catkin_generated/installspace/tesla1.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tesla1/cmake" TYPE FILE FILES
    "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/build/car_model/tesla1/catkin_generated/installspace/tesla1Config.cmake"
    "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/build/car_model/tesla1/catkin_generated/installspace/tesla1Config-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tesla1" TYPE FILE FILES "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/src/car_model/tesla1/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tesla1/config" TYPE DIRECTORY FILES "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/src/car_model/tesla1/config/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tesla1/launch" TYPE DIRECTORY FILES "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/src/car_model/tesla1/launch/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tesla1/meshes" TYPE DIRECTORY FILES "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/src/car_model/tesla1/meshes/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tesla1/urdf" TYPE DIRECTORY FILES "/home/zxp-s-works/Desktop/Mobile_Rob/practice/catkin_ws/src/car_model/tesla1/urdf/")
endif()

