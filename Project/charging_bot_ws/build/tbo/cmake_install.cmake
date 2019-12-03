# Install script for directory: /home/zxp-s-works/Desktop/Mobile_Rob/Project/charging_bot_ws/src/tbo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zxp-s-works/Desktop/Mobile_Rob/Project/charging_bot_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zxp-s-works/Desktop/Mobile_Rob/Project/charging_bot_ws/build/tbo/catkin_generated/installspace/tbo.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tbo/cmake" TYPE FILE FILES
    "/home/zxp-s-works/Desktop/Mobile_Rob/Project/charging_bot_ws/build/tbo/catkin_generated/installspace/tboConfig.cmake"
    "/home/zxp-s-works/Desktop/Mobile_Rob/Project/charging_bot_ws/build/tbo/catkin_generated/installspace/tboConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tbo" TYPE FILE FILES "/home/zxp-s-works/Desktop/Mobile_Rob/Project/charging_bot_ws/src/tbo/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tbo/config" TYPE DIRECTORY FILES "/home/zxp-s-works/Desktop/Mobile_Rob/Project/charging_bot_ws/src/tbo/config/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tbo/launch" TYPE DIRECTORY FILES "/home/zxp-s-works/Desktop/Mobile_Rob/Project/charging_bot_ws/src/tbo/launch/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tbo/meshes" TYPE DIRECTORY FILES "/home/zxp-s-works/Desktop/Mobile_Rob/Project/charging_bot_ws/src/tbo/meshes/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tbo/urdf" TYPE DIRECTORY FILES "/home/zxp-s-works/Desktop/Mobile_Rob/Project/charging_bot_ws/src/tbo/urdf/")
endif()

