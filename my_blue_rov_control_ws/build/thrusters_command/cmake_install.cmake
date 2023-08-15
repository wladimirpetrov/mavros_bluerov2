# Install script for directory: /local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/src/thrusters_command

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install" TYPE PROGRAM FILES "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/build/thrusters_command/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install" TYPE PROGRAM FILES "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/build/thrusters_command/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install/setup.bash;/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install" TYPE FILE FILES
    "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/build/thrusters_command/catkin_generated/installspace/setup.bash"
    "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/build/thrusters_command/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install/setup.sh;/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install" TYPE FILE FILES
    "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/build/thrusters_command/catkin_generated/installspace/setup.sh"
    "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/build/thrusters_command/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install/setup.zsh;/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install" TYPE FILE FILES
    "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/build/thrusters_command/catkin_generated/installspace/setup.zsh"
    "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/build/thrusters_command/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/install" TYPE FILE FILES "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/build/thrusters_command/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/build/thrusters_command/catkin_generated/installspace/thrusters_command.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/thrusters_command/cmake" TYPE FILE FILES
    "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/build/thrusters_command/catkin_generated/installspace/thrusters_commandConfig.cmake"
    "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/build/thrusters_command/catkin_generated/installspace/thrusters_commandConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/thrusters_command" TYPE FILE FILES "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/src/thrusters_command/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/build/thrusters_command/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/local/vol00/home/vpetrov/dev/mavros_bluerov2/my_blue_rov_control_ws/build/thrusters_command/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
