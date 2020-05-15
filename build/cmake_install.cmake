# Install script for directory: /home/dexter/my_workspace/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/dexter/my_workspace/install")
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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/dexter/my_workspace/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/dexter/my_workspace/install" TYPE PROGRAM FILES "/home/dexter/my_workspace/build/catkin_generated/installspace/_setup_util.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/dexter/my_workspace/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/dexter/my_workspace/install" TYPE PROGRAM FILES "/home/dexter/my_workspace/build/catkin_generated/installspace/env.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/dexter/my_workspace/install/setup.bash;/home/dexter/my_workspace/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/dexter/my_workspace/install" TYPE FILE FILES
    "/home/dexter/my_workspace/build/catkin_generated/installspace/setup.bash"
    "/home/dexter/my_workspace/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/dexter/my_workspace/install/setup.sh;/home/dexter/my_workspace/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/dexter/my_workspace/install" TYPE FILE FILES
    "/home/dexter/my_workspace/build/catkin_generated/installspace/setup.sh"
    "/home/dexter/my_workspace/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/dexter/my_workspace/install/setup.zsh;/home/dexter/my_workspace/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/dexter/my_workspace/install" TYPE FILE FILES
    "/home/dexter/my_workspace/build/catkin_generated/installspace/setup.zsh"
    "/home/dexter/my_workspace/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/dexter/my_workspace/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/dexter/my_workspace/install" TYPE FILE FILES "/home/dexter/my_workspace/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/dexter/my_workspace/build/gtest/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/gazebo_ros_pkgs/gazebo_dev/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/gazebo_ros_pkgs/gazebo_ros_pkgs/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/husky/husky_desktop/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/husky/husky_robot/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/husky/husky_simulator/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/octomap_mapping/octomap_mapping/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/husky/husky_msgs/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/teleop_twist_keyboard/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/turtlebot3_msgs/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/turtlebot3_navigation/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/turtlebot3_simulations/turtlebot3_simulations/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/main/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/husky/husky_bringup/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/husky/husky_control/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/husky/husky_description/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/husky/husky_gazebo/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/husky/husky_navigation/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/husky/husky_viz/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/gazebo_ros_pkgs/gazebo_msgs/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/husky/husky_base/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/LMS1xx/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/interactive_marker_twist_server/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/octomap_mapping/octomap_server/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/turtlebot3_bringup/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/turtlebot3_simulations/turtlebot3_fake/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/turtlebot3_simulations/turtlebot3_gazebo/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/turtlebot3_slam/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/twist_mux/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/gazebo_ros_pkgs/gazebo_ros_control/cmake_install.cmake")
  include("/home/dexter/my_workspace/build/turtlebot3_description/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/dexter/my_workspace/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
