cmake_minimum_required(VERSION 2.8.3)
project(controllers_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_msgs
  sensor_msgs
  realtime_tools
  controller_interface
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/combined_resource_controller.cpp
           #Add below new source files of new controller plug-ins
           # src/new_controller_1.cpp
           # src/new_controller_2.cpp
                                                                )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

install(FILES tutorial_controller_plugins.xml               # This file is needed to build the controller as a plugin library.
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
