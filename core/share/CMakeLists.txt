cmake_minimum_required (VERSION 3.10)
project (saw_galil_controller_config VERSION 0.0.1)

find_package (catkin QUIET)
if (catkin_FOUND)
  catkin_package()
endif ()

install (
  DIRECTORY "${saw_galil_controller_config_SOURCE_DIR}"
  DESTINATION share/sawGalilController
  COMPONENT mtsGalilController-Share
  PATTERN "CMakeLists.txt" EXCLUDE
  PATTERN "package.xml" EXCLUDE)
