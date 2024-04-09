#
# (C) Copyright 2016-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawGalilController VERSION 0.1.0)

# So we can use Findgclib.cmake
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes)

find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  if (NOT CISST_HAS_JSON)
    message( FATAL_ERROR "sawGalilController requires cisst to be built with JSON" )
  endif ()

  # catkin/ROS paths
  cisst_set_output_path ()


  # Find Galil gclib but don't fail if not found
  find_package (gclib)

  if (gclib_FOUND)

    include_directories (${gclib_INCLUDE_DIR})
    link_directories (${gclib_LIBRARY_DIR})

    set (sawGalilController_INCLUDE_DIR
      "${sawGalilController_SOURCE_DIR}/include"
      "${sawGalilController_BINARY_DIR}/include")
    set (sawGalilController_HEADER_DIR "${sawGalilController_SOURCE_DIR}/include/sawGalilController")
    set (sawGalilController_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}" "${gclib_LIBRARY_DIR}")
    set (sawGalilController_LIBRARIES sawGalilController)

    include_directories (BEFORE ${sawGalilController_INCLUDE_DIR})

    # add all config files for this component
    cisst_add_config_files (sawGalilController)

    # create data type using the data generator
    cisst_data_generator (sawGalilController
      "${sawGalilController_BINARY_DIR}/include"
      "sawGalilController/"
      code/sawGalilControllerConfig.cdg)

    set (sawGalilController_HEADER_FILES
      "${sawGalilController_HEADER_DIR}/mtsGalilController.h"
      "${sawGalilController_HEADER_DIR}/sawGalilControllerExport.h"
      ${sawGalilController_CISST_DG_HDRS})

    set (sawGalilController_SOURCE_FILES
      code/mtsGalilController.cpp
      ${sawGalilController_CISST_DG_SRCS})

    add_library (
      sawGalilController
      ${IS_SHARED}
      ${sawGalilController_HEADER_FILES}
      ${sawGalilController_SOURCE_FILES})

    set_target_properties (
      sawGalilController PROPERTIES
      VERSION ${sawGalilController_VERSION}
      FOLDER "sawGalilController")

    target_link_libraries (
      sawGalilController
      ${gclib_LIBRARIES})

    cisst_target_link_libraries (
      sawGalilController
      ${REQUIRED_CISST_LIBRARIES})

    # Install target for headers and library
    install (
      DIRECTORY
      "${sawGalilController_SOURCE_DIR}/include/sawGalilController"
      "${sawGalilController_BINARY_DIR}/include/sawGalilController"
      DESTINATION include
      COMPONENT sawGalilController-dev)

    install (
      TARGETS sawGalilController
      COMPONENT sawGalilController
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib)

  else (gclib_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires the Galil SDK")
  endif (gclib_FOUND)
else (cisst_FOUND_AS_REQUIRED)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
