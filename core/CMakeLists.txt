#
# (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required(VERSION 3.10)
project (sawGalilControllerCore VERSION 0.0.1)

# to locate cisst macros
find_package (cisst REQUIRED cisstCommon)
include (${CISST_USE_FILE})

cisst_cpack_settings (
  VENDOR "JHU"
  MAINTAINER "anton.deguet@jhu.edu")

add_subdirectory (components)

set (sawGalilController_DIR "${sawGalilControllerCore_BINARY_DIR}/components")
add_subdirectory (examples)
add_subdirectory (share)

include (CPack)
cpack_add_component (sawGalilController)
cpack_add_component (sawGalilController-dev
                     DEPENDS sawGalilController)
cpack_add_component (sawGalilController-Examples
                     DEPENDS sawGalilController)
cpack_add_component (sawGalilController-Share
                     DEPENDS sawGalilController sawGalilController-Share)
