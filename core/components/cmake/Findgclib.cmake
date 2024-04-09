# (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# Findgclib
#
# Find the Galil Controller Library (gclib). It is sufficient to set gclib_INCLUDE_DIR
# because the other files should then be found automatically.
#
#    gclib_ROOT            -- root directory for library
#    gclib_INCLUDE_DIR     -- path to header files
#    gclib_LIBRARY_DIR     -- path to library files
#    gclib_LIBRARIES       -- list of library names
#    gclib_FOUND           -- true if package found
#
# Also sets the following:
#    gclib_LIBRARY_gclib   -- full path to gclib library
#    gclib_LIBRARY_gclibo  -- full path to gclibo library

set (gclib_FOUND FALSE)
set (gclib_LIBRARIES "")

find_path (gclib_INCLUDE_DIR NAMES gclib.h
           DOC "Directory for gclib header files")

if (gclib_INCLUDE_DIR)
  get_filename_component(gclib_ROOT ${gclib_INCLUDE_DIR} DIRECTORY)

  # Determine whether to look for 32-bit or 64-bit libraries
  if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(gclib_ARCH "64")
  else ()
    set(gclib_ARCH "32")
  endif ()

  find_library (gclib_LIBRARY_GCLIB  NAMES gclib
                DOC "Galil gclib library"
                HINTS "${gclib_ROOT}/lib"
		PATH_SUFFIXES "dynamic/x${gclib_ARCH}")

  if (gclib_LIBRARY_GCLIB)
    get_filename_component(gclib_NAME_GCLIB ${gclib_LIBRARY_GCLIB} NAME)
    set (gclib_LIBRARIES ${gclib_LIBRARIES} ${gclib_NAME_GCLIB})
    unset (gclib_NAME_GCLIB)
    get_filename_component(gclib_LIBRARY_DIR ${gclib_LIBRARY_GCLIB} DIRECTORY)

    find_library (gclib_LIBRARY_GCLIBO NAMES gclibo
                  DOC "Galil gclibo library"
                  PATHS ${gclib_LIBRARY_DIR})
    if (gclib_LIBRARY_GCLIBO)
      get_filename_component(gclib_NAME_GCLIBO ${gclib_LIBRARY_GCLIBO} NAME)
      set (gclib_LIBRARIES ${gclib_LIBRARIES} ${gclib_NAME_GCLIBO})
      unset (gclib_NAME_GCLIBO)
      set (gclib_FOUND TRUE)
    endif ()
  endif ()
endif ()
