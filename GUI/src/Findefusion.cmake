###############################################################################
# Find efusion
#
# This sets the following variables:
# EFUSION_FOUND - True if EFUSION was found.
# EFUSION_INCLUDE_DIRS - Directories containing the EFUSION include files.
# EFUSION_LIBRARIES - Libraries needed to use EFUSION.

find_path(EFUSION_INCLUDE_DIR ElasticFusion.h
          PATHS
            ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/src
          PATH_SUFFIXES Core
)

find_library(EFUSION_LIBRARY
             NAMES libefusion.so
             PATHS
               ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/build
               ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/src/build
             PATH_SUFFIXES ${EFUSION_PATH_SUFFIXES}
)

set(EFUSION_INCLUDE_DIRS ${EFUSION_INCLUDE_DIR})
set(EFUSION_LIBRARIES ${EFUSION_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(EFUSION DEFAULT_MSG EFUSION_LIBRARY EFUSION_INCLUDE_DIR)
mark_as_advanced(EFUSION_LIBRARY EFUSION_INCLUDE_DIR)
