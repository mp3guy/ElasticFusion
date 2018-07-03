###############################################################################
# Find Realsense2
#
# This sets the following variables:
# REALSENSE2_FOUND - True if librealsense2 was found.
# REALSENSE2_INCLUDE_DIRS - Directories containing the librealsense2 include files.
# REALSENSE2_LIBRARIES - Libraries needed to use librealsense2.

find_package(PkgConfig)
if(${CMAKE_VERSION} VERSION_LESS 2.8.2)
    pkg_check_modules(PC_REALSENSE2 librealsense2-dev)
else()
    pkg_check_modules(PC_REALSENSE2 QUIET librealsense2-dev)
endif()

set(REALSENSE2_DEFINITIONS ${PC_REALSENSE2_CFLAGS_OTHER})

#add a hint so that it can find it without the pkg-config
find_path(REALSENSE2_INCLUDE_DIR rs.hpp
          HINTS
          ${PC_REALSENSE2_INCLUDEDIR}
          ${PC_REALSENSE2_INCLUDE_DIRS}
          PATHS
            "${PROGRAM_FILES}/librealsense2/Include"
            /usr/include
            /usr/include/librealsense2
            /user/include
            /user/include/librealsense2
          PATH_SUFFIXES librealsense2
)

if(${CMAKE_CL_64})
    set(REALSENSE2_PATH_SUFFIXES lib64)
else()
    set(REALSENSE2_PATH_SUFFIXES lib)
endif()

#add a hint so that it can find it without the pkg-config
find_library(REALSENSE2_LIBRARY
             NAMES librealsense2.so
             HINTS
             ${PC_REALSENSE2_LIBDIR}
             ${PC_REALSENSE2_LIBRARY_DIRS}
             PATHS
               "${PROGRAM_FILES}/librealsense2"
               /usr/lib
               /usr/lib/x86_64-linux-gnu
               /user/lib
)

set(REALSENSE2_INCLUDE_DIRS ${REALSENSE2_INCLUDE_DIR})
set(REALSENSE2_LIBRARIES ${REALSENSE2_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(librealsense2 DEFAULT_MSG
    REALSENSE2_LIBRARY REALSENSE2_INCLUDE_DIR)

mark_as_advanced(REALSENSE2_LIBRARY REALSENSE2_INCLUDE_DIR)
