###############################################################################
# Find Realsense2
#
# This sets the following variables:
# REALSENSE2_FOUND - True if librealsense2 was found.
# REALSENSE2_INCLUDE_DIRS - Directories containing the librealsense2 include files.
# REALSENSE2_LIBRARIES - Libraries needed to use librealsense2.

find_package(PkgConfig)
if(${CMAKE_VERSION} VERSION_LESS 2.8.2)
    pkg_check_modules(PC_LIBREALSENSE2 librealsense2-dev)
else()
    pkg_check_modules(PC_LIBREALSENSE2 QUIET librealsense2-dev)
endif()

set(LIBREALSENSE2_DEFINITIONS ${PC_LIBREALSENSE2_CFLAGS_OTHER})

#add a hint so that it can find it without the pkg-config
find_path(LIBREALSENSE2_INCLUDE_DIR rs.hpp
          HINTS
          ${PC_LIBREALSENSE2_INCLUDEDIR}
          ${PC_LIBREALSENSE2_INCLUDE_DIRS}
          PATHS
            "${PROGRAM_FILES}/librealsense2/Include"
            /usr/include
            /usr/include/librealsense2
            /user/include
            /user/include/librealsense2
          PATH_SUFFIXES librealsense2
)

if(${CMAKE_CL_64})
    set(LIBREALSENSE2_PATH_SUFFIXES lib64)
else()
    set(LIBREALSENSE2_PATH_SUFFIXES lib)
endif()

#add a hint so that it can find it without the pkg-config
find_library(LIBREALSENSE2_LIBRARY
             NAMES librealsense2.so
             HINTS
             ${PC_LIBREALSENSE2_LIBDIR}
             ${PC_LIBREALSENSE2_LIBRARY_DIRS}
             PATHS
               "${PROGRAM_FILES}/librealsense2"
               /usr/lib
               /usr/lib/x86_64-linux-gnu
               /user/lib
)

set(LIBREALSENSE2_INCLUDE_DIRS ${LIBREALSENSE2_INCLUDE_DIR})
set(LIBREALSENSE2_LIBRARIES ${LIBREALSENSE2_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(librealsense2 DEFAULT_MSG
    LIBREALSENSE2_LIBRARY LIBREALSENSE2_INCLUDE_DIR)

mark_as_advanced(LIBREALSENSE2_LIBRARY LIBREALSENSE2_INCLUDE_DIR)
