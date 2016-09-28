# - Try to find SUITESPARSE
# Once done this will define
#  
#  SUITESPARSE_FOUND            - system has SUITESPARSE
#  SUITESPARSE_INCLUDE_DIRS     - the SUITESPARSE include directory
#  SUITESPARSE_LIBRARIES        - Link these to use SUITESPARSE
#  SUITESPARSE_SPQR_LIBRARY     - name of spqr library (necessary due to error in debian package)
#  SUITESPARSE_SPQR_LIBRARY_DIR - name of spqr library (necessary due to error in debian package)
#  SUITESPARSE_LIBRARY_DIR      - Library main directory containing suitesparse libs
#  SUITESPARSE_LIBRARY_DIRS     - all Library directories containing suitesparse libs
#  SUITESPARSE_SPQR_VALID       - automatic identification whether or not spqr package is installed correctly

IF (SUITESPARSE_INCLUDE_DIRS)
  # Already in cache, be silent
  SET(SUITESPARSE_FIND_QUIETLY TRUE)
ENDIF (SUITESPARSE_INCLUDE_DIRS)

FIND_PATH( CHOLMOD_INCLUDE_DIR cholmod.h
	      PATHS /usr/local/include 
	            /usr/include 
	            /usr/include/suitesparse/ 
	            ${CMAKE_SOURCE_DIR}/MacOS/Libs/cholmod
      	      PATH_SUFFIXES cholmod/ CHOLMOD/ )
   	
FIND_PATH( SUITESPARSE_LIBRARY_DIR
          NAMES libcholmod.so libcholmod.a
          PATHS /usr/lib 
                /usr/lib64
                /usr/lib/x86_64-linux-gnu
                /usr/lib/i386-linux-gnu
                /usr/local/lib )

   # Add cholmod include directory to collection include directories
   IF ( CHOLMOD_INCLUDE_DIR )
	list ( APPEND SUITESPARSE_INCLUDE_DIRS ${CHOLMOD_INCLUDE_DIR} )
   ENDIF( CHOLMOD_INCLUDE_DIR )

   # if we found the library, add it to the defined libraries
   IF ( SUITESPARSE_LIBRARY_DIR )
		FIND_LIBRARY( SUITESPARSE_AMD_LIBRARY
                     NAMES libamd
                     PATHS ${SUITESPARSE_LIBRARY_DIR} )
		FIND_LIBRARY( SUITESPARSE_CAMD_LIBRARY
                     NAMES libcamd
                     PATHS ${SUITESPARSE_LIBRARY_DIR} )
		FIND_LIBRARY( SUITESPARSE_CCOLAMD_LIBRARY
                     NAMES libccolamd
                     PATHS ${SUITESPARSE_LIBRARY_DIR} )
		FIND_LIBRARY( SUITESPARSE_CHOLMOD_LIBRARY
                     NAMES libcholmod
                     PATHS ${SUITESPARSE_LIBRARY_DIR} )
		FIND_LIBRARY( SUITESPARSE_COLAMD_LIBRARY
                     NAMES libcolamd
                     PATHS ${SUITESPARSE_LIBRARY_DIR} )
		FIND_LIBRARY( SUITESPARSE_CXSPARSE_LIBRARY
                     NAMES libcxsparse
                     PATHS ${SUITESPARSE_LIBRARY_DIR} )
		FIND_LIBRARY( SUITESPARSE_SUITESPARSECONFIG_LIBRARY
                     NAMES suitesparseconfig
                     PATHS ${SUITESPARSE_LIBRARY_DIR} )
		FIND_LIBRARY( SUITESPARSE_BLAS_LIBRARY
                     NAMES liblapack
                     PATHS ${SUITESPARSE_LIBRARY_DIR}/lapack_blas_windows )
		FIND_LIBRARY( SUITESPARSE_LAPACK_LIBRARY
                     NAMES libblas
                     PATHS ${SUITESPARSE_LIBRARY_DIR}/lapack_blas_windows )
					 
					 
       list ( APPEND SUITESPARSE_LIBRARIES ${SUITESPARSE_AMD_LIBRARY})
       list ( APPEND SUITESPARSE_LIBRARIES ${SUITESPARSE_CAMD_LIBRARY})
       list ( APPEND SUITESPARSE_LIBRARIES ${SUITESPARSE_CCOLAMD_LIBRARY})
       list ( APPEND SUITESPARSE_LIBRARIES ${SUITESPARSE_CHOLMOD_LIBRARY})
       list ( APPEND SUITESPARSE_LIBRARIES ${SUITESPARSE_COLAMD_LIBRARY})
       list ( APPEND SUITESPARSE_LIBRARIES ${SUITESPARSE_CXSPARSE_LIBRARY})
       list ( APPEND SUITESPARSE_LIBRARIES ${SUITESPARSE_SUITESPARSECONFIG_LIBRARY})
       list ( APPEND SUITESPARSE_LIBRARIES ${SUITESPARSE_BLAS_LIBRARY})
       list ( APPEND SUITESPARSE_LIBRARIES ${SUITESPARSE_LAPACK_LIBRARY})

       # Metis and spqr are optional
       FIND_LIBRARY( SUITESPARSE_METIS_LIBRARY
                     NAMES metis
                     PATHS ${SUITESPARSE_LIBRARY_DIR} )
       IF (SUITESPARSE_METIS_LIBRARY)			
	  list ( APPEND SUITESPARSE_LIBRARIES ${SUITESPARSE_METIS_LIBRARY})
       ENDIF(SUITESPARSE_METIS_LIBRARY)

       if(EXISTS  "${CHOLMOD_INCLUDE_DIR}/SuiteSparseQR.hpp")
	  SET(SUITESPARSE_SPQR_VALID TRUE CACHE BOOL "SuiteSparseSPQR valid")
       else()
	  SET(SUITESPARSE_SPQR_VALID false CACHE BOOL "SuiteSparseSPQR valid")
       endif()

       if(SUITESPARSE_SPQR_VALID)
	  FIND_LIBRARY( SUITESPARSE_SPQR_LIBRARY
		      NAMES libspqr
		      PATHS ${SUITESPARSE_LIBRARY_DIR} )
	  IF (SUITESPARSE_SPQR_LIBRARY)			
	    list ( APPEND SUITESPARSE_LIBRARIES ${SUITESPARSE_SPQR_LIBRARY})
	  ENDIF (SUITESPARSE_SPQR_LIBRARY)
       endif()
    ENDIF( SUITESPARSE_LIBRARY_DIR )  
   
IF (SUITESPARSE_INCLUDE_DIRS AND SUITESPARSE_LIBRARIES)
   SET(SUITESPARSE_FOUND TRUE)
   MESSAGE(STATUS "Found SuiteSparse")
ELSE (SUITESPARSE_INCLUDE_DIRS AND SUITESPARSE_LIBRARIES)
   SET( SUITESPARSE_FOUND FALSE )
   MESSAGE(FATAL_ERROR "Unable to find SuiteSparse")
ENDIF (SUITESPARSE_INCLUDE_DIRS AND SUITESPARSE_LIBRARIES)
