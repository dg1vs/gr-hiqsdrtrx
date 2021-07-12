INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_HIQSDRTRX hiqsdrtrx)

FIND_PATH(
    HIQSDRTRX_INCLUDE_DIRS
    NAMES hiqsdrtrx/api.h
    HINTS $ENV{HIQSDRTRX_DIR}/include
        ${PC_HIQSDRTRX_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    HIQSDRTRX_LIBRARIES
    NAMES gnuradio-hiqsdrtrx
    HINTS $ENV{HIQSDRTRX_DIR}/lib
        ${PC_HIQSDRTRX_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/hiqsdrtrxTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(HIQSDRTRX DEFAULT_MSG HIQSDRTRX_LIBRARIES HIQSDRTRX_INCLUDE_DIRS)
MARK_AS_ADVANCED(HIQSDRTRX_LIBRARIES HIQSDRTRX_INCLUDE_DIRS)
