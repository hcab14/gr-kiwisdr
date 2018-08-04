INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_KIWISDR kiwisdr)

FIND_PATH(
    KIWISDR_INCLUDE_DIRS
    NAMES kiwisdr/api.h
    HINTS $ENV{KIWISDR_DIR}/include
        ${PC_KIWISDR_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    KIWISDR_LIBRARIES
    NAMES gnuradio-kiwisdr
    HINTS $ENV{KIWISDR_DIR}/lib
        ${PC_KIWISDR_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(KIWISDR DEFAULT_MSG KIWISDR_LIBRARIES KIWISDR_INCLUDE_DIRS)
MARK_AS_ADVANCED(KIWISDR_LIBRARIES KIWISDR_INCLUDE_DIRS)

