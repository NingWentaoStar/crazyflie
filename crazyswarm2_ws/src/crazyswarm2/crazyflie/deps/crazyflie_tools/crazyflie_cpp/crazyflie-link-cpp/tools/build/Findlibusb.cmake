find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_LIBUSB REQUIRED libusb-1.0)

find_path(libusb_INCLUDE_DIR
  NAMES libusb.h
  PATHS ${PC_LIBUSB_INCLUDE_DIRS}
)

find_library(libusb_LIBRARY
  NAMES usb-1.0
  PATHS ${PC_LIBUSB_LIBRARY_DIRS}
)

set(LIBUSB_INCLUDE_DIR ${libusb_INCLUDE_DIR})
set(LIBUSB_LIBRARY ${libusb_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(libusb REQUIRED_VARS libusb_INCLUDE_DIR libusb_LIBRARY)

if(NOT TARGET libusb)
  add_library(libusb UNKNOWN IMPORTED)
  set_target_properties(libusb PROPERTIES
    IMPORTED_LOCATION "${libusb_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${libusb_INCLUDE_DIR}"
  )
endif()
