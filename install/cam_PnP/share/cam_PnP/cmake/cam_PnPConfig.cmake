# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cam_PnP_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cam_PnP_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cam_PnP_FOUND FALSE)
  elseif(NOT cam_PnP_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cam_PnP_FOUND FALSE)
  endif()
  return()
endif()
set(_cam_PnP_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cam_PnP_FIND_QUIETLY)
  message(STATUS "Found cam_PnP: 0.0.0 (${cam_PnP_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cam_PnP' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cam_PnP_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cam_PnP_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cam_PnP_DIR}/${_extra}")
endforeach()
