# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_edu_robocup_rescue_stack_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED edu_robocup_rescue_stack_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(edu_robocup_rescue_stack_FOUND FALSE)
  elseif(NOT edu_robocup_rescue_stack_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(edu_robocup_rescue_stack_FOUND FALSE)
  endif()
  return()
endif()
set(_edu_robocup_rescue_stack_CONFIG_INCLUDED TRUE)

# output package information
if(NOT edu_robocup_rescue_stack_FIND_QUIETLY)
  message(STATUS "Found edu_robocup_rescue_stack: 0.0.0 (${edu_robocup_rescue_stack_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'edu_robocup_rescue_stack' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${edu_robocup_rescue_stack_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(edu_robocup_rescue_stack_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${edu_robocup_rescue_stack_DIR}/${_extra}")
endforeach()
