#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rclc::rclc" for configuration "Release"
set_property(TARGET rclc::rclc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rclc::rclc PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librclc.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS rclc::rclc )
list(APPEND _IMPORT_CHECK_FILES_FOR_rclc::rclc "${_IMPORT_PREFIX}/lib/librclc.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
