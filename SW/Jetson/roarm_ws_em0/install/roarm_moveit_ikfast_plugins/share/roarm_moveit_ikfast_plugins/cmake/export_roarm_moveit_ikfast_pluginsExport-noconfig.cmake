#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "roarm_moveit_ikfast_plugins::roarm_description_hand_moveit_ikfast_plugin" for configuration ""
set_property(TARGET roarm_moveit_ikfast_plugins::roarm_description_hand_moveit_ikfast_plugin APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(roarm_moveit_ikfast_plugins::roarm_description_hand_moveit_ikfast_plugin PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libroarm_description_hand_moveit_ikfast_plugin.so"
  IMPORTED_SONAME_NOCONFIG "libroarm_description_hand_moveit_ikfast_plugin.so"
  )

list(APPEND _cmake_import_check_targets roarm_moveit_ikfast_plugins::roarm_description_hand_moveit_ikfast_plugin )
list(APPEND _cmake_import_check_files_for_roarm_moveit_ikfast_plugins::roarm_description_hand_moveit_ikfast_plugin "${_IMPORT_PREFIX}/lib/libroarm_description_hand_moveit_ikfast_plugin.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
