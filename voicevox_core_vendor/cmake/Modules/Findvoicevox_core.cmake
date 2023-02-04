find_library(voicevox_core_LIBRARY voicevox_core
  PATH_SUFFIXES lib/voicevox_core_vendor/voicevox_core
)
find_path(voicevox_core_INCLUDE_DIR voicevox_core.h
  PATH_SUFFIXES lib/voicevox_core_vendor/voicevox_core
)

mark_as_advanced(voicevox_core_LIBRARY voicevox_core_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(voicevox_core REQUIRED_VARS
  voicevox_core_LIBRARY
  voicevox_core_INCLUDE_DIR
)

if(voicevox_core_FOUND)
  add_library(voicevox_core::voicevox_core SHARED IMPORTED)
  set_target_properties(voicevox_core::voicevox_core PROPERTIES
    IMPORTED_LINK_INTERFACE_LANGUAGES "C"
    IMPORTED_LOCATION "${voicevox_core_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${voicevox_core_INCLUDE_DIR}"
  )
endif()
