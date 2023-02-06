cmake_minimum_required(VERSION 3.8)
project(voicevox_core_vendor)

find_package(ament_cmake REQUIRED)

option(WITH_CUDA "Use voicevox_core with CUDA" OFF)
option(WITH_DIRECTML "Use voicevox_core with DirectML" OFF)

set(voicevox_core_VERSION 0.14.0)
set(voicevox_additional_libraries_VERSION 0.1.0)

# OS名を変換
set(OS_Linux linux)
set(OS_Windows windows)
set(OS_Darwin osx)
set(OS ${OS_${CMAKE_SYSTEM_NAME}})

# アーキテクチャ名を変換
set(ARCH_x86_64 x64)
set(ARCH_arm64 arm64)
set(ARCH_aarch64 arm64)
set(ARCH ${ARCH_${CMAKE_SYSTEM_PROCESSOR}})

# デバイスの選択
set(DEVICE cpu)
if(WITH_CUDA)
  set(DEVICE gpu)
  set(voicevox_additional_libraries_URL https://github.com/VOICEVOX/voicevox_additional_libraries/releases/download/${voicevox_additional_libraries_VERSION}/CUDA-${OS}-${ARCH}.zip)
elseif(WITH_DIRECTML)
  set(DEVICE directml)
  set(voicevox_additional_libraries_URL https://github.com/VOICEVOX/voicevox_additional_libraries/releases/download/${${voicevox_additional_libraries_VERSION}}/DirectML-${OS}-${ARCH}.zip)
endif()

set(voicevox_core_URL https://github.com/VOICEVOX/voicevox_core/releases/download/${voicevox_core_VERSION}/voicevox_core-${OS}-${ARCH}-${DEVICE}-${voicevox_core_VERSION}.zip)
set(open_jtalk_dict_URL https://jaist.dl.sourceforge.net/project/open-jtalk/Dictionary/open_jtalk_dic-1.11/open_jtalk_dic_utf_8-1.11.tar.gz)

# voicevox_core, open_jtalk_dictの取得, インストール
include(FetchContent)
FetchContent_Declare(voicevox_core
  URL ${voicevox_core_URL}
)
FetchContent_Declare(open_jtalk_dict
  URL ${open_jtalk_dict_URL}
)
FetchContent_MakeAvailable(voicevox_core open_jtalk_dict)

install(
  DIRECTORY ${voicevox_core_SOURCE_DIR}/
  DESTINATION lib
)

install(
  DIRECTORY ${open_jtalk_dict_SOURCE_DIR}/
  DESTINATION lib/open_jtalk_dict
)

# voicevox_additional_librariesの取得, インストール
if(DEFINED voicevox_additional_libraries_URL)
  FetchContent_Declare(voicevox_additional_libraries
    URL ${voicevox_additional_libraries_URL}
  )
  FetchContent_MakeAvailable(voicevox_additional_libraries)

  install(
    DIRECTORY ${voicevox_additional_libraries_SOURCE_DIR}/
    DESTINATION lib
  )
endif()

# ヘッダファイル生成
configure_file(
  "open_jtalk_dict_dir.h.in"
  "open_jtalk_dict_dir.h"
  @ONLY
)

install(
  FILES
    ${voicevox_core_SOURCE_DIR}/voicevox_core.h
    ${CMAKE_CURRENT_BINARY_DIR}/open_jtalk_dict_dir.h
  DESTINATION include/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_library_names(voicevox_core)

ament_package()
