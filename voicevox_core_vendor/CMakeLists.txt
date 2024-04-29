cmake_minimum_required(VERSION 3.8)
project(voicevox_core_vendor)

find_package(ament_cmake REQUIRED)

option(WITH_CUDA "Use voicevox_core with CUDA" OFF)
option(WITH_DIRECTML "Use voicevox_core with DirectML" OFF)

set(voicevox_core_VERSION 0.15.3)
set(voicevox_additional_libraries_VERSION 0.1.0)

# デバイスの選択
set(DEVICE cpu)
if(WITH_CUDA)
  set(DEVICE cuda)
elseif(WITH_DIRECTML)
  set(DEVICE directml)
endif()

# アーキテクチャ名を変換
set(CPU_ARCH_x86_64 x64)
set(CPU_ARCH_amd64 x64)
set(CPU_ARCH_AMD64 x64)
set(CPU_ARCH_arm64 arm64)
set(CPU_ARCH_aarch64 arm64)
set(CPU_ARCH ${CPU_ARCH_${CMAKE_SYSTEM_PROCESSOR}})

# OS名を変換
set(OS_Linux linux)
set(OS_Windows windows)
set(OS_Darwin osx)
set(OS ${OS_${CMAKE_SYSTEM_NAME}})

# voicevox_coreの取得, インストール
if(${OS} STREQUAL "windows")
  add_custom_target(voicevox_core ALL
    COMMAND curl -sSfL https://github.com/VOICEVOX/voicevox_core/releases/latest/download/download-${OS}-${CPU_ARCH}.exe -o download.exe
    COMMAND download.exe --version ${voicevox_core_VERSION} --additional-libraries-version ${voicevox_additional_libraries_VERSION} --device ${DEVICE} --cpu-arch ${CPU_ARCH} --os ${OS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )
else()
  add_custom_target(voicevox_core ALL
    COMMAND curl -sSfL https://github.com/VOICEVOX/voicevox_core/releases/latest/download/download-${OS}-${CPU_ARCH} -o download
    COMMAND chmod +x download
    COMMAND ./download --version ${voicevox_core_VERSION} --additional-libraries-version ${voicevox_additional_libraries_VERSION} --device ${DEVICE} --cpu-arch ${CPU_ARCH} --os ${OS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )
endif()

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/voicevox_core/
  DESTINATION lib
)

# ヘッダファイル生成
configure_file(
  "open_jtalk_dict_dir.h.in"
  "open_jtalk_dict_dir.h"
  @ONLY
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/voicevox_core/voicevox_core.h
    ${CMAKE_CURRENT_BINARY_DIR}/open_jtalk_dict_dir.h
  DESTINATION include/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_library_names(voicevox_core)

ament_package()
