#!/bin/bash

CMAKE_COMMAND=$1
CMAKE_CURRENT_BINARY_DIR=$2
CMAKE_INSTALL_PREFIX=$3
USE_CUDA=$4

if [ ! -d "${CMAKE_INSTALL_PREFIX}/voicevox_core" ]; then
  ${CMAKE_COMMAND} -E rm -rf "${CMAKE_CURRENT_BINARY_DIR}/voicevox_core" "${CMAKE_INSTALL_PREFIX}/voicevox_core"
  if [ "${USE_CUDA}" = "ON" ]; then
    curl -sSfL https://github.com/VOICEVOX/voicevox_core/releases/latest/download/download.sh | bash -s -- --device cuda
  else
    curl -sSfL https://github.com/VOICEVOX/voicevox_core/releases/latest/download/download.sh | bash -s
  fi
  ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_BINARY_DIR}/voicevox_core" "${CMAKE_INSTALL_PREFIX}/voicevox_core"
fi

open_jtalk_dict_path=$(find ${CMAKE_INSTALL_PREFIX}/voicevox_core -name open_jtalk_dic_utf_8-\*)
echo "#define OPEN_JTALK_DICT_PATH \"${open_jtalk_dict_path}\"" > "${CMAKE_CURRENT_BINARY_DIR}/open_jtalk_dict_path.h"
