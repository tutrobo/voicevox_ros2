/*
 * Copyright 2023 Rin Iwai.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>

#include "voicevox_core.h"
#include "voicevox_core_vendor/open_jtalk_dict_dir.h"
#include "voicevox_ros2_msgs/msg/talk.hpp"

#include "mixer.hpp"
#include "queue.hpp"

namespace tutrobo {
class VoicevoxRos2 : public rclcpp::Node {
  Queue<voicevox_ros2_msgs::msg::Talk, 10> talk_queue_;
  std::thread talk_thread_;
  rclcpp::Subscription<voicevox_ros2_msgs::msg::Talk>::SharedPtr
      voicevox_ros2_sub_;

  static inline Mixer mixer;

public:
  VoicevoxRos2(const rclcpp::NodeOptions &options)
      : VoicevoxRos2("", options) {}

  VoicevoxRos2(const std::string &name_space = "",
               const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("voicevox_ros2_node", name_space, options) {

    talk_thread_ = std::thread{&VoicevoxRos2::voicevox_talk, this};

    voicevox_ros2_sub_ =
        this->create_subscription<voicevox_ros2_msgs::msg::Talk>(
            "voicevox_ros2", rclcpp::QoS(10),
            [this](const voicevox_ros2_msgs::msg::Talk::SharedPtr msg) {
              if (msg->queuing) {
                talk_queue_.push_back(*msg);
              } else {
                talk_queue_.push_front(*msg);
              }
            });
  }

  ~VoicevoxRos2() {
    talk_queue_.abort();
    talk_thread_.join();
  }

private:
  void voicevox_talk() {
    // voicevox_core初期化
    RCLCPP_INFO(this->get_logger(), "Initializing voicevox_core...");
    VoicevoxInitializeOptions voicevox_options =
        voicevox_make_default_initialize_options();
    voicevox_options.load_all_models = true;
    voicevox_options.open_jtalk_dict_dir = OPEN_JTALK_DICT_DIR;
    if (voicevox_initialize(voicevox_options) == VOICEVOX_RESULT_OK) {
      RCLCPP_INFO(this->get_logger(), "Initialized voicevox_core.");
    } else {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize voicevox_core.");
    }

    // SDL2初期化
    SDL_Init(SDL_INIT_AUDIO);
    Mix_Init(0);
    Mix_OpenAudio(24000, AUDIO_S16LSB, 1, 4096);
    Mix_ChannelFinished([](int channel) { mixer.stop(channel); });

    voicevox_ros2_msgs::msg::Talk msg;

    while (this->talk_queue_.pop(msg)) {
      RCLCPP_INFO(this->get_logger(), "Synthesizing \"%s\"...",
                  msg.text.c_str());
      [[maybe_unused]] size_t wav_size = 0;
      uint8_t *wav = nullptr;
      VoicevoxResultCode voicevox_result =
          voicevox_tts(msg.text.c_str(), msg.speaker_id,
                       voicevox_make_default_tts_options(), &wav_size, &wav);
      if (voicevox_result != VOICEVOX_RESULT_OK) {
        RCLCPP_ERROR(this->get_logger(), "%s",
                     voicevox_error_result_to_message(voicevox_result));
      }

      if (msg.queuing) {
        mixer.enqueue(Mix_QuickLoad_WAV(wav));
      } else {
        mixer.play(Mix_QuickLoad_WAV(wav));
      }
    }

    // 音声をすべて停止
    mixer.stop_all();

    // 終了処理
    Mix_CloseAudio();
    Mix_Quit();
    SDL_Quit();
  }
};
} // namespace tutrobo
