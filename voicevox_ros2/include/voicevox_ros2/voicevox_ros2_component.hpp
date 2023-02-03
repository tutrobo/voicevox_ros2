#pragma once

#include <ao/ao.h>
#include <rclcpp/rclcpp.hpp>
#include <voicevox_ros2_msgs/msg/talk.hpp>

#include "voicevox_core.h"
#include "voicevox_core_vendor/open_jtalk_dict_dir.h"

extern const char *open_jtalk_dict_path;

namespace tutrobo {
class VoicevoxRos2 : public rclcpp::Node {
private:
  rclcpp::Subscription<voicevox_ros2_msgs::msg::Talk>::SharedPtr subscription_;
  ao_device *device;

public:
  VoicevoxRos2(const rclcpp::NodeOptions &options)
      : VoicevoxRos2("", options) {}
  VoicevoxRos2(const std::string &name_space = "",
               const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("voicevox_ros2_node", name_space, options) {

    RCLCPP_INFO(this->get_logger(), "Initializing voicevox_core...");

    VoicevoxInitializeOptions voicevox_options =
        voicevox_make_default_initialize_options();
    voicevox_options.load_all_models = true;
    voicevox_options.open_jtalk_dict_dir = OPEN_JTALK_DICT_DIR;
    if (voicevox_initialize(voicevox_options) != VOICEVOX_RESULT_OK) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize voicevox_core");
    }

    RCLCPP_INFO(this->get_logger(), "Initialized voicevox_core");

    ao_initialize();

    ao_sample_format format = {0};
    format.bits = 16;
    format.rate = 24000;
    format.channels = 1;
    format.byte_format = AO_FMT_LITTLE;

    int default_driver = ao_default_driver_id();
    device = ao_open_live(default_driver, &format, nullptr);

    subscription_ = this->create_subscription<voicevox_ros2_msgs::msg::Talk>(
        "voicevox_ros2", rclcpp::QoS(10),
        [&, this](const voicevox_ros2_msgs::msg::Talk::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(), "Generating voice data...");

          size_t output_wav_size = 0;
          uint8_t *output_wav = nullptr;
          VoicevoxResultCode voicevox_result =
              voicevox_tts(msg->text.c_str(), msg->speaker_id,
                           voicevox_make_default_tts_options(),
                           &output_wav_size, &output_wav);
          if (voicevox_result != VOICEVOX_RESULT_OK) {
            RCLCPP_ERROR(this->get_logger(), "%s",
                         voicevox_error_result_to_message(voicevox_result));
          }

          ao_play(this->device, reinterpret_cast<char *>(output_wav + 44),
                  output_wav_size - 44);

          voicevox_wav_free(output_wav);
        });
  }

  ~VoicevoxRos2() {
    ao_close(this->device);
    ao_shutdown();
  }
};
} // namespace tutrobo
