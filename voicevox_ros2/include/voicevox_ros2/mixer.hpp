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

#include <deque>
#include <unordered_map>

#include <SDL2/SDL_mixer.h>

class Mixer {
  std::unordered_map<int, Mix_Chunk *> channels_;
  std::deque<Mix_Chunk *> queue_;

public:
  void play(Mix_Chunk *chunk) {
    Mix_HaltChannel(-1);
    channels_[Mix_PlayChannel(-1, chunk, 0)] = chunk;
  }

  void enqueue(Mix_Chunk *chunk) {
    if (queue_.empty() && Mix_Playing(-1) == 0) {
      play(chunk);
    } else {
      queue_.push_back(chunk);
    }
  }

  void stop(int channel) {
    Mix_FreeChunk(channels_[channel]);
    channels_.erase(channel);
    if (!queue_.empty()) {
      Mix_Chunk *chunk = queue_.front();
      queue_.pop_front();
      play(chunk);
    }
  }

  void stop_all() {
    Mix_ChannelFinished(nullptr);
    Mix_HaltChannel(-1);
    for (const auto &kv : channels_) {
      Mix_FreeChunk(kv.second);
    }
    channels_.clear();
  }
};
