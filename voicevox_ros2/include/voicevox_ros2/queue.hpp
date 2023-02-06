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

#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

template <class T, size_t N> class Queue {
  std::deque<T> queue_;
  bool aborted_ = false;
  std::mutex mtx_;
  std::condition_variable cv_;

public:
  void push_back(const T &x) {
    std::unique_lock lock{mtx_};
    if (queue_.size() < N) {
      queue_.push_back(x);
      cv_.notify_all();
    }
  }

  void push_front(const T &x) {
    std::unique_lock lock{mtx_};
    if (queue_.size() == N) {
      queue_.pop_back();
    }
    queue_.push_front(x);
    cv_.notify_all();
  }

  bool pop(T &x) {
    std::unique_lock lock{mtx_};
    cv_.wait(lock, [this] { return !queue_.empty() || aborted_; });
    if (aborted_) {
      return false;
    }
    x = queue_.front();
    queue_.pop_front();
    cv_.notify_all();
    return true;
  }

  void abort() {
    std::unique_lock lock{mtx_};
    aborted_ = true;
    cv_.notify_all();
  }
};
