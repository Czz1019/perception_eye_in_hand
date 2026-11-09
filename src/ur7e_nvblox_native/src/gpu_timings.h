// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cuda_runtime_api.h>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Measurements never add a CUDA stream synchronization. The single processing
// group owns this pool; events are polled and reused only after completion.
class GpuTimings {
public:
  using Clock = std::chrono::steady_clock;
  struct Sample {
    cudaEvent_t start = nullptr, end = nullptr;
    std::string label;
    Clock::time_point host_start, input_seen;
    uint64_t version = 0;
    Sample() {
      check(cudaEventCreate(&start));
      try { check(cudaEventCreate(&end)); }
      catch (...) { cudaEventDestroy(start); throw; }
    }
    ~Sample() { cudaEventDestroy(start); cudaEventDestroy(end); }
    static void check(cudaError_t error) {
      if (error != cudaSuccess) throw std::runtime_error(cudaGetErrorString(error));
    }
  };
  using Token = std::unique_ptr<Sample>;
  Token begin(const std::string &label, Clock::time_point input_seen, cudaStream_t stream) {
    if (pending_.size() >= 64) { ++dropped_; return {}; }
    Token token;
    if (free_.empty()) token = std::make_unique<Sample>();
    else { token = std::move(free_.back()); free_.pop_back(); }
    token->label = label;
    token->host_start = Clock::now(); token->input_seen = input_seen;
    Sample::check(cudaEventRecord(token->start, stream));
    return token;
  }
  void finish(Token token, uint64_t version, cudaStream_t stream) {
    if (!token) return;
    token->version = version;
    Sample::check(cudaEventRecord(token->end, stream));
    pending_.push_back(std::move(token));
  }
  void cancel(Token token) { if (token) free_.push_back(std::move(token)); }
  template <class Callback> void poll(Callback callback) {
    while (!pending_.empty()) {
      auto status = cudaEventQuery(pending_.front()->end);
      if (status == cudaErrorNotReady) break;
      Sample::check(status);
      auto token = std::move(pending_.front()); pending_.pop_front();
      float gpu_ms = 0;
      Sample::check(cudaEventElapsedTime(&gpu_ms, token->start, token->end));
      callback(*token, gpu_ms);
      free_.push_back(std::move(token));
    }
  }
  // Called after the reset path has synchronized/cleared the map.
  void reset() { pending_.clear(); free_.clear(); }
  size_t pending() const { return pending_.size(); }
  uint64_t dropped() const { return dropped_; }
private:
  std::deque<Token> pending_;
  std::vector<Token> free_;
  uint64_t dropped_ = 0;
};
