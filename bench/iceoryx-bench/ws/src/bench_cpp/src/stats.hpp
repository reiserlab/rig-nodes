// Shared helpers: wall-clock nanoseconds and a JSON summary line.
#pragma once
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <vector>

inline int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

inline void report(std::vector<int64_t> & lat, const char * who) {
  if (lat.empty()) { std::printf("{\"sub\":\"%s\",\"n\":0}\n", who); return; }
  std::sort(lat.begin(), lat.end());
  auto q = [&](double p) { return lat[static_cast<size_t>(p * (lat.size() - 1))] / 1e6; };
  std::printf("{\"sub\":\"%s\",\"n\":%zu,\"median_ms\":%.3f,\"p99_ms\":%.3f}\n",
    who, lat.size(), q(0.5), q(0.99));
}
