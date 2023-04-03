// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SYNC_EVENT_FRAMES__TIME_OFFSET_HPP_
#define SYNC_EVENT_FRAMES__TIME_OFFSET_HPP_

#include <cstdint>
#include <iostream>

namespace sync_event_frames
{
class TimeOffset
{
public:
  void setAveragingConstant(double alpha)
  {
    alpha_ = alpha;
    oneMinusAlpha_ = 1.0 - alpha;
  }
  void update(uint64_t t1, uint64_t t2)
  {
    const double dt =
      static_cast<double>(static_cast<int64_t>(t1) - static_cast<int64_t>(t2));
    if (__builtin_expect(firstTime_, false)) {
      avgDt_ = dt;
      firstTime_ = false;
    } else {
      avgDt_ = avgDt_ * oneMinusAlpha_ + alpha_ * dt;
    }
  }
  int64_t getOffset() const { return static_cast<int64_t>(avgDt_); }
  bool isValid() const { return (!firstTime_); }

private:
  double avgDt_{0};
  double alpha_{0.01};
  double oneMinusAlpha_{1.0 - 0.01};
  bool firstTime_{true};
};
}  // namespace sync_event_frames
#endif  // TIME_OFFSET_HPP_
