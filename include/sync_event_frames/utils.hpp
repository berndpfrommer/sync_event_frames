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

#ifndef SYNC_EVENT_FRAMES__UTILS_HPP_
#define SYNC_EVENT_FRAMES__UTILS_HPP_

#include <event_array_codecs/decoder.h>
#include <event_array_codecs/decoder_factory.h>

#include "sync_event_frames/ros_compat.hpp"

namespace sync_event_frames
{
namespace utils
{
template <class MsgPtrT, class RosTimeT>
bool findNextFrameTime(
  MsgPtrT m, uint64_t * nextSensorFrameTime, RosTimeT * nextROSFrameTime,
  uint64_t sliceInterval)
{
  event_array_codecs::DecoderFactory decoderFactory;
  auto decoder = decoderFactory.getInstance(m->encoding, m->width, m->height);
  if (!decoder) {
    std::cout << "invalid encoding: " << m->encoding << std::endl;
    throw(std::runtime_error("invalid encoding!"));
  }
  uint64_t firstSensorTime{0};
  if (decoder->findFirstSensorTime(
        m->events.data(), m->events.size(), &firstSensorTime)) {
    *nextSensorFrameTime = (firstSensorTime / sliceInterval) * sliceInterval;
    *nextROSFrameTime =
      RosTimeT(m->header.stamp) +
      ros_compat::duration_from_nanoseconds(firstSensorTime % sliceInterval);
    return (true);
  }
  std::cout << "WARNING: no time stamp found in packet!" << std::endl;
  return (false);
}

}  // namespace utils
}  // namespace sync_event_frames
#endif  // SYNC_EVENT_FRAMES__UTILS_HPP_
