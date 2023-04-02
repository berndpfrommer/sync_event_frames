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

#ifndef SYNC_EVENT_FRAMES__APPROX_RECONSTRUCTOR_HPP_
#define SYNC_EVENT_FRAMES__APPROX_RECONSTRUCTOR_HPP_

#include <event_array_codecs/decoder_factory.h>
#include <event_array_codecs/event_processor.h>

#include <memory>
#include <queue>
#include <simple_image_recon_lib/simple_image_reconstructor.hpp>
#include <string>

#include "sync_event_frames/check_endian.hpp"
#include "sync_event_frames/frame_handler.hpp"
#include "sync_event_frames/ros_compat.hpp"
#include "sync_event_frames/time_offset.hpp"

namespace sync_event_frames
{
template <
  typename EventArrayT, typename EventArrayConstSharedPtrT, typename ImageT,
  typename ImageConstPtrT, typename RosTimeT>
class ApproxReconstructor : public event_array_codecs::EventProcessor
{
public:
  struct FrameTime
  {
    FrameTime(uint64_t s, const RosTimeT & r) : sensorTime(s), rosTime(r) {}
    uint64_t sensorTime{0};
    RosTimeT rosTime;
  };

  using EventArray = EventArrayT;
  explicit ApproxReconstructor(
    FrameHandler<ImageConstPtrT> * fh, const std::string & topic,
    int cutoffNumEvents = 30, double fillRatio = 0.6, int tileSize = 2)
  : frameHandler_(fh),
    topic_(topic),
    cutoffNumEvents_(cutoffNumEvents),
    fillRatio_(fillRatio),
    tileSize_(tileSize)
  {
    imageMsgTemplate_.height = 0;
  }

  // ---------- inherited from EventProcessor
  inline void eventCD(
    uint64_t t, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    if (__builtin_expect(updateROStoSensorTimeOffset_, false)) {
      auto msg = bufferedMessages_.front();
      updateROSTimeOffset(
        ros_compat::to_nanoseconds(RosTimeT(msg->header.stamp)), t);
    }
    simpleReconstructor_.event(t, ex, ey, polarity);
  }
  void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {}
  void finished() override{};
  void rawData(const char *, size_t) override{};
  // --------- end of inherited from EventProcessor

  void addFrameTime(uint64_t sensorTime, const RosTimeT t)
  {
    // if there is no sync cable between the cameras,
    // compute the sensor time from ros time and offset
    const uint64_t st =
      syncOnSensorTime_
        ? sensorTime
        : (ros_compat::to_nanoseconds(t) - timeOffset_.getOffset());
    frameTimes_.push(FrameTime(st, t));
    process();
  }

  void setSyncOnSensorTime(bool s) { syncOnSensorTime_ = s; }
  void processMsg(EventArrayConstSharedPtrT msg)
  {
    if (imageMsgTemplate_.height == 0) {
      imageMsgTemplate_.header = msg->header;
      imageMsgTemplate_.width = msg->width;
      imageMsgTemplate_.height = msg->height;
      imageMsgTemplate_.encoding = "mono8";
      imageMsgTemplate_.is_bigendian = check_endian::isBigEndian();
      imageMsgTemplate_.step = imageMsgTemplate_.width;
      simpleReconstructor_.initialize(
        msg->width, msg->height,
        static_cast<uint32_t>(std::abs(cutoffNumEvents_)), tileSize_,
        fillRatio_);
      decoder_ =
        decoderFactory_.getInstance(msg->encoding, msg->width, msg->height);
      if (!decoder_) {
        std::cerr << "invalid encoding: " << msg->encoding << std::endl;
        throw(std::runtime_error("invalid encoding!"));
      }

      auto decoder = event_array_codecs::DecoderFactory().getInstance(
        msg->encoding, msg->width, msg->height);
      uint64_t firstTS{0};
      bool foundTime = decoder->findFirstSensorTime(
        msg->events.data(), msg->events.size(), &firstTS);
      if (!foundTime) {
        std::cout << "WARNING: first message does not contain time stamp!"
                  << std::endl;
      } else {
        updateROSTimeOffset(
          ros_compat::to_nanoseconds(RosTimeT(msg->header.stamp)), firstTS);
      }
    }
    bufferedMessages_.push(msg);
    process();
  }

  void process()
  {
    while (!bufferedMessages_.empty() && !frameTimes_.empty()) {
      auto msg = bufferedMessages_.front();
      const size_t bytesToDecode = msg->events.size() - firstByteToDecode_;
      uint64_t nextTime{0};
      if (firstByteToDecode_ == 0) {
        updateROStoSensorTimeOffset_ = true;
      }
      const size_t bytesDecoded = decoder_->decodeUntil(
        &(msg->events[firstByteToDecode_]), bytesToDecode, this,
        frameTimes_.front().sensorTime, &nextTime);
      if (bytesDecoded < bytesToDecode) {
        // message could not be completely decoded
        // before hitting the next frame time
        firstByteToDecode_ += bytesDecoded;  // move the index pointer
        emitFramesOlderThan(nextTime);
      } else {
        // message was completely decoded
        bufferedMessages_.pop();
        firstByteToDecode_ = 0;
      }
    }
  }

private:
  void updateROSTimeOffset(uint64_t tros, uint64_t tsens)
  {
    timeOffset_.update(tros, tsens);
    updateROStoSensorTimeOffset_ =
      false;  // only update on first event after stamp
  }
  void emitFramesOlderThan(uint64_t currentTime)
  {
    while (!frameTimes_.empty() &&
           frameTimes_.front().sensorTime <= currentTime) {
      const auto & frameTime = frameTimes_.front();
      auto msg = std::make_unique<ImageT>(imageMsgTemplate_);
      msg->data.resize(msg->height * msg->step);
      simpleReconstructor_.getImage(&(msg->data[0]), msg->step);
      msg->header.stamp = frameTime.rosTime;
      frameHandler_->frame(std::move(msg), topic_);
      frameTimes_.pop();
    }
  }

  // ------------------------  variables ------------------------------
  FrameHandler<ImageConstPtrT> * frameHandler_{nullptr};
  std::string topic_;
  ImageT imageMsgTemplate_;
  int cutoffNumEvents_{0};
  double fillRatio_{0};
  int tileSize_{0};
  TimeOffset timeOffset_;
  bool updateROStoSensorTimeOffset_{true};
  bool syncOnSensorTime_{false};
  event_array_codecs::Decoder<ApproxReconstructor> * decoder_{nullptr};
  event_array_codecs::DecoderFactory<ApproxReconstructor> decoderFactory_;
  simple_image_recon_lib::SimpleImageReconstructor simpleReconstructor_;
  std::queue<FrameTime> frameTimes_;
  std::queue<EventArrayConstSharedPtrT> bufferedMessages_;
  size_t firstByteToDecode_{0};
};
}  // namespace sync_event_frames
#endif  // SYNC_EVENT_FRAMES__APPROX_RECONSTRUCTOR_HPP_
