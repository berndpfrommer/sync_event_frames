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

#ifndef BAG_TO_FRAMES_ROS2_HPP_
#define BAG_TO_FRAMES_ROS2_HPP_

#include <event_array_msgs/msg/event_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <unordered_map>

#include "sync_event_frames/approx_reconstructor.hpp"
#include "sync_event_frames/frame_handler.hpp"
#include "sync_event_frames/utils.hpp"

using event_array_msgs::msg::EventArray;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;

template <class MsgT>
std::shared_ptr<MsgT> deserialize(
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & msg)
{
  rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
  std::shared_ptr<MsgT> m(new MsgT());
  rclcpp::Serialization<MsgT> serialization;
  try {
    serialization.deserialize_message(&serializedMsg, m.get());
  } catch (const rclcpp::exceptions::RCLError & e) {
    return (std::shared_ptr<MsgT>());
  }
  return (m);
}

class OutBagWriter
: public sync_event_frames::FrameHandler<Image::ConstSharedPtr>
{
public:
  explicit OutBagWriter(
    const std::string & bagName, const std::vector<std::string> & outTopics,
    const std::vector<std::string> & compressedOutTopics)
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(bagName);
    for (const auto & topic : outTopics) {
      writer_->create_topic(
        {topic, "sensor_msgs/msg/Image", rmw_get_serialization_format(), ""});
    }
    for (const auto & topic : compressedOutTopics) {
      writer_->create_topic(
        {topic, "sensor_msgs/msg/CompressedImage",
         rmw_get_serialization_format(), ""});
    }
  }

  void frame(
    const Image::ConstSharedPtr & img, const std::string & topic) override
  {
    write<Image>(img, topic, "sensor_msgs/msg/Image");
    numFrames_++;
    if (numFrames_ % 100 == 0) {
      std::cout << "wrote " << numFrames_ << " frames " << std::endl;
    }
  }

  template <typename MsgT>
  void write(
    const typename MsgT::ConstSharedPtr & m, const std::string & topic,
    const std::string & topicType)
  {
    auto smsg = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<MsgT> serialization;
    serialization.serialize_message(m.get(), smsg.get());
#ifdef USE_NEW_ROSBAG_WRITE_INTERFACE
    writer_->write(smsg, topic, topicType, rclcpp::Time(m->header.stamp));
#else
    writer_->write(*smsg, topic, topicType, rclcpp::Time(m->header.stamp));
#endif
  }

private:
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  size_t numFrames_{0};
};

using ApproxRecon = sync_event_frames::ApproxReconstructor<
  EventArray, EventArray::ConstSharedPtr, Image, Image::ConstSharedPtr,
  rclcpp::Time>;

size_t processFreeRunning(
  double fps, rosbag2_cpp::Reader & reader, bool syncOnSensorTime,
  std::unordered_map<std::string, ApproxRecon> * recons)
{
  std::cout << "Converting in freerunning mode..." << std::endl;
  const uint64_t sliceInterval =
    static_cast<uint64_t>(1000000000 / std::abs(fps));
  const rclcpp::Duration rosSliceInterval =
    rclcpp::Duration::from_nanoseconds(sliceInterval);
  for (auto & r : *recons) {
    r.second.setSyncOnSensorTime(syncOnSensorTime);
  }

  size_t numMessages(0);
  uint64_t nextSensorFrameTime = 0;
  rclcpp::Time nextROSFrameTime;
  bool hasValidTime{false};
  rclcpp::Serialization<EventArray> eventsSerialization;

  while (reader.has_next()) {
    auto msg = reader.read_next();
    auto it = recons->find(msg->topic_name);
    if (it != recons->end()) {
      auto m = deserialize<EventArray>(msg);
      if (!hasValidTime) {
        hasValidTime = sync_event_frames::utils::findNextFrameTime(
          m, &nextSensorFrameTime, &nextROSFrameTime, sliceInterval);
      }
      if (hasValidTime) {
        // call processMsg() *before* adding the frames such that the
        // offset between sensor and ROS time is initialized when frame time
        // is added
        it->second.processMsg(m);
        // now add the frames
        if (nextROSFrameTime < m->header.stamp) {
          nextSensorFrameTime += sliceInterval;
          nextROSFrameTime += rosSliceInterval;
          for (auto & r : *recons) {
            r.second.addFrameTime(nextSensorFrameTime, nextROSFrameTime);
          }
        }
      }
      numMessages++;
    }
  }
  return (numMessages);
}

void addFrameTime(
  std::unordered_map<std::string, ApproxRecon> * recons, const rclcpp::Time & t)
{
  for (auto & ir : *recons) {
    auto & r = ir.second;
    if (r.hasValidSensorTimeOffset()) {
      r.addFrameTime(t.nanoseconds() - r.getSensorTimeOffset(), t);
    }
  }
}

template <class MsgT>
void handleFrame(
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & msg,
  const std::string & topic, const std::string & topicType,
  std::set<rclcpp::Time> * frameTimes,
  std::unordered_map<std::string, ApproxRecon> * recons, OutBagWriter * writer)
{
  auto m = deserialize<MsgT>(msg);
  if (m) {
    rclcpp::Time rosTime(m->header.stamp);
    if (frameTimes->find(rosTime) == frameTimes->end()) {
      addFrameTime(recons, rosTime);
      frameTimes->insert(rosTime);
    }
    writer->write<MsgT>(m, topic, topicType);
  }
}

size_t processFrameBased(
  const std::vector<std::string> & ft,
  const std::unordered_map<std::string, std::string> & topicToType,
  rosbag2_cpp::Reader & reader,
  std::unordered_map<std::string, ApproxRecon> * recons, OutBagWriter * writer)
{
  std::cout << "Sync to frame based cameras with topics: ";
  for (const auto & tp : ft) {
    std::cout << " " << tp;
  }
  std::cout << std::endl;
  size_t numMessages(0);
  std::set<rclcpp::Time> frameTimes;

  while (reader.has_next()) {
    auto msg = reader.read_next();
    const std::string topic = msg->topic_name;
    auto ite = recons->find(topic);
    if (ite != recons->end()) {
      // handle event based camera msg
      auto m = deserialize<EventArray>(msg);
      ite->second.processMsg(m);
    }
    if (std::find(ft.begin(), ft.end(), topic) != ft.end()) {
      auto topicType = topicToType.find(topic);
      if (topicType == topicToType.end()) {
        std::cout << "unknown image type for topic: " << topic << std::endl;
        throw(std::runtime_error("unknown image type!"));
      }
      if (topicType->second == "sensor_msgs/msg/CompressedImage") {
        handleFrame<CompressedImage>(
          msg, topic, topicType->second, &frameTimes, recons, writer);
      } else if (topicType->second == "sensor_msgs/msg/Image") {
        handleFrame<Image>(
          msg, topic, topicType->second, &frameTimes, recons, writer);
      } else {
        std::cout << "unregistered image type for topic: " << topic
                  << std::endl;
        throw(std::runtime_error("unregistered image type!"));
      }
    }
    numMessages++;
  }
  return (numMessages);
}

std::unordered_map<std::string, std::string> getTopicMetaData(
  rosbag2_cpp::Reader & reader, const std::vector<std::string> & ft,
  std::vector<std::string> * uncomp, std::vector<std::string> * comp)
{
  std::unordered_map<std::string, std::string> topicToType;
  std::vector<rosbag2_storage::TopicMetadata> meta =
    reader.get_all_topics_and_types();
  for (const auto & m : meta) {
    topicToType.insert({m.name, m.type});
    if (std::find(ft.begin(), ft.end(), m.name) != ft.end()) {
      if (m.type == "sensor_msgs/msg/CompressedImage") {
        comp->push_back(m.name);
      } else {
        uncomp->push_back(m.name);
      }
    }
  }
  return (topicToType);
}

size_t process_bag(
  const std::string & inBagName, const std::string & outBagName,
  const std::vector<std::string> & inTopics,
  const std::vector<std::string> & outTopics,
  const std::vector<std::string> & frameTopics, int cutoffPeriod,
  bool hasSyncCable, double fps)
{
  rosbag2_cpp::Reader reader;
  reader.open(inBagName);

  std::vector<std::string> uncompressedTopics = outTopics;
  std::vector<std::string> compressedTopics;
  std::unordered_map<std::string, std::string> topicToType = getTopicMetaData(
    reader, frameTopics, &uncompressedTopics, &compressedTopics);

  OutBagWriter writer(outBagName, uncompressedTopics, compressedTopics);

  // set up the reconstruction objects
  std::unordered_map<std::string, ApproxRecon> recons;
  for (size_t i = 0; i < inTopics.size(); i++) {
    const double fillRatio = 0.6;
    const int tileSize = 2;
    recons.insert(
      {inTopics[i],
       ApproxRecon(&writer, outTopics[i], cutoffPeriod, fillRatio, tileSize)});
  }

  size_t numMessages(0);
  if (fps > 0) {
    numMessages = processFreeRunning(fps, reader, hasSyncCable, &recons);
  } else {
    numMessages =
      processFrameBased(frameTopics, topicToType, reader, &recons, &writer);
  }
  return (numMessages);
  return (0);
}

#endif  // BAG_TO_FRAMES_ROS2_HPP_
