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

#ifndef BAG_TO_FRAMES_ROS1_HPP_
#define BAG_TO_FRAMES_ROS1_HPP_

#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_msgs/EventPacket.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <filesystem>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <memory>
#include <unordered_map>
#include <cv_bridge/cv_bridge.h>

#include "sync_event_frames/approx_reconstructor.hpp"
#include "sync_event_frames/frame_handler.hpp"
#include "sync_event_frames/utils.hpp"

using event_camera_msgs::EventPacket;
using sensor_msgs::CompressedImage;
using sensor_msgs::Image;

class OutBagWriter : public sync_event_frames::FrameHandler<Image::ConstPtr>
{
public:
  explicit OutBagWriter(const std::string & bagName, bool writeFrames = false):
      bagName_(bagName), writeFrames_(writeFrames)
  {
    outBag_.open(bagName, rosbag::bagmode::Write);
    if (writeFrames_) {
      std::filesystem::create_directories(
        bagName + "/frames");  // does recursive...
    }

  }
  void frame(uint64_t sensor_time,
	     const Image::ConstPtr & img, const std::string & topic) override
  {
    outBag_.write(topic, img->header.stamp, img);
    numFrames_++;
    if (numFrames_ % 100 == 0) {
      std::cout << "wrote " << numFrames_ << " frames " << std::endl;
    }
    if (writeFrames_) {
      std::cout << "frame " << numFrames_ << " ros time: "
                << ros::Time(img->header.stamp).toNSec()
                << " sensor time: " << sensor_time << std::endl;
      std::stringstream ss;
      ss << std::setw(10) << std::setfill('0') << sensor_time / 1000UL;
      const auto fname = bagName_ + "/frames/frame_" + ss.str() + ".png";
      auto cvImg = cv_bridge::toCvShare(img, "mono8");
      cv::imwrite(fname, cvImg->image);
    }
    numFrames_++;
    if (numFrames_ % 100 == 0) {
      std::cout << "wrote " << numFrames_ << " frames " << std::endl;
    }
  }

  rosbag::Bag * getBag() { return (&outBag_); }

private:
  rosbag::Bag outBag_;
  size_t numFrames_{0};
  std::string bagName_;
  bool writeFrames_{false};
};

using ApproxRecon = sync_event_frames::ApproxReconstructor<
  EventPacket, EventPacket::ConstPtr, Image, Image::ConstPtr, ros::Time>;

size_t processFreeRunning(
  double fps, rosbag::Bag & inBag, const std::vector<std::string> & inTopics,
  bool syncOnSensorTime, std::unordered_map<std::string, ApproxRecon> * recons)
{
  std::cout << "Converting in freerunning mode..." << std::endl;
  const uint64_t sliceInterval =
    static_cast<uint64_t>(1000000000 / std::abs(fps));
  const ros::Duration rosSliceInterval =
    ros::Duration().fromNSec(sliceInterval);
  for (auto & r : *recons) {
    r.second.setSyncOnSensorTime(syncOnSensorTime);
  }

  size_t numMessages(0);
  uint64_t nextSensorFrameTime = 0;
  ros::Time nextROSFrameTime;
  bool hasValidTime{false};
  rosbag::View view(inBag, rosbag::TopicQuery(inTopics));
  for (const rosbag::MessageInstance & msg : view) {
    auto it = recons->find(msg.getTopic());
    if (it != recons->end()) {
      auto m = msg.instantiate<EventPacket>();
      if (!hasValidTime) {
        hasValidTime = sync_event_frames::utils::findNextFrameTime(
          m, &nextSensorFrameTime, &nextROSFrameTime, sliceInterval);
      }
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

      numMessages++;
    }
  }
  return (numMessages);
}

std::vector<uint64_t> readTimeStamps(const std::string & fn)
{
  std::ifstream file(fn);
  if (file.fail()) {
    throw(std::runtime_error("cannot read time stamp file!"));
  }
  uint64_t t;
  std::vector<uint64_t> v;
  while (file >> t) {
    v.push_back(t * 1000);  // assume file is in usec
  }
  return (v);
}


size_t processOnTimeStamps(
  const std::string &tsFile, rosbag::Bag & inBag, const std::vector<std::string> & inTopics,
  bool syncOnSensorTime, std::unordered_map<std::string, ApproxRecon> * recons)
{
  std::cout << "Using time stamp file " << tsFile << std::endl;
  const auto timeStamps = readTimeStamps(tsFile);
  std::cout << "Read " << timeStamps.size() << " time stamps" << std::endl;
  if (timeStamps.empty()) {
    std::cerr << "no valid time stamps found in file!" << std::endl;
  }
  std::cout << "time range: " << *timeStamps.begin() << " - "
            << *timeStamps.rbegin() << std::endl;

  for (auto & r : *recons) {
    r.second.setSyncOnSensorTime(syncOnSensorTime);
    for (const auto & t : timeStamps) {
      r.second.addFrameTime(t, ros::Time());
    }
  }

  size_t numMessages(0);
  rosbag::View view(inBag, rosbag::TopicQuery(inTopics));
  for (const rosbag::MessageInstance & msg : view) {
    auto it = recons->find(msg.getTopic());
    if (it != recons->end()) {
      auto m = msg.instantiate<EventPacket>();
      it->second.processMsg(m);
      numMessages++;
    }
  }
  return (numMessages);
}

void addFrameTime(
  std::unordered_map<std::string, ApproxRecon> * recons, const ros::Time & t)
{
  for (auto & ir : *recons) {
    auto & r = ir.second;
    if (r.hasValidSensorTimeOffset()) {
      r.addFrameTime(t.toNSec() - r.getSensorTimeOffset(), t);
    }
  }
}

template <class MsgT>
void handleFrame(
  const rosbag::MessageInstance & msg, std::set<ros::Time> * frameTimes,
  std::unordered_map<std::string, ApproxRecon> * recons, rosbag::Bag * outBag)
{
  auto m = msg.instantiate<MsgT>();
  if (m) {
    ros::Time rosTime = m->header.stamp;
    if (frameTimes->find(rosTime) == frameTimes->end()) {
      addFrameTime(recons, rosTime);
      frameTimes->insert(rosTime);
    }
    outBag->write(msg.getTopic(), msg.getTime(), m);
  }
}

size_t processFrameBased(
  const std::vector<std::string> & ft, rosbag::Bag & inBag,
  const std::vector<std::string> & inTopics,
  std::unordered_map<std::string, ApproxRecon> * recons, rosbag::Bag * outBag)
{
  std::cout << "Sync to frame based cameras with topics: ";
  for (const auto & tp : ft) {
    std::cout << " " << tp;
  }
  std::cout << std::endl;
  size_t numMessages(0);
  std::vector<std::string> combinedTopics = inTopics;
  combinedTopics.insert(combinedTopics.end(), ft.begin(), ft.end());
  rosbag::View view(inBag, rosbag::TopicQuery(combinedTopics));
  std::set<ros::Time> frameTimes;
  for (const rosbag::MessageInstance & msg : view) {
    const std::string topic = msg.getTopic();
    auto ite = recons->find(topic);
    if (ite != recons->end()) {
      // handle event based camera msg
      auto m = msg.instantiate<EventPacket>();
      ite->second.processMsg(m);
    }
    if (std::find(ft.begin(), ft.end(), topic) != ft.end()) {
      handleFrame<Image>(msg, &frameTimes, recons, outBag);
      handleFrame<CompressedImage>(msg, &frameTimes, recons, outBag);
    }
    numMessages++;
  }
  return (numMessages);
}

size_t process_bag(
  const std::string & inBagName, const std::string & outBagName,
  const std::string &timeStampFile,
  const std::vector<std::string> & inTopics,
  const std::vector<std::string> & outTopics,
  const std::vector<std::string> & frameTopics, int cutoffPeriod,
  bool hasSyncCable, double fps, bool writePNG)
{
  OutBagWriter writer(outBagName, writePNG);

  const double fillRatio = 0.6;
  const int tileSize = 2;
  std::unordered_map<std::string, ApproxRecon> recons;
  for (size_t i = 0; i < inTopics.size(); i++) {
    recons.insert(
      {inTopics[i],
       ApproxRecon(&writer, outTopics[i], cutoffPeriod, fillRatio, tileSize)});
  }

  rosbag::Bag inBag;
  inBag.open(inBagName, rosbag::bagmode::Read);

  size_t numMessages(0);
  if (fps > 0) {
    numMessages =
      processFreeRunning(fps, inBag, inTopics, hasSyncCable, &recons);
  } else {
    if (timeStampFile.empty()) {
    numMessages =
      processFrameBased(frameTopics, inBag, inTopics, &recons, writer.getBag());
    } else {
       numMessages =  processOnTimeStamps(timeStampFile, inBag, inTopics, hasSyncCable, &recons);

    }
  }
  std::cout << "processed " << numMessages << " messages." << std::endl;

  return (0);
}

#endif  // BAG_TO_FRAMES_ROS1_HPP_
