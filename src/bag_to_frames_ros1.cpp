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

#include <event_array_codecs/decoder_factory.h>
#include <event_array_msgs/EventArray.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <unistd.h>

#include <memory>

#include "sync_event_frames/approx_reconstructor.hpp"
#include "sync_event_frames/frame_handler.hpp"

using event_array_msgs::EventArray;
using sensor_msgs::Image;
using ApproxRecon = sync_event_frames::ApproxReconstructor<
  EventArray, EventArray::ConstPtr, Image, Image::ConstPtr, ros::Time>;

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout
    << "bag_to_frames -i input_bag -o output_bag "
       "[-t input_topic] "
       "[-T output_topic] [-f fps] [-s (has sync cable)] [-c cutoff_period]"
    << std::endl;
}

class OutBagWriter : public sync_event_frames::FrameHandler<Image::ConstPtr>
{
public:
  explicit OutBagWriter(const std::string & bagName)
  {
    outBag_.open(bagName, rosbag::bagmode::Write);
  }
  void frame(const Image::ConstPtr & img, const std::string & topic) override
  {
    outBag_.write(topic, img->header.stamp, img);
    numFrames_++;
    if (numFrames_ % 100 == 0) {
      std::cout << "wrote " << numFrames_ << " frames " << std::endl;
    }
  }

private:
  rosbag::Bag outBag_;
  size_t numFrames_{0};
};

using sync_event_frames::ApproxReconstructor;

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
      auto m = msg.instantiate<EventArray>();
      if (!hasValidTime) {
        event_array_codecs::DecoderFactory decoderFactory;
        auto decoder =
          decoderFactory.getInstance(m->encoding, m->width, m->height);
        if (!decoder) {
          std::cerr << "invalid encoding: " << m->encoding << std::endl;
          throw(std::runtime_error("invalid encoding!"));
        }
        uint64_t firstSensorTime{0};
        if (decoder->findFirstSensorTime(
              m->events.data(), m->events.size(), &firstSensorTime)) {
          nextSensorFrameTime =
            (firstSensorTime / sliceInterval) * sliceInterval;
          nextROSFrameTime =
            m->header.stamp +
            ros::Duration().fromNSec(firstSensorTime % sliceInterval);
          hasValidTime = true;
        } else {
          std::cout << "WARNING: no time stamp found in packet!" << std::endl;
          continue;  // never tested!
        }
      }
      it->second.processMsg(m);
      // call processMsg() *before* adding the frames such that the
      // offset between sensor and ROS time is initialized when frame time
      // is added
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

int main(int argc, char ** argv)
{
  int opt;
  std::string inBagName;
  std::string outBagName;
  std::vector<std::string> inTopics;
  std::vector<std::string> outTopics;
  int cutoff_period(30);
  bool hasSyncCable{false};
  double fps(25);
  while ((opt = getopt(argc, argv, "i:o:t:T:f:c:sh")) != -1) {
    switch (opt) {
      case 'i':
        inBagName = optarg;
        break;
      case 'o':
        outBagName = optarg;
        break;
      case 't':
        inTopics.push_back(std::string(optarg));
        break;
      case 'T':
        outTopics.push_back(std::string(optarg));
        break;
      case 'f':
        fps = atof(optarg);
        break;
      case 'c':
        cutoff_period = atoi(optarg);
        break;
      case 's':
        hasSyncCable = true;
        break;
      case 'h':
        usage();
        return (-1);
        break;
      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }

  if (inBagName.empty()) {
    std::cout << "missing input bag file name!" << std::endl;
    usage();
    return (-1);
  }

  if (outBagName.empty()) {
    std::cout << "missing output bag file name!" << std::endl;
    usage();
    return (-1);
  }

  if (inTopics.empty()) {
    std::cout << "no input topics found!" << std::endl;
    return (-1);
  }
  if (outTopics.empty()) {
    for (const auto s : inTopics) {
      outTopics.push_back(s + "/image_raw");
    }
  }

  if (outTopics.size() != inTopics.size()) {
    std::cout << "must have same number of input and output topics!"
              << std::endl;
    return (-1);
  }

  if (fps < 1e-5) {
    std::cout << "fps too small: " << fps << std::endl;
    return (-1);
  }

  const double fillRatio = 0.6;
  const int tileSize = 2;
  OutBagWriter writer(outBagName);
  std::unordered_map<std::string, ApproxRecon> recons;
  for (size_t i = 0; i < inTopics.size(); i++) {
    recons.insert(
      {inTopics[i],
       ApproxRecon(&writer, outTopics[i], cutoff_period, fillRatio, tileSize)});
  }

  rosbag::Bag inBag;
  inBag.open(inBagName, rosbag::bagmode::Read);

  size_t numMessages =
    processFreeRunning(fps, inBag, inTopics, hasSyncCable, &recons);
  std::cout << "processed " << numMessages << " messages." << std::endl;

  return (0);
}
