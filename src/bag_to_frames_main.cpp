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

#include <unistd.h>

#include <chrono>
#include <iostream>

#ifdef USING_ROS_1
#include "bag_to_frames_ros1.hpp"
#else
#include "bag_to_frames_ros2.hpp"
#endif

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_frames -i input_bag -o output_bag"
               " -t event_camera_input_topic [-T event_frame_output_topic]"
               " [-c frame_camera_input_topic] [-f fps]"
               " [-C cutoff_period]"
            << std::endl;
}

int main(int argc, char ** argv)
{
  int opt;
  std::string inBagName;
  std::string outBagName;
  std::vector<std::string> inTopics;
  std::vector<std::string> outTopics;
  std::vector<std::string> frameTopics;
  int cutoffPeriod(30);
  bool hasSyncCable{false};
  double fps(-1);
  while ((opt = getopt(argc, argv, "i:o:t:T:f:C:c:sh")) != -1) {
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
        frameTopics.emplace_back(optarg);
        break;
      case 'C':
        cutoffPeriod = atoi(optarg);
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

  if (fps < 0 && frameTopics.empty()) {
    std::cout << "must specify either frame camera topics or fps!" << std::endl;
    return (-1);
  }

  if (fps > 0 && !frameTopics.empty()) {
    std::cout << "cannot specify fps and frame camera topics simultaneously!"
              << std::endl;
    return (-1);
  }
  auto start = std::chrono::high_resolution_clock::now();

  size_t numMessages = process_bag(
    inBagName, outBagName, inTopics, outTopics, frameTopics, cutoffPeriod,
    hasSyncCable, fps);
  auto final = std::chrono::high_resolution_clock::now();
  auto dt =
    std::chrono::duration_cast<std::chrono::microseconds>(final - start);
  std::cout << "processed " << numMessages << " messages in "
            << dt.count() * 1e-6 << " s" << std::endl;

  return (0);
}
