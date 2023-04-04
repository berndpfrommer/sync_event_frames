# -*- cmake -*-
# Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

add_compile_options(-Wall -Wextra -Wpedantic -Werror)
add_definitions(-DUSING_ROS_1)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(simple_image_recon_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  rosbag
  event_array_msgs
  event_array_codecs
  sensor_msgs
  image_transport)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

#
# -------- bag_to_frames
#

add_executable(bag_to_frames src/bag_to_frames_ros1.cpp)
target_include_directories(bag_to_frames PUBLIC include)
target_link_libraries(bag_to_frames ${ALL_LIBS})


# the node must go into the paroject specific lib directory or else
# the launch file will not find it

install(TARGETS
  bag_to_frames
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})