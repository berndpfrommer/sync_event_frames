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

#ifndef ROS_COMPAT_HPP_
#define ROS_COMPAT_HPP_

#ifdef USING_ROS_1
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

namespace ros_compat
{
#ifdef USING_ROS_1
uint64_t to_nanoseconds(const ros::Time & t) { return (t.toNSec()); }
#else
uint64_t to_nanoseconds(const rclcpp::Time & t) { return (t.to_nanoseconds()); }
#endif
}  // namespace ros_compat
#endif  // ROS_COMPAT_HPP_
