// MIT License
//
// Copyright (c) 2022 Clyde McQueen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Simple ROS2 parameter macro that declares, gets and logs a parameter
// Inspired by https://github.com/ptrmu/ros2_shared

#ifndef ORCA_NAV2__PARAM_MACRO_HPP_
#define ORCA_NAV2__PARAM_MACRO_HPP_

#include "nav2_util/node_utils.hpp"

#define PARAMETER(node, prefix, param, default) \
  nav2_util::declare_parameter_if_not_declared( \
    node, prefix + "." + #param, rclcpp::ParameterValue(default)); \
  node->get_parameter(prefix + "." + #param, param ## _); \
  std::cout << prefix << "." << #param << " = " << param ## _ << std::endl;

#endif  // ORCA_NAV2__PARAM_MACRO_HPP_
