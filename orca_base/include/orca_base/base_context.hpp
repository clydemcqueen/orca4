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

#ifndef ORCA_BASE__BASE_CONTEXT_HPP_
#define ORCA_BASE__BASE_CONTEXT_HPP_

#include <string>

#include "orca_shared/model.hpp"

namespace orca_base
{

#define BASE_PARAMS \
  CXT_MACRO_MEMBER(map_frame_id, std::string, "map") \
  CXT_MACRO_MEMBER(slam_frame_id, std::string, "slam") \
  CXT_MACRO_MEMBER(down_frame_id, std::string, "down") \
  CXT_MACRO_MEMBER(odom_frame_id, std::string, "odom") \
  CXT_MACRO_MEMBER(base_frame_id, std::string, "base_link") \
  CXT_MACRO_MEMBER(camera_frame_id, std::string, "left_camera_link") \
 \
  CXT_MACRO_MEMBER(slam_timeout_ms, int, 1000) \
  CXT_MACRO_MEMBER(transform_expiration_ms, int, 0) \
 \
  CXT_MACRO_MEMBER(timer_rate, int, 20) \
 \
  CXT_MACRO_MEMBER(x_vel, double, 0.4) \
  CXT_MACRO_MEMBER(y_vel, double, 0.4) \
  CXT_MACRO_MEMBER(z_vel, double, 0.2) \
  CXT_MACRO_MEMBER(yaw_vel, double, 0.4) \
 \
  CXT_MACRO_MEMBER(x_accel, double, 0.4) \
  CXT_MACRO_MEMBER(y_accel, double, 0.4) \
  CXT_MACRO_MEMBER(z_accel, double, 0.2) \
  CXT_MACRO_MEMBER(yaw_accel, double, 0.4) \
 \
  CXT_MACRO_MEMBER(coast, bool, false) \
  /* Coast to decelerate (vs powered decel), useful for ROV ops */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct BaseContext : orca::Model
{
  CXT_MACRO_DEFINE_MEMBERS(BASE_PARAMS)
};

#define BASE_ALL_PARAMS \
  MODEL_PARAMS \
    BASE_PARAMS \
/* End of list */

}  // namespace orca_base

#endif  // ORCA_BASE__BASE_CONTEXT_HPP_
