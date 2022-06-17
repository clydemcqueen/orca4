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

#ifndef ORCA_SHARED__PWM_HPP_
#define ORCA_SHARED__PWM_HPP_

#include <cstdint>

namespace orca
{
//=============================================================================
// Camera tilt servo
//=============================================================================

constexpr int TILT_MIN = -45;
constexpr int TILT_MAX = 45;

constexpr uint16_t PWM_TILT_45_UP = 1100;
constexpr uint16_t PWM_TILT_0 = 1500;
constexpr uint16_t PWM_TILT_45_DOWN = 1900;

uint16_t tilt_to_pwm(int tilt);

int pwm_to_tilt(uint16_t pwm);

//=============================================================================
// BlueRobotics Lumen subsea light
//=============================================================================

constexpr int BRIGHTNESS_MIN = 0;
constexpr int BRIGHTNESS_MAX = 100;

constexpr uint16_t PWM_LIGHTS_OFF = 1100;
constexpr uint16_t PWM_LIGHTS_FULL = 1900;

uint16_t brightness_to_pwm(int brightness);

int pwm_to_brightness(uint16_t pwm);

//=============================================================================
// BlueRobotics T200 thruster + ESC
//=============================================================================

constexpr double THRUST_FULL_REV = -1.0;
constexpr double THRUST_STOP = 0.0;
constexpr double THRUST_FULL_FWD = 1.0;

constexpr uint16_t PWM_THRUST_FULL_REV = 1100;
constexpr uint16_t PWM_THRUST_STOP = 1500;
constexpr uint16_t PWM_THRUST_FULL_FWD = 1900;

uint16_t effort_to_pwm(uint16_t thrust_dz_pwm, double effort);

double pwm_to_effort(uint16_t thrust_dz_pwm, uint16_t pwm);

}  // namespace orca

#endif  // ORCA_SHARED__PWM_HPP_
