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

#include "orca_shared/pwm.hpp"

#include "orca_shared/util.hpp"

namespace orca
{

uint16_t tilt_to_pwm(const int tilt)
{
  return orca::scale(
    tilt, TILT_MIN, TILT_MAX,
    PWM_TILT_45_UP, PWM_TILT_45_DOWN);
}

int pwm_to_tilt(const uint16_t pwm)
{
  return orca::scale(
    pwm, PWM_TILT_45_UP,
    PWM_TILT_45_DOWN,
    TILT_MIN, TILT_MAX);
}

uint16_t brightness_to_pwm(const int brightness)
{
  return orca::scale(
    brightness, BRIGHTNESS_MIN, BRIGHTNESS_MAX,
    PWM_LIGHTS_OFF, PWM_LIGHTS_FULL);
}

int pwm_to_brightness(const uint16_t pwm)
{
  return orca::scale(
    pwm, PWM_LIGHTS_OFF, PWM_LIGHTS_FULL,
    BRIGHTNESS_MIN, BRIGHTNESS_MAX);
}

uint16_t effort_to_pwm(const uint16_t thrust_dz_pwm, const double effort)
{
  uint16_t thrust_range_pwm = 400 - thrust_dz_pwm;

  return clamp(
    static_cast<uint16_t>(PWM_THRUST_STOP +
    (effort > THRUST_STOP ? thrust_dz_pwm : (effort < THRUST_STOP ?
    -thrust_dz_pwm : 0)) +
    std::round(effort * thrust_range_pwm)),
    PWM_THRUST_FULL_REV,
    PWM_THRUST_FULL_FWD);
}

double pwm_to_effort(const uint16_t thrust_dz_pwm, const uint16_t pwm)
{
  uint16_t thrust_range_pwm = 400 - thrust_dz_pwm;

  return static_cast<double>(
    pwm - PWM_THRUST_STOP +
    (pwm > PWM_THRUST_STOP ? -thrust_dz_pwm :
    (pwm < PWM_THRUST_STOP ? thrust_dz_pwm : 0))) /
         thrust_range_pwm;
}

}  // namespace orca
