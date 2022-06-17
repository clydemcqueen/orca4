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

#ifndef ORCA_SHARED__MODEL_HPP_
#define ORCA_SHARED__MODEL_HPP_

#include <cmath>

#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "orca_msgs/msg/effort.hpp"
#include "rclcpp/logger.hpp"
#include "ros2_shared/context_macros.hpp"

namespace orca
{

// Orca motion model and dynamics. Basically everything is in the body frame.
// _x forward
// _y strafe
// _z vertical
// _yaw yaw
// roll and pitch are always 0

#define MODEL_PARAMS \
  CXT_MACRO_MEMBER(mdl_mass, double, 9.75) \
  CXT_MACRO_MEMBER(mdl_volume, double, 0.01) \
  CXT_MACRO_MEMBER(mdl_fluid_density, double, 997) \
  /* kg/m^3, 997 for freshwater, 1029 for seawater  */ \
  CXT_MACRO_MEMBER(mdl_thrust_scale, double, 0.7) \
  /* Scale max thruster forces to give a better linear approximation  */ \
  CXT_MACRO_MEMBER(mdl_drag_coef_x, double, 0.8) \
  /* Forward drag, 1.0 is a box  */ \
  CXT_MACRO_MEMBER(mdl_drag_coef_y, double, 0.95) \
  /* Strafe drag  */ \
  CXT_MACRO_MEMBER(mdl_drag_coef_z, double, 0.95) \
  /* Vertical drag  */ \
  CXT_MACRO_MEMBER(mdl_drag_partial_const_yaw, double, 0.004) \
  /* Yaw drag, wild guess  */ \
  CXT_MACRO_MEMBER(mdl_thrust_dz_pwm, uint16_t, 35) \
  /* Thruster deadzone  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct Model
{
  //=====================================================================================
  // Physics constants
  //=====================================================================================

  static constexpr double GRAVITY = 9.8;  // Gazebo default is 9.8, 9.80665 is a bit more accurate

  //=====================================================================================
  // Sensor constants
  //=====================================================================================

  static constexpr double BARO_STDDEV = 201.7;      // Measured during ft3
  static constexpr double BARO_FREQ = 20.0;         // Slowed down for Orca
  static constexpr double DEPTH_STDDEV = 0.02064;   // Measured during ft3

  //=====================================================================================
  // Vehicle constants, in the body frame (x forward, y left, z up)
  //=====================================================================================

  static constexpr double ROV_DIM_X = 0.457;       // Length
  static constexpr double ROV_DIM_Y = 0.338;       // Width
  static constexpr double ROV_DIM_Z = 0.254;       // Height

  static constexpr double TETHER_DIAM = 0.008;

  static constexpr double ROV_AREA_BOW = ROV_DIM_Y * ROV_DIM_Z;   // Bow and stern
  static constexpr double ROV_AREA_TOP = ROV_DIM_X * ROV_DIM_Y;   // Top and bottom
  static constexpr double ROV_AREA_PORT = ROV_DIM_X * ROV_DIM_Z;  // Port and starboard

  // From BlueRobotics specs, in Newtons
  static constexpr double T200_MAX_POS_FORCE = 50;
  static constexpr double T200_MAX_NEG_FORCE = 40;

  // bollard_force_xy_ could also be called bollard_force_fs_
  static constexpr double BOLLARD_FORCE_XY = 0.76 * 2 * (T200_MAX_POS_FORCE + T200_MAX_NEG_FORCE);

  // Up and down forces are different
  static constexpr double BOLLARD_FORCE_Z_UP = 2 * T200_MAX_POS_FORCE;
  static constexpr double BOLLARD_FORCE_Z_DOWN = 2 * T200_MAX_NEG_FORCE;

  // Estimate maximum yaw torque by looking at 4 thrusters (2 forward, 2 reverse),
  // each mounted ~tangent to a circle with radius = 18cm
  static constexpr double MAX_TORQUE_YAW = 0.18 * 2 * (T200_MAX_POS_FORCE + T200_MAX_NEG_FORCE);

  //=====================================================================================
  // Parameters
  //=====================================================================================

  MODEL_PARAMS

  //=====================================================================================
  // Force / torque <=> acceleration
  //=====================================================================================

  // Assume a uniform distribution of mass in the vehicle box
  double moment_of_inertia_yaw_ = mdl_mass_ / 12.0 *
    (ROV_DIM_X * ROV_DIM_X + ROV_DIM_Y * ROV_DIM_Y);

  // Force / torque => acceleration
  [[nodiscard]] double force_to_accel(double force) const {return force / mdl_mass_;}

  [[nodiscard]] double torque_to_accel_yaw(double torque_yaw) const
  {
    return torque_yaw / moment_of_inertia_yaw_;
  }

  // Acceleration => force / torque
  [[nodiscard]] double accel_to_force(double accel) const {return mdl_mass_ * accel;}

  [[nodiscard]] double accel_to_torque_yaw(double accel_yaw) const
  {
    return moment_of_inertia_yaw_ * accel_yaw;
  }

  //=====================================================================================
  // Force / torque <=> effort
  // Effort is force / max_force, and ranges from -1.0 to 1.0
  //=====================================================================================

  [[nodiscard]] double bollard_force_xy() const {return BOLLARD_FORCE_XY * mdl_thrust_scale_;}

  [[nodiscard]] double bollard_force_z_up() const {return BOLLARD_FORCE_Z_UP * mdl_thrust_scale_;}

  [[nodiscard]] double bollard_force_z_down() const
  {
    return BOLLARD_FORCE_Z_DOWN * mdl_thrust_scale_;
  }

  [[nodiscard]] double max_torque_yaw() const {return MAX_TORQUE_YAW * mdl_thrust_scale_;}

  [[nodiscard]] double force_to_effort_xy(double force_xy) const
  {
    return force_xy / bollard_force_xy();
  }

  [[nodiscard]] double force_to_effort_z(double force_z) const
  {
    return force_z / (force_z > 0 ? bollard_force_z_up() : bollard_force_z_down());
  }

  [[nodiscard]] double torque_to_effort_yaw(double torque_yaw) const
  {
    return torque_yaw / max_torque_yaw();
  }

  [[nodiscard]] double effort_to_force_xy(double effort_xy) const
  {
    return effort_xy * bollard_force_xy();
  }

  [[nodiscard]] double effort_to_force_z(double effort_z) const
  {
    return effort_z * (effort_z > 0 ? bollard_force_z_up() : bollard_force_z_down());
  }

  [[nodiscard]] double effort_to_torque_yaw(double effort_yaw) const
  {
    return effort_yaw * max_torque_yaw();
  }

  //=====================================================================================
  // Acceleration <=> effort
  //=====================================================================================

  [[nodiscard]] double accel_to_effort_xy(double accel_xy) const
  {
    return force_to_effort_xy(accel_to_force(accel_xy));
  }

  [[nodiscard]] double accel_to_effort_z(double accel_z) const
  {
    return force_to_effort_z(accel_to_force(accel_z));
  }

  [[nodiscard]] double accel_to_effort_yaw(double accel_yaw) const
  {
    return torque_to_effort_yaw(accel_to_torque_yaw(accel_yaw));
  }

  [[nodiscard]] double effort_to_accel_xy(double effort_xy) const
  {
    return force_to_accel(effort_to_force_xy(effort_xy));
  }

  [[nodiscard]] double effort_to_accel_z(double effort_z) const
  {
    return force_to_accel(effort_to_force_z(effort_z));
  }

  [[nodiscard]] double effort_to_accel_yaw(double effort_yaw) const
  {
    return torque_to_accel_yaw(effort_to_torque_yaw(effort_yaw));
  }

  //=====================================================================================
  // Water pressure depends on fluid density
  //=====================================================================================

  [[nodiscard]] double pressure_to_z(double atmospheric_pressure, double pressure) const
  {
    return -(pressure - atmospheric_pressure) / (mdl_fluid_density_ * GRAVITY);
  }

  [[nodiscard]] double z_to_pressure(double atmospheric_pressure, double z) const
  {
    return mdl_fluid_density_ * GRAVITY * -z + atmospheric_pressure;
  }

  [[nodiscard]] double atmospheric_pressure(double pressure, double z) const
  {
    return pressure - mdl_fluid_density_ * GRAVITY * -z;
  }

  // Mass displaced by the volume, in kg
  [[nodiscard]] double displaced_mass() const {return mdl_volume_ * mdl_fluid_density_;}

  // Weight in water, in Newtons (kg * m/s^2)
  [[nodiscard]] double weight_in_water() const {return GRAVITY * (mdl_mass_ - displaced_mass());}

  // Z acceleration required to hover, in m/s^2
  [[nodiscard]] double hover_accel_z() const {return weight_in_water() / mdl_mass_;}

  [[nodiscard]] double hover_force_z() const {return accel_to_force(hover_accel_z());}

  //=====================================================================================
  // Drag in the body frame (x forward, y left, z up)
  // = 0.5 * density * area * velocity^2 * coefficient
  //
  // The ROV constants below capture all but velocity:
  //    drag constant = 0.5 * density * area * coefficient
  //=====================================================================================

  [[nodiscard]] double drag_const_x() const
  {
    return 0.5 * mdl_fluid_density_ * ROV_AREA_BOW * mdl_drag_coef_x_;
  }

  [[nodiscard]] double drag_const_y() const
  {
    return 0.5 * mdl_fluid_density_ * ROV_AREA_PORT * mdl_drag_coef_y_;
  }

  [[nodiscard]] double drag_const_z() const
  {
    return 0.5 * mdl_fluid_density_ * ROV_AREA_TOP * mdl_drag_coef_z_;
  }

  [[nodiscard]] double drag_const_yaw() const
  {
    return mdl_fluid_density_ * mdl_drag_partial_const_yaw_;
  }

  //=====================================================================================
  // Drag force, and acceleration-due-to-drag, in the body frame (x forward, y left, z up)
  //=====================================================================================

  // Velocity => drag force / torque
  // Motion works in all 4 quadrants, note the use of abs()
  static double drag_force(double velo, double drag_constant)
  {
    return velo * std::abs(velo) * -drag_constant;
  }

  [[nodiscard]] double drag_force_x(double velo_x) const
  {
    return drag_force(velo_x, drag_const_x());
  }

  [[nodiscard]] double drag_force_y(double velo_y) const
  {
    return drag_force(velo_y, drag_const_y());
  }

  [[nodiscard]] double drag_force_z(double velo_z) const
  {
    return drag_force(velo_z, drag_const_z());
  }

  [[nodiscard]] double drag_torque_yaw(double velo_yaw) const
  {
    return velo_yaw * std::abs(velo_yaw) * -drag_const_yaw();
  }

  // Velocity => acceleration due to drag
  [[nodiscard]] double drag_accel(double velo, double drag_constant) const
  {
    return force_to_accel(drag_force(velo, drag_constant));
  }

  [[nodiscard]] double drag_accel_x(double velo_x) const
  {
    return drag_accel(velo_x, drag_const_x());
  }

  [[nodiscard]] double drag_accel_y(double velo_y) const
  {
    return drag_accel(velo_y, drag_const_y());
  }

  [[nodiscard]] double drag_accel_z(double velo_z) const
  {
    return drag_accel(velo_z, drag_const_z());
  }

  [[nodiscard]] double drag_accel_yaw(double velo_yaw) const
  {
    return torque_to_accel_yaw(drag_torque_yaw(velo_yaw));
  }

  //=====================================================================================
  // Combine all 4 DoF
  //=====================================================================================

  [[nodiscard]] geometry_msgs::msg::Accel drag_accel(const geometry_msgs::msg::Twist & vel) const;

  [[nodiscard]] geometry_msgs::msg::Wrench accel_to_wrench(const geometry_msgs::msg::Accel & accel)
  const;

  [[nodiscard]] geometry_msgs::msg::Wrench drag_wrench(const geometry_msgs::msg::Twist & vel) const
  {
    return accel_to_wrench(drag_accel(vel));
  }

  // Wrench scaled by bollard force, clamped to [-1, 1]
  [[nodiscard]] orca_msgs::msg::Effort wrench_to_effort(const geometry_msgs::msg::Wrench & wrench)
  const;

  [[nodiscard]] orca_msgs::msg::Effort accel_to_effort(const geometry_msgs::msg::Accel & accel)
  const;

  //=====================================================================================
  // Logging
  //=====================================================================================

  // Log some info... handy for debugging
  void log_info(const rclcpp::Logger & logger) const;
};

}  // namespace orca

#endif  // ORCA_SHARED__MODEL_HPP_
