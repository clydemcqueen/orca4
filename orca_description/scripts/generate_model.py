#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 Clyde McQueen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Generate the model.sdf file by substituting strings of the form "@foo" with calculated values

The SDF file uses the ArduPilotPlugin COMMAND control method; this sends commands to a specified
ign-transport topic rather than directly controlling a joint.

We use the COMMAND method to send commands to the Gazebo Sim ThrusterPlugin. The ThrusterPlugin
supports 2 control methods:
      control thrust via /cmd_thrust
      control angular velocity via /cmd_vel

The ThrusterPlugin uses the Fossen equation to relate thrust to angular velocity, and will apply
thrust force to the joint and spin the propeller. Propellers have bounding boxes and inertia, so
spinning the propeller does affect the simulation.
"""

import math
import re
import sys

# SDF 1.9 supports degrees="true"; provide some nice vars for earlier versions
d180 = math.pi
d90 = d180 / 2
d45 = d90 / 2
d135 = d90 + d45

mass = 10
visual_x = 0.457
visual_y = 0.338
visual_z = 0.25
fluid_density = 1000
dome_x = visual_x / 2

# The ROV should be positively buoyant
buoyancy_adjustment = 0.05
displaced_mass = mass + buoyancy_adjustment

# The collision box is used by the BuoyancyPlugin
# collision_x * collision_y * collision_z * density == displaced_mass
collision_x = visual_x
collision_y = visual_y
collision_z = displaced_mass / (visual_x * visual_y * fluid_density)

# The center of mass is just above the origin
mass_z = 0.011

# The center of volume is directly above the center of mass, resulting in a restoring force
volume_z = 0.06

ixx = mass / 12 * (collision_y * collision_y + collision_z * collision_z)
iyy = mass / 12 * (collision_x * collision_x + collision_z * collision_z)
izz = mass / 12 * (collision_x * collision_x + collision_y * collision_y)

# 2nd order stability for the HydrodynamicsPlugin
xUU = -0.5 * visual_y * visual_z * 0.8 * fluid_density
yVV = -0.5 * visual_x * visual_z * 0.95 * fluid_density
zWW = -0.5 * visual_x * visual_y * 0.95 * fluid_density
kPP = -0.5 * 0.008 * fluid_density
mQQ = -0.5 * 0.008 * fluid_density
nRR = -0.5 * 0.008 * fluid_density

# Thruster placement
thruster_x = 0.15
thruster_y = 0.09
thruster_z = -0.009
vert_thruster_y = 0.105
vert_thruster_z = 0.09

# Propeller link parameters
propeller_size = "0.1 0.02 0.01"
propeller_mass = 0.002
propeller_ixx = 0.001
propeller_iyy = 0.001
propeller_izz = 0.001

# ThrusterPlugin parameters
propeller_diameter = 0.1
thrust_coefficient = 0.02

# Max thrust force, N
# Both forward and reverse thrust must be the same
max_thrust = 50

# ArduPilotPlugin control parameters
servo_min = 1100
servo_max = 1900
control_offset = -0.5

# From the command line
use_angvel_cmd = False

# Set by update_globals()
cw_control_multiplier = 0   # Thrusters 3, 4 and 6
ccw_control_multiplier = 0  # Thrusters 1, 2 and 5
thruster1_topic = "/model/orca4/joint/thruster1_joint/cmd_"
thruster2_topic = "/model/orca4/joint/thruster2_joint/cmd_"
thruster3_topic = "/model/orca4/joint/thruster3_joint/cmd_"
thruster4_topic = "/model/orca4/joint/thruster4_joint/cmd_"
thruster5_topic = "/model/orca4/joint/thruster5_joint/cmd_"
thruster6_topic = "/model/orca4/joint/thruster6_joint/cmd_"

# Stereo camera
# TODO(clyde) adjust mass, add collision (buoyancy) volume, etc.
stereo_baseline = 0.36  # Use same baseline as Orca3 for now
camera_radius = 0.0275
camera_height = 0.135
camera_x = -0.18
camera_y = stereo_baseline / 2
camera_z = (camera_height - visual_z) / 2 - 0.01
camera_sensor_z = -camera_height / 2 - 0.02
camera_mass = 0.001
camera_ixx = 0.001
camera_iyy = 0.001
camera_izz = 0.001
camera_far_clip = 4  # Furthest distance the camera can "see"

# Fossen equation, see "Guidance and Control of Ocean Vehicles" p. 246
def thrust_to_ang_vel(thrust):
    assert thrust >= 0
    assert thrust_coefficient >= 0
    return math.sqrt(thrust / (fluid_density * thrust_coefficient * pow(propeller_diameter, 4)))


def update_globals():
    global cw_control_multiplier
    global ccw_control_multiplier
    global thruster1_topic
    global thruster2_topic
    global thruster3_topic
    global thruster4_topic
    global thruster5_topic
    global thruster6_topic

    if use_angvel_cmd:
        print("control method: angular velocity")
        thruster1_topic += "vel"
        thruster2_topic += "vel"
        thruster3_topic += "vel"
        thruster4_topic += "vel"
        thruster5_topic += "vel"
        thruster6_topic += "vel"

        # Angular velocity range in rad/s
        # Thrust ~ sqrt(angular velocity), so the curves are quite different
        # Reverse the angular velocity for thrusters 3, 4 and 6
        cw_control_multiplier = -thrust_to_ang_vel(max_thrust) * 2
        ccw_control_multiplier = thrust_to_ang_vel(max_thrust) * 2
    else:
        print("control method: thrust force")
        thruster1_topic += "thrust"
        thruster2_topic += "thrust"
        thruster3_topic += "thrust"
        thruster4_topic += "thrust"
        thruster5_topic += "thrust"
        thruster6_topic += "thrust"

        # Force range [-50, 50] in N
        cw_control_multiplier = max_thrust * 2
        ccw_control_multiplier = max_thrust * 2


def generate_model(input_path, output_path):
    s = open(input_path, "r").read()
    pattern = re.compile(r"@(\w+)")
    # globals()['foo'] will return the value of foo
    # TODO(clyde) trim floats
    s = re.sub(pattern, lambda m: str(globals()[m.group(1)]), s)
    open(output_path, "w").write(s)


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage:")
        print("generate_model.py infile outfile 0|1")
        print("0: control thrust force")
        print("1: control angular velocity")
        exit(-100)

    use_angvel_cmd = bool(int(sys.argv[3]))

    update_globals()

    generate_model(sys.argv[1], sys.argv[2])
