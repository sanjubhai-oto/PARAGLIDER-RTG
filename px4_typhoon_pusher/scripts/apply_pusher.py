#!/usr/bin/env python3
"""Add a rear PUSHER motor to PX4's gazebo-classic typhoon_h480 + write a pusher airframe.

Run ON YOUR MAC from inside a PX4-Autopilot checkout (or pass --px4 <path>).
Idempotent: re-running won't double-insert. Back up files before first run if you like.

  python3 apply_pusher.py --px4 ~/PX4-Autopilot

What it does:
  1. Inserts a 7th motor (forward-facing pusher) into
     Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/typhoon_h480/typhoon_h480.sdf.jinja
       - pusher link + joint (spin axis = body +X  -> thrust forward)
       - gazebo_motor_model plugin  motorNumber=6
       - mavlink_interface control_channel  input_index=6   (PX4 output slot 7)
  2. Writes airframe 6012_gazebo-classic_typhoon_h480_pusher that:
       - keeps the stock 6-rotor hexa control allocation (pusher NOT in attitude mix)
       - assigns PX4 output 7 (PWM_MAIN_FUNC7) to Offboard_Actuator_Set1 (=2000)
         so the pusher is driven MANUALLY via MAV_CMD_DO_SET_ACTUATOR / a script.
"""
import argparse, os, sys

PUSHER_LINK_JOINT = """
    <!-- ===== PUSHER (forward-thrust) motor, added by apply_pusher.py ===== -->
    <link name='rotor_pusher'>
      <pose>-0.26 0 0.06 0 1.5707963 0</pose>
      <inertial><mass>0.005</mass>
        <inertia><ixx>9.75e-07</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>0.000166</iyy><iyz>0</iyz><izz>0.000167</izz></inertia></inertial>
      <visual name='rotor_pusher_visual'>
        <geometry><cylinder><length>0.006</length><radius>0.10</radius></cylinder></geometry>
        <material><script><name>Gazebo/Orange</name>
          <uri>file://media/materials/scripts/gazebo.material</uri></script></material>
      </visual>
      <gravity>1</gravity><self_collide>0</self_collide>
    </link>
    <joint name='rotor_pusher_joint' type='revolute'>
      <child>rotor_pusher</child><parent>base_link</parent>
      <axis><xyz>1 0 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper><effort>10</effort><velocity>-1</velocity></limit>
        <dynamics><damping>0.005</damping></dynamics>
        <use_parent_model_frame>1</use_parent_model_frame></axis>
      <physics><ode><implicit_spring_damper>1</implicit_spring_damper></ode></physics>
    </joint>
"""

PUSHER_MOTOR_PLUGIN = """    <plugin name='pusher_motor_model' filename='libgazebo_motor_model.so'>
      <robotNamespace></robotNamespace>
      <jointName>rotor_pusher_joint</jointName>
      <linkName>rotor_pusher</linkName>
      <turningDirection>cw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1500</maxRotVelocity>
      <motorConstant>1.2e-05</motorConstant>
      <momentConstant>0.06</momentConstant>
      <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
      <motorNumber>6</motorNumber>
      <rotorDragCoefficient>0.000806428</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <motorSpeedPubTopic>/motor_speed/6</motorSpeedPubTopic>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    </plugin>
"""

PUSHER_CHANNEL = """        <channel name="pusher">
          <input_index>6</input_index>
          <input_offset>0</input_offset>
          <input_scaling>1500</input_scaling>
          <zero_position_disarmed>0</zero_position_disarmed>
          <zero_position_armed>0</zero_position_armed>
          <joint_control_type>velocity</joint_control_type>
        </channel>
"""

AIRFRAME = """#!/bin/sh
#
# @name Typhoon H480 + Pusher SITL
#
# @type Hexarotor x
#
# Stock typhoon_h480 hexa with an added forward PUSHER motor on output 7.
# Pusher is driven MANUALLY via MAV_CMD_DO_SET_ACTUATOR (Offboard Actuator Set 1).

. ${R}etc/init.d/airframes/6011_gazebo-classic_typhoon_h480

# --- Pusher output: PX4 output 7 = Offboard Actuator Set 1 (manual via DO_SET_ACTUATOR) ---
# NOTE: 2000 = Offboard_Actuator_Set1 in recent PX4. If your PX4 uses a different
#       OutputFunction enum value, set it here (or assign via QGC > Actuators).
param set-default PWM_MAIN_FUNC7 2000
param set-default PWM_MAIN_TIM0 -4
"""

def insert_once(text, anchor, snippet, marker):
    if marker in text:
        print(f"  [skip] {marker} already present"); return text
    i = text.find(anchor)
    if i < 0:
        print(f"  [WARN] anchor not found: {anchor[:40]!r} — insert manually"); return text
    return text[:i] + snippet + text[i:]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--px4", default=os.path.expanduser("~/PX4-Autopilot"))
    a = ap.parse_args()
    sdf = os.path.join(a.px4, "Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/typhoon_h480/typhoon_h480.sdf.jinja")
    if not os.path.isfile(sdf):
        sys.exit(f"typhoon SDF not found: {sdf}\nRun `make px4_sitl` once or `git submodule update --init` first.")
    t = open(sdf).read()
    t = insert_once(t, "    <plugin name='rosbag'", PUSHER_LINK_JOINT, "rotor_pusher_joint")
    t = insert_once(t, "    <plugin name='groundtruth_plugin'", PUSHER_MOTOR_PLUGIN, "pusher_motor_model")
    t = insert_once(t, '        <channel name="gimbal_roll">', PUSHER_CHANNEL, '<channel name="pusher">')
    open(sdf, "w").write(t)
    print(f"  [ok] patched {sdf}")

    af = os.path.join(a.px4, "ROMFS/px4fmu_common/init.d-posix/airframes/6012_gazebo-classic_typhoon_h480_pusher")
    open(af, "w").write(AIRFRAME)
    print(f"  [ok] wrote airframe {af}")
    cmake = os.path.join(a.px4, "ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt")
    if os.path.isfile(cmake):
        c = open(cmake).read()
        if "6012_gazebo-classic_typhoon_h480_pusher" not in c:
            c = c.replace("6011_gazebo-classic_typhoon_h480\n",
                          "6011_gazebo-classic_typhoon_h480\n\t6012_gazebo-classic_typhoon_h480_pusher\n")
            open(cmake, "w").write(c); print("  [ok] registered airframe in CMakeLists.txt")
        else:
            print("  [skip] airframe already in CMakeLists.txt")
    print("\nDone. Build & run on your Mac:")
    print("  cd", a.px4)
    print("  make px4_sitl gazebo-classic_typhoon_h480_pusher")

if __name__ == "__main__":
    main()
