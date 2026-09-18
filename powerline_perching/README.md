# Powerline perching SITL

AI-assisted ROS 2 / Gazebo simulation integration requested on 2026-09-17.
Branch: `codex/powerline-vision-perching`, based on `641838a774`.
Pre-existing UARTDriver.cpp and GCS_Common.cpp edits are preserved and are not
part of this feature. No change to autopilot control laws is needed: the
companion uses existing LOITER RC input and GUIDED MAVLink velocity targets.

`mav.parm` is a simulation profile, not a hardware parameter recommendation.
Load after `Tools/autotest/default_params/copter.parm` with a fresh SITL
working directory. The Gazebo backend uses UDP 9002/9003; MAVLink uses TCP 5760.
The ROS workspace contains the camera, line detector, scenario and orchestration.
Do not connect the companion to a physical vehicle.
