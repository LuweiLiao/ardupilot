# uni350 Gazebo / ROS 2 SITL

`mav.parm` is the SITL profile for the existing UAVROS uni350 model.
Load it after `Tools/autotest/default_params/copter.parm` in a fresh SITL working directory.
Gazebo must be running first. Use `--model Gazebo`, input port 9003 and output port 9002.

The original `ArduRotorNormPlugin` maps motor channels `[0, 3, 1, 2]`;
`FRAME_CLASS=1`, `FRAME_TYPE=1` select Quad X.
The RotorS model uses 1000 rad/s normalized motor commands, mass 5.75 kg total,
and thrust coefficient 8.599e-5. Hover speed is approximately 405 rad/s.
`MOT_THST_HOVER=0.405` is a starting setting with `MOT_THST_EXPO=0`.
No vehicle geometry or dynamics were tuned in this adaptation.

Validated on 2026-09-18 with ROS 2 Jazzy / Gazebo Harmonic:
normal arm, GUIDED takeoff to 3 m, LOITER hover, LAND and automatic disarm.
Detailed launch instructions and acceptance evidence are in the companion
uavros2_ws repository at `docs/uni350-sitl.md`.
These parameters are simulation-only and have not been calibrated for hardware.

Flight validation used the existing local SITL binary from the
`codex/powerline-vision-perching` development checkout (HEAD `554891b013`),
with SHA256 `01c9a97f9d630b06587d4fc5caf5a5a75294a52d0f52980b1dbbe717d50e55a9`.
Publishing this parameter profile to master does not imply a separate
flight validation of a binary rebuilt from master.
