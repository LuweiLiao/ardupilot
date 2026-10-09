# Tilt-quadcopter simulation parameters

`tiltquad.parm` is the original parameter snapshot. `mav_0_1.parm` is a
separate snapshot with the impedance contact controller settings, ADM002
input on SERIAL6, and MAVLink rangefinder input on SERIAL7 enabled.

The new snapshot also changes motor tilt directions and mixing, hover thrust,
and servo assignments. It includes learned sensor offsets and flight
statistics from the source session. Treat it as a simulation snapshot;
loading it replaces the corresponding vehicle settings.

These snapshots are not hardware calibration profiles. Gazebo flight and
contact validation must use the matching model and actuator mapping.
