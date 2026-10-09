# Tilt-tricopter impedance simulation

This setup connects the local Gazebo Classic tilt-tricopter model to
ArduCopter SITL. Gazebo supplies two inputs used by Impedance mode:

- the front-rod contact wrench, encoded as the ADM002 5-byte stream;
- a forward ray sensor, encoded as MAVLink `DISTANCE_SENSOR`.

Build ArduCopter and the Gazebo plugin first:

```sh
cd /home/pix/firmare/llw-apm
./waf configure --board sitl
./waf copter

cd /home/pix/uavros_ws
task erb-tilt_tricopter:tilt_tricopter
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin build uav_gazebo_plugin
```

Start Gazebo in the first terminal:

```sh
source /opt/ros/noetic/setup.bash
source /home/pix/uavros_ws/devel/setup.bash
roslaunch uav_gazebo spawn.launch world_name:=tilt_tricopter
```

Start SITL in a second terminal:

```sh
cd /home/pix/firmare/llw-apm/tricopter
../Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --console \
    --udp \
    --add-param-file=mav.parm \
    --add-param-file=../Tools/autotest/default_params/gazebo-tilt-tricopter-impedance.parm \
    -A '--serial6=udpclient:127.0.0.1:9024' \
    -A '--serial7=udpclient:127.0.0.1:9025'
```

Use `-w` on the first run, or after changing startup parameters. Do not use
`-w` when an existing `eeprom.bin` must be preserved.

The loopback UDP client for SERIAL7 binds its source to `127.0.0.1:9026`
when connecting to `127.0.0.1:9025`. The rangefinder bridge can send replies
to this fixed endpoint. This setup supports one SITL instance per host.

Before engaging Impedance mode, verify the two telemetry paths:

```text
status
watch DISTANCE_SENSOR
```

The expected controller sequence is:

```text
force tare complete -> approach -> confirming contact -> force hold
```

Use a centered pitch input before selecting Impedance mode. The current abort
logic switches to AltHold when normalized positive pitch is greater than 0.6.
The SITL defaults also leave the tilt controls away from their neutral states;
set RC7 to 1500 and RC8 to 1000 before arming:

```text
rc 2 1500
rc 7 1500
rc 8 1000
```

Keep the first validation run at the conservative gains in the parameter file;
do not treat stable simulation contact as evidence that the real vehicle is
safe to fly.
