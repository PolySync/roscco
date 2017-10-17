# ROSCCO

## Overview

ROSCCO is an interface between ROS and OSCC allowing ROS users to interface with
OSCC from their own ROS nodes utilizing ROS messages created from the OSCC API.

## Getting Started

### Dependencies

ROSCCO is built on the Linux implementation of ROS Kinetic. The only known
limitation for using older versions is that the tests require C++11.

To install ROS Kinetic see the [install instructions](http://wiki.ros.org/kinetic/Installation) on your Linux distribution.

### Building ROSCCO in a catkin project

ROSCCO uses the OSCC API which builds for specific vehicle models. OSCC is a
git submodule so that it can be built during the catkin build process.

```
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone --recursive https://github.com/PolySync/roscco.git
cd ..
source /opt/ros/kinetic/setup.bash
catkin_make -DKIA_SOUL=ON
```

## Usage

### Game Controller Example

ROSCCO comes with a joystick message conversion example to work with. To make
use of this, build the project with example turned on from your `catkin_ws`
directory:

```
catkin_make -DKIA_SOUL=ON -DEXAMPLE=ON
```

This example has been used with a Logitech F310 and an Xbox 360 controller
before, make sure that your controller maps accordingly. If it is not mapped
correctly you'll want to make the required changes to the launch file and
possibly the example code. More information on the joystick node can be found
at: http://wiki.ros.org/joy

To use the example you'll need to bring up a CAN connection for OSCC to
communicate, source the newly compiled package and launch the three nodes,
joy_node, roscco_teleop, and roscco_node.

`joy_node` uses `/dev/js0` by default to change this see the
[joystick documentation](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick).

```
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
source devel/setup.bash
roslaunch src/roscco/example/example.launch
```

The roscco_teleop converts the joy messages from the joy_node into messages
for roscco_node. The converted messages are sent on a 50ms cadence to ensure
OSCC receives a message within the required OSCC API timing for detection of
lost connection. The roscco_node sends it's received messages to OSCC API.

## Running ROSCCO

The default CAN channel is `can0` but can be specified as a node parameter and
assumes there is a can device available through linux can. The best way to
validate that is through `ifconfig | grep can`

```
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
roscore &
rosrun roscco roscco_node _can_channel=0
```

Passing a message through ROS should yield results on your OSCC device such as
enabling control and holding the brake position:
```
rostopic pub /EnableDisable -1 \
'{header: {stamp: now}, enable_control: true}'

rostopic pub /BrakeCommand roscco/BrakeCommand -r 10 \
'{header: {stamp: now}, brake_position: 50}'
```

Similarly to the OSCC API, the Reports and CanFrame messages can be
subscribed to in order to receive feedback from OSCC. The Command and
EnableDisable messages will call the commands for the OSCC API.

OSCC modules will disable if no messages are received within 200 ms. It is
recommended to publish ROSCCO messages, at a rate of 20 Hz (every 50 ms) to
ensure OSCC modules do not disable.

## Tests

## Test Dependencies

ROSCCO requires GoogleTest to be [installed](https://github.com/google/googletest/blob/master/googletest/README.md).

ROSCCO also depends on RapidCheck which is included as a submodule, ensure that
the repository has been cloned recursively to include all submodules.

```
git submodule update --init --recursive
```

## Testing ROSCCO build

If you have already cloned the repository and have build ROSCCO you should be
able to compile and run the tests as well after you `cd` into your `catkin_ws`
directory:

```
catkin_make run_tests -DKIA_SOUL=ON
```

If you have not cloned the repository yet:

```
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone --recursive https://github.com/PolySync/roscco.git
cd ..
source /opt/ros/kinetic/setup.bash
catkin_make -DKIA_SOUL=ON
catkin_make run_tests -DKIA_SOUL=ON
```

# License

ROSCCO is licensed under the MIT License (MIT) unless otherwise noted
(e.g. 3rd party dependencies, etc.).

Please see [LICENSE.md](LICENSE.md) for more details.
