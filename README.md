# ROSCCO

## Overview

ROSCCO is an interface between ROS and OSCC allowing ROS users to interface with
OSCC from their own ROS nodes utilizing ROS messages created from the OSCC API.

## Getting Started

### Dependencies

ROSCCO is built on the Linux implementation of ROS Kinetic. The only known
limitation for using older versions is that the tests require C++11.

To install ROS Kinetic see the
[install instructions](http://wiki.ros.org/kinetic/Installation) on your Linux
distribution.

### Building ROSCCO in a catkin project

ROSCCO uses the OSCC API which builds for specific vehicle models. OSCC is a
git submodule so that it can be built during the catkin build process. Make sure
the version of ROSCCO you're using matches with the version of OSCC firmware you
are using. See the [release page](https://github.com/PolySync/roscco/releases)
for information about what releases ROSCCO points to.

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

To install joy node
```
sudo apt-get install ros-kinetic-joy
```

`joy_node` uses `/dev/js0` by default to change this see the
[joystick documentation](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick).

```
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
source devel/setup.bash
roslaunch src/roscco/example/example.launch
```

The roscco_teleop converts the joy messages from the joy_node into messages
for roscco_node and the roscco_node sends it's received messages to OSCC API.

### Apollo open autonomous driving platform

ROSCCO can serve as a bridge to Baidu's open autonomous driving platform Apollo.

This example only support the Kia Soul EV. We are planning on extending support to the Kia Niro.
You will need a double channel Kvaser or two single channel Kvaser, to connect to drivekit CAN and diagnostics CAN.

#### Installation and build

Install and build Apollo

This project was developed on Apollo v2.0.0. It would most probably build on other version of Apollo with some modifications.

```
git clone https://github.com/ApolloAuto/apollo.git --branch v2.0.0
cd apollo/
./docker/scripts/dev_start.sh
./docker/scripts/dev_into.sh
./apollo.sh build
```


Install and build ROSCCO

The installation instructions are very similar to the one above, the main difference is that everything needs to be done within Apollo's docker environment.

```
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone --recursive https://github.com/PolySync/roscco.git
cd ..
catkin_make -DCATKIN_ENABLE_TESTING=0 -DKIA_SOUL_EV=ON -DAPOLLO=ON
source devel/setup.bash
```

#### Running ROSCCO with Apollo

With Apollo control module running, 

```
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
sudo ip link set can1 type can bitrate 500000
sudo ip link set up can1
roslaunch roscco apollo.launch
```

You can enable OSCC using `rostopic pub`,

```
rostopic pub /enable_disable roscco/EnableDisable "header:
  seq: 0
  stamp: now
  frame_id: ''
enable_control: true"
```


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
enabling control and turning the steering right with 20% torque:
```
rostopic pub /enable_disable roscco/EnableDisable -1 \
'{header: {stamp: now}, enable_control: true}'

rostopic pub /steering_command roscco/SteeringCommand -1 \
'{header: {stamp: now}, steering_torque: 0.2}'
```

Similarly to the OSCC API, the Reports and CanFrame messages can be
subscribed to in order to receive feedback from OSCC. The Command and
EnableDisable messages will call the commands for the OSCC API.

OSCC modules will disable if no messages are received within 200 ms. It is
recommended to publish ROSCCO messages, at a rate of 20 Hz (every 50 ms) to
ensure OSCC modules do not disable.

## Tests

## Test Dependencies

ROSCCO requires GoogleTest to be
[installed](https://github.com/google/googletest/blob/master/googletest/README.md).

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
