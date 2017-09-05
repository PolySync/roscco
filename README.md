ROSCCO is an interface between ROS and OSCC allowing ROS users to interface with
OSCC from their own ROS nodes utilizing ROS messages created from the OSCC API.


## Building ROSCCO in a catkin project

ROSCCO uses the OSCC API which builds for specific vehicle models. OSCC is a
git submodule so that it can be built in the catkin process at the same time as
the ROSCCO node.

ROSCCO is built on ROS Kinetic. The only known limitation for using older
versions is that the tests require C++11.

```
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone --recursive https://github.com/PolySync/roscco.git
cd ..
catkin_make -DKIA_SOUL=ON
```

## Running roscco

The default can channel is can0 but can be specified as a node parameter and
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

Similarly to the OSCC API The Reports and CanFrame messages can be
subscribed to as feedback about what is going on in OSCC. The Command and
EnableDisable messages will call the commands for the OSCC API.

## Testing roscco build

```
catkin_make run_tests -DKIA_SOUL=ON
```
