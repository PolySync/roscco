# roscco

## Building roscco in a catkin project

```
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone --recursive https://github.com/PolySync/roscco.git
cd ..
catkin_make -DKIA_SOUL=ON
```

## Running roscco

roscco defaults to can channel but can be specified as a node parameter
```
rosrun roscco roscco_node _can_channel=0
```

## Testing roscco

```
catkin_make run_tests -DKIA_SOUL=ON
```
