# ROS navi_goals package

## Abstract
ROS node that reads in a series of navigation waypoints from a YAML file and publishes them to move_base to drive a robot through them one by one.

## Repository architecture 
### Source code files :
+ **waypoints.cpp** : load waypoints from YAML file and publish them one at a time
 
### Directories :
+ **src/** : (required) contains source code, **waypoints.cpp**
+ **config/** : (required) contains YAML file that stores the position and orientation of waypoints
+ **launch/** : (optional) contains launch file for the executable file of this package
+ **rviz/** : (optional) contains Rviz configuration file to display navigation information

## YAML file structure

'The package requires a `waypoints.yaml` file that should look like this:

```yml
task:
  name: classroom_c018
  type: waypoint_following
  waypoints:
  - [44.43, 27.78, 0]
  - [51.28, 25.02, 0]
  - [43.18, 13.45, 0]
  - [37.28, 17.58, 0]
```

As showed above the file `waypoints.yaml` has the following structure:

- `task` (required): root object that identifies the YAML file as containing waypoints data
- `waypoints` (required): block key that groups the definition of each waypoint position and orientation in the map  
- the [x, y, z] is interpreted as `x`, `y` and `z` (required): key-value pairs that contain the coordinates (x,y) and orientation (th), in degrees, of each waypoint

## Direct usage:

- Clone this repository into a ROS catkin workspace
- Build and source the workspace
- To execute this package: 

```bash
$ roslaunch navi_goals navi_goals.launch task_name:=classroom_c018_task
```
here task name is the yaml file in floor plan code.

Inorder to use local file change the launch file. 

  
## Acknowledgement

The package was modified from https://github.com/rfzeg/navi_goals

The development of this package started based on the ROS Tutorial [Sendig Goals to the Navigation Stack](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals) and its sample code.
Since then it has been modified and extended significantly.
