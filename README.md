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


## YAML file structure

'The package requires a `waypoints.yaml` file that should look like this:

```yml
waypoints:
  - point:
      x: 3.0
      y: 2.0
      th: 0 # in degree
  - point:
      x: -3.0
      y: 2.5
      th: 90 # in degree 
```

As showed above the file `waypoints.yaml` has the following structure:

- `waypoints` (required): root object that identifies the YAML file as containing waypoints data
- `point` (required): block key that groups the definition of each waypoint position and orientation in the map  
- `x`, `y` and `th` (required): key-value pairs that contain the coordinates (x,y) and orientation (th), in degrees, of each waypoint

It is possible to add comments to the YAML file by using the # sign. All `point` elements begin with a dash (-) and must be prefixed with the same amount of spaces, in the example above two spaces are used, the number of spaces can vary from file to file, but tabs are not allowed. In like manner the `x`, `y` and `th` keys also require the same amount of spaces in front of each (at least as many spaces as each `point` key has).  
  
The YAML file can be adjusted by adding or deleting waypoints, changing they order and adding/removing comments. It is important to do not break the formatting rules described above while editing the file or the waypoints will not load properly.

## Direct usage:

- Clone this repository into a ROS catkin workspace
- Build and source the workspace
- To execute this package: `roslaunch navi_goals navi_goals.launch`


## Usage example in combination with a waypoint generator node using Gazebo as simulation:

+ Clone following repositories to a catkin workspace (for example ~/catkin_ws/src):

  A Gazebo simulated environment (world):  
  `git clone https://github.com/rfzeg/service_bot.git`  
  A URDF robot model for simulation:  
  `git clone https://github.com/rfzeg/dumpster.git`  
  This package to give a robot a predefined set of waypoints:  
  `git clone https://github.com/rfzeg/navi_goals.git`  

+ If not already present, install xterm (required to run the bash script):

  `sudo apt-get install xterm`
  
+ Build and source your workspace :

  `cd ~/catkin_ws`  
  `catkin_make`  
  `source devel/setup.bash`
    
+ Run the project with the following bash script provided inside the service_bot package:

  `./run_waypoints.sh`

  Or run each node one by one manually in separated terminal instances.
  
## Acknowledgement

The development of this package started based on the ROS Tutorial [Sendig Goals to the Navigation Stack](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals) and its sample code.
Since then it has been modified and extended significantly.
