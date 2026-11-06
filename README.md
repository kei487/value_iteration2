# value_iteration2: real-time value iteration planner package for ROS 2

a monolithic mobile robot navigation system as a replacement for the navigation stack

* ROS 1 version: https://github.com/ryuichiueda/value_iteration
* with A\* algorithm version: https://github.com/kei487/value_iteration2/tree/feature/withastar2d

## How to try

```bash
$ sudo apt install ros-humble-grid-map*
$ sudo apt install ... (I will write later about required packages. )
$ cd <workspace>
$ git clone https://github.com/ryuichiueda/emcl2_ros2.git #Please use this version due to a problem of launch file.
$ git clone https://github.com/ryuichiueda/value_iteration2.git
$ cd <workspace>
$ colcon build
$ source install/setup.bash
$ source install/local_setup.bash
$ ros2 launch value_iteration2 turtle.launch.py
```

### how to use with RViz and Gazebo

[![](https://img.youtube.com/vi/qNjMH5Ao6QM/0.jpg)](https://www.youtube.com/watch?v=qNjMH5Ao6QM)

## Nodes

### vi_node

This node executes value iteration.

#### how to execute value iteration

Give the goal to topic `/goal_pose`. The planner immediately starts working and giving the velocity of the robot through `/cmd_vel`. The action server is not implemented in this stage.

#### Services Called

* static_map ([nav_msgs/srv/GetMap](https://docs.ros2.org/foxy/api/nav_msgs/srv/GetMap.html))
    * Initiate the map for value iteration.

#### Subscribed Topics

* goal_pose ([geometry_msgs/msg/PoseStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html))
    * goal position and orientation
* scan ([sensor_msgs/msg/LaserScan](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html))
    * laser scans
* tf ([tf2_msgs/msg/tfMessage](https://docs.ros2.org/foxy/api/tf2_msgs/msg/TFMessage.html))
    * transforms

#### Published Topics

* /cmd_vel ([geometry_msgs/msg/Twist](https://docs.ros2.org/galactic/api/geometry_msgs/msg/Twist.html))
    * the control order to the robot; published only when the parameter `online` is true

#### Parameters

* action_list (still not imported from ROS 1)
    * Currently, `vi_node` requires an action list like the following. `onestep_forward_m` and `onestep_rotation_deg` mean the forward and rotational velocities respectively. `vi_node` calculates the optimal value function and the oplimal policy based on this action list.

```
  <rosparam> <!-- in the case of ROS 1 -->
    vi_node:
      action_list:
        - name: forward
          onestep_forward_m: 0.3
          onestep_rotation_deg: 0.0
        - name: back
          onestep_forward_m: -0.2
          onestep_rotation_deg: 0.0
        - name: right
          onestep_forward_m: 0.0
          onestep_rotation_deg: -20.0
        - name: left
          onestep_forward_m: 0.0
          onestep_rotation_deg: 20.0
  </rosparam>
```

* ~online (bool, defalut: true)
    * flag for using vi_node as a real-time planner
* ~theta_cell_num (int, default: 60) 
    * number of intervals of the discrete state space on theta-axis
* ~global_thread_num (int, default: 2) 
    * number of threads used on value iteration
* ~goal_margin_radius (double, default: 0.2[m]) 
    * radius of the goal on xy-plane
* ~goal_margin_theta (int, default: 10[deg]) 
    * radius of the goal on theta-axis
* map_type (string, "cost" or "occupancy", default: occupancy) 
    * choice of map for setting immediate costs and occupancy (please read the Maps section)
* map_service (string, default: `/map_server/map`)
    * GetMap service used in occupancy mode
* cost_map_service (string, default: `/cost_map_server/map`)
    * GetMap service used in cost mode
* local_cost_source (string, "scan" or "costmap", default: scan)
    * source of the online local penalty. `scan` uses `/scan`. `costmap` uses a Nav2 local costmap.
* local_costmap_topic (string, default: `/local_costmap/costmap`)
    * OccupancyGrid topic used when `local_cost_source` is `costmap`
* local_cost_scale (double, default: 1.0[s])
    * seconds added to `local_penalty_` per costmap cell value in `1`–`99`. Value `100` or higher is lethal and uses the same large penalty as a laser hit (`2048`[s]). Unknown (`-1`) is ignored. Costs are sampled over each value-iteration cell at the costmap resolution and the maximum is kept. The local window stays 1[m].

#### Maps

We can choose two types of maps for initializing parametes of states. 

##### occupancy mode: 

A cost map is created from an occupancy grid map. Occupied cells are marked as roped-off area. Moreover, cells near occupied ones are given immediate penalty. The near cells and the value of the penalty are controlled through the following parametes. 

* parapeters
    * safety_radius (double, default: 0.2[m]) 
        * distance that should be kept between the center of the robot and an occupancy grid 
    * safety_radius_penalty (double, default: 30[s], max: 1,000,000,000[s]) 
        * immediate penality (negative immediate reward in the field of reinforcement learning) when the robot invades the safety radius. 

##### cost mode:

We can also use a "cost map," which contains the immediate cost of every cell. This map should be written with "[Raw mode](https://docs.nav2.org/configuration/packages/configuring-map-server.html)". 

Cell value semantics in cost mode:

* `0`–`100`: free cells. The value is used as an immediate cost [s] in addition to the per-step base cost (1[s]).
* otherwise (`101`–`255`, or unknown): prohibited cells.

Free cells within `safety_radius` of a prohibited cell additionally receive `safety_radius_penalty`.

There are an information file (`cost.yaml`) and a map file (`cost.pgm`) in the `maps` directory. The information file must contain the line `mode: raw` as shown below.

```
image: ./cost.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65 #not used
free_thresh: 0.196    #not used
mode: raw
```

`vi_node` reads a cost map from the service specified by `cost_map_service` (default: `/cost_map_server/map`). Launch with cost mode as follows.

```bash
$ ros2 launch value_iteration2 turtle.launch.py map_type:=cost
```

This starts a dedicated `nav2_map_server` named `cost_map_server` that loads `maps/cost.yaml`, remaps its `map` topic to `cost_map_for_vi` (so it does not overwrite the occupancy map used by localization/RViz), and activates it through a lifecycle manager.

## Notes

### What's the cost means?

It is the sum of costs from a pose (position + orientation) to a point of goal. The cost means the time. Therefore, it means the time required to reach the goal if immediate penarties don't exist. As shown in the parameter list, we can define some kinds of immediate penalties, which are quantified in units of seconds. 

## publication

* R. Ueda, L. Tonouchi, T. Ikebe, and Y. Hayashibara, [“Implementation of Brute-Force Value Iteration for Mobile Robot Path Planning and Obstacle Bypassing,”](https://www.fujipress.jp/jrm/rb/robot003500061489/) J. Robot. Mechatron., Vol.35 No.6, pp. 1489-1502, 2023.

## acknowledgement

* This software is developped on the support of JSPS KAKENHI JP20K04382.
* A custom simulation environment in this repository is derived from [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations), which is licensed with Apache License 2.0. 
