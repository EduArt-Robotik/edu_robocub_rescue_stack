# edu_robotcub_stack 4maps

It is not recommended to continue working on this implementation. It is recommended to continue working on the [implementation that uses the Navigation Stack 2](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack).

## launch
ros2 launch edu_robocup_rescue_stack edu_control.launch.py 

## Team

Katarina, Jakob, Daniel

## Control

The control was tested with the Gazebo robot model [eduard_offroad](https://github.com/EduArt-Robotik/edu_simulation/tree/feature/sand_gravel_ramp/model/eduard_offroad) for the [TER0_ramp](https://github.com/EduArt-Robotik/edu_simulation/tree/feature/sand_gravel_ramp/model/TER0_ramp) track and the robot control is implemented accordingly. To control the Gazebo robot model eduard_offroad, a geometry_msgs::msg::Twist must be pushed under /cmd_vel. In the linear.x value the forward speed and in angular.z the rotation speed around the yaw angle of the robot is passed. The algorithm that is implemented is a point following algorithm. The closer the robot gets to the point, the slower the robot moves. The robot must be within a certain radius of the point to move to the next point. When the robot changes the ramp, the robot should not move to the target point, since this is replaced as soon as the robot has fallen down the ramp. To ensure that the robot changes ramps at the correct angle.


## Orientation and localization

To steer the robot through the parqour the current position is needed at any time. To obtain the position, Adaptive Monte Carlo Localization (AMCL) was chosen. The algorithm used for the localization is provided by the open-source library [Nav2](https://navigation.ros.org/). It requires a 2D laser scanner and a previously created map. The laser scanner was added to the [Gazebo robot model](https://github.com/EduArt-Robotik/edu_simulation/tree/feature/sand_gravel_ramp/model/eduard_offroad). [source: Github nav2_amcl](https://github.com/ros-planning/navigation2/tree/main/nav2_amcl)

### Multiple maps

The parkour is three-dimensional. In addition to flat surfaces, it also includes ramps. However, position recognition with AMCL has only been developed for a two-dimensional flat surface. Therefore, when developing a robot control algorithm with only one map, position detection problems have been encountered. For example, the robot recognized the ramp as a wall when standing on the level surface and the level as a wall when standing on the ramp. To remedy this, a map was implemented for both ramps and both flat surfaces.
In addition, the AMCL algorithm used could not distinguish which part of the track it was on due to the symmetry of the track. 

### Generate Map 

To generate the maps, the [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
from SteveMacenski was used. The generation was started with the [online_async_launch.py](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py). A [copy](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py) of the source code is in this repository.

### Switching maps

To change the map, two implementation options were compared. A new map can either be loaded in a map server and a completely new map server can be started for each map. Both options worked.

To provide a map for the Nav2 system, a map server is used. The map server node provides the map at startup. With load_map the map can also be exchanged while the map server node is already running. Instead of swapping the map, the whole map server can be "swapped" by activating and deactivating the map servers as mentioned above. This is possible because a map server is a 'LifecycleNode'. [source: Map_server.cpp line 65](https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/src/map_server/map_server.cpp)

A LifecycleNode has four primary states: Unconfigured, Inactivate, Active, Finalized. Initially, each LifecyleNode is in the Unconfigured state, which can be transformed to the Inactive state. From the state, the node can become Active by puplishing the map and making it available to other nodes, such as the AMCL node.
To change the maps, four map servers are launched at startup (with amcl_4maps.launch.py), one for each map. [source](https://design.ros2.org/articles/node_lifecycle.html)

#### Terminal
For testing purposes first a map change was executed by commands in the terminal. To load a new map `ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}"` must be executed. [source](https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/README.md) 

To swap the map by activating and deactivating the map servers, it is necessary to query the current state of the nodes and to be able to trigger the state change. To get the current lifecycle state for the example node named map_server ros2 service call `/map_server/get_state lifecycle_msgs/GetState` can be executed. To change the state from Unconfigured to Inactive `ros2 lifecycle set /map_server configure` or `ros2 service call /lc_talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 2}}"` can be executed. [source](https://index.ros.org/p/lifecycle/)

#### Implementation of the map change

##### Implementation where the current map is loaded
The relevant functions are within the class 'loadMap'.

To change a map by loading a new map, a client is created. With this client an asyncRequest with the desired map can be sent to the map server. Beside the desired ramp a callback function is passed, by which a successful change can be determined.

##### Implementation in which the current map server is activated 
A new class 'clientService' was created to implement the map switch using lifecycle management. There the functions activeServices and deactiveService are provided, with which the Map_server can be activated or deactivated. The implementation was based on the ['service_client' of thehummingbird](https://github.com/thehummingbird/robotics_demos/blob/main/lifecycle_node/src/demo_lifecycle/src/service_client.cpp). 

To get the current state and to trigger a state change an asyncRequest is sent. In contrast to the implementation of thehummingbird a callback function was passed. The callback function provieds feedback over the success/failure of an asyncRequest.

###### Problems with the implementation of the 4 map servers
During the implementation of the 4 map servers it came to the problem that during the startup often not all map server nodes and the AMCL Node were started completely. To solve this problem, the thread waited for 2 seconds at the beginning (in the constructor of the 'LocalisationControlNode' ). If not waited it can lead to errors because the map server can be accessed although the node is not ready.

#### Problems :
The navigation steps are very error-prone. It happened that the algorithmen jumped to the next navigation step too early or not at all. Additionally, there were problems with the localization, as already described for the alogrithm that used Nav2. (see [here](README.md#Incorrect-localization) ). If the robot is in a different place, which was not considered during the planning of the respective navigation step, the further travel of the robot is not possible and it leads ob to accidents.
In addition, the physical properties of the robot model are not yet fully developed, which is why the robot may have tipped into an unstable position when crossing the ramp and, for example, hopped or tipped completely onto its side.
