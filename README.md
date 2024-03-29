# edu_robotcub_rescue_stack

## setup:

### Prerequisites:

- For simulation, the [edu_simulation-repository](https://github.com/EduArt-Robotik/edu_simulation) needs to be cloned to your local computer.

- In order to use the [Navigation Stack 2 (Nav2)](https://navigation.ros.org/) function library, the [navigation2-repostiory](https://github.com/ros-planning/navigation2) needs to be cloned to your local computer. 

- The [lattice_primitves](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/tree/main/lattice_primitives) which are generated specifically for the eduard offroad robot and are located in the "lattice_primitives" folder in this repository, must be placed in the cloned navigation2 repository under the path `/navigation2/nav2_smac_planner/lattice_primitves`. Then the absolute path to the lattice_primitves-folder is to add inside the nav2_edu_params.yaml-file to the parameter "lattice_filepath" (line 502) at the planner_server. 

- The absolute path to this repository has to be set in the "repository_path"-variable (line 35) inside the constructor of navigation.cpp. 

### Launch:

1. Launch Gazebo:  
    `ros2 launch edu_simulation eduard.launch.py`

2. Select parqour „TER0_ramp“ in the „Insert“ tab and place it in the free space.

3. Click on the parqour and set the following pose in left bar:  
    x: 3,14  
    y: 0,6  
    z: 0,00  
    roll: 0,00  
    pitch: 0,00  
    yaw: 0,00  

    → The target positions of this algorithm are designed to have the parqour located at that position in the global coordinate system.

4. Choose „Eduard Offroad“ in the „Insert“ tab and place it close to the coordinate origin (blue Z-axis).

5. Launch navigation and control algorithm.
    `ros2 launch edu_robocup_rescue_stack navigation.launch.py`

## Introduction / Task

The goal is to design a concept for autonomous driving of the Eduard offroad robot through a simulated parqour. 
The robot is designed to traverse a predefined parqour  ([(TER 1)Sand/Grave](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf)) as many times and as fast as possible. When it reaches the end of the (TER1 )Sand/Grave parqour, the robot shall backtrack (without making a 180° turn). The robot must first cross a flat element, then drive up a ramp, get onto another ramp, drive down this ramp and finally drive on a flat element to its end. A particular difficulty is crossing from one ramp to the other ramp.

## Orientation and localization

To steer the robot through the parqour the current position is needed at any time. To obtain the position, Adaptive Monte Carlo Localization (AMCL) was chosen. The algorithm used for the localization is provided by [Nav2](https://navigation.ros.org/). It requires a 2D laser scanner and a previously created map. The laser scanner was added to the [Gazebo robot model](https://github.com/EduArt-Robotik/edu_simulation/tree/feature/sand_gravel_ramp/model/eduard_offroad). [source: Github nav2_amcl](https://github.com/ros-planning/navigation2/tree/main/nav2_amcl)

### AMCL - Adaptive Monte Carlo Localisation

The AMCL is an algorithm, that uses a particle filter to estimate the position and orientation of a robot in a known map, based on information from a 2D laser scanner and odometry. Within the AMCL the particle filter must be initialized with the initial position of the robot, otherwise it starts at the origin of the reference coordinate system (Here: map). The particles that should represent the pose of the robot are randomly distributed in space at the beginning. If the robot moves (motion-update), the algorithm uses the initial position and the odometry information to make a prediction of the particle positions. The laser scanner then takes a measurement that is compared to the predicted particle positions. Then the algorithm weights the particles according to the match with the measurement. Particles with higher weights have a higher probability of representing the actual pose. A process called resampling rejects particles with low weighting on it. This again adjusts the particle set more to the more likely outcome and thus improves the position estimate. The algorithm repeats these steps several times. The final estimate of the pose is calculated based on the weights of the particles, with higher weighted particles having a greater impact on the result. In addition to the pose, AMCL also publishes the estimated odometry. [source_1](https://roboticsknowledgebase.com/wiki/state-estimation/adaptive-monte-carlo-localization/), [source_2](https://tams.informatik.uni-hamburg.de/lectures/2020ss/seminar/ir/doc/jtb_7328242_final.pdf)

### Multiple maps

The parkour is three-dimensional. In addition to flat surfaces, it also includes ramps. However, position recognition with AMCL has only been developed for a two-dimensional flat surface. Therefore, when developing a robot control algorithm with only one map, position detection problems have been encountered. For example, the robot recognized the ramp as a wall when standing on the level surface and the level as a wall when standing on the ramp. To remedy this, a map was implemented for both ramps and both flat surfaces.
In addition, the AMCL algorithm used could not distinguish which part of the track it was on due to the symmetry of the track. 

### Generate Map 

To generate the maps, the [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
from SteveMacenski was used. The generation was started with the [online_async_launch.py](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py). A [copy](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py) of the source code is in this repository.

### Switching maps

To change the map, two implementation options were compared. A new map can either be loaded in a map server or a completely new map server can be activated for each map. Both options worked. A new map is loaded in this repository to change the maps.

To provide a map for the Nav2 system, a map server is used. The map server node provides the map at startup. With load_map the map can also be exchanged while the map server node is already running. Instead of swapping the map, the whole map server can be swapped by activating and deactivating the map servers as mentioned above. This is possible because a map server is a 'LifecycleNode'. [source: Map_server.cpp line 65](https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/src/map_server/map_server.cpp)

A LifecycleNode has four primary states: Unconfigured, Inactivate, Active, Finalized. Initially, each LifecyleNode is in the Unconfigured state, which can be transformed to the Inactive state. From the state, the node can become Active by puplishing the map and making it available to other nodes, such as the AMCL node.
To change the maps, four map servers are launched at startup (with amcl_4maps.launch.py), one for each map. [source](https://design.ros2.org/articles/node_lifecycle.html)

#### Testing form the Linux Terminal
For testing purposes first a map change was executed by commands in the terminal. To load a new map `ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}"` must be executed. [source](https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/README.md) 

To swap the map by activating and deactivating the map servers, it is necessary to query the current state of the nodes and to be able to trigger the state change. To get the current lifecycle state for the example node named map_server ros2 service call `/map_server/get_state lifecycle_msgs/GetState` can be executed. To change the state from Unconfigured to Inactive `ros2 lifecycle set /map_server configure` or `ros2 service call /lc_talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 2}}"` can be executed. [source](https://index.ros.org/p/lifecycle/)

### Implementation of the map change

##### Implementation in which the current map is loaded
To change a map by loading a new map, a client is created. With this client an asyncRequest with the desired map can be sent to the map server. Beside the desired ramp a callback function is passed, by which a successful change can be determined.

##### Implementation in which the current map server is activated for test purposes
A new class was created to implement the map switch using lifecycle management. There the functions activeServices and deactiveService are provided, with which the Map_server can be activated or deactivated. The implementation was based on the ['service_client' of thehummingbird](https://github.com/thehummingbird/robotics_demos/blob/main/lifecycle_node/src/demo_lifecycle/src/service_client.cpp). 

To get the current state and to trigger a state change an asyncRequest is sent. In contrast to the implementation of thehummingbird a callback function was passed. The callback function provieds feedback over the success/failure of an asyncRequest.

###### Problems with the implementation of the 4 map servers
During the implementation of the 4 map servers it often came to the problem that not all map server nodes as well as the AMCL node were started completely during the startup.
To solve this problem, the thread waited for 2 seconds at the beginning (in the constructor of the 'LocalisationControlNode' ). Not forcing the programm to wait can cause errors, because the map server could be accessed while the node is not ready.

#### Comparison 
With both implementations the maps can be exchanged. The implementation by means of MapServer change is clearly more complex than the implementation that loads a new map within one map server. Therefore the implementation in which the map is reloaded was chosen in the end.

## Control

This repository branch contains an algorithm that uses [navigation stack 2 (Nav2)](https://navigation.ros.org/). As part of the search for the concept of traversing the course, two algorithms were developed and tested. One algorithm does not use any third party navigation library for navigation, the other is the algorithm mentioned earlier, which is using the ROS Navigation Stack 2. Both use Nav2 AMCL Algorithmen for localisation and are developed in C++. All tests of the two algorithms were performed in Gazebo with an [Eduard offroad robot](https://github.com/EduArt-Robotik/edu_simulation/tree/feature/sand_gravel_ramp/model/eduard_offroad) on the [TER0_ramp](https://github.com/EduArt-Robotik/edu_simulation/tree/feature/sand_gravel_ramp/model/TER0_ramp) track.

### Algorithm using Nav2
This branch contains the source code of the algorithmen that uses the ROS Navigation Stack 2.

#### Navigation:

##### Process:

As explained earlier, position detection based on two dimensions with AMCL does not provide the possibility of continuous localization over the entire three-dimensional course. Due to this, depending on the current level (straight 1, ramp 1, ramp 2 oder straight 2), a specific map needs to be loaded ([LoadMap](README.md#Implementierung-des-Map-Wechsels)) in which the robot position has to be reinitialized every time.

![Strecke: Ter0_Ramp](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/bev_ter0_ramp.png)

The robot must also be given a new target, which it must approach on the map. A beneficial procedure is to load the layer-specific map at the beginning, then to send the target position and finally to initialize the robot pose.

Intuitively, one would send the target position to the path planner after the robot position has been initialized. For one use case, this is a disadvantage: After the robot has changed the ramp, it wobbles. For an accurate determination of the initialization position, it should not have a pitch and roll angle. Therefore, the program must wait until the robot is straight again after changing the ramp and then determine and publish the initial position. The waiting-duration is set to 1.0 seconds. The algorithm then checks the status of the robot. Immediately assigning the new target prevents the robot from orienting itself back to the previous target (old map) in the meantime and instead allows it to move directly in the right direction (new target).

Based on different angle constellations as well as the current X/Y-position, the currently traveled plane can be identified. The registration of a change of the plane is followed by the loading of a new map, the navigation to a new destination selected depending on the plane as well as on the current direction of movement, and the initialization of the new robot pose. This procedure is identical for each map.

##### Goal-position: 

For the way to the end of the track and back, one given target position for each case (incl. orientation) per map would be sufficient. Due to the tendency of the path planner to calculate a path for the target on the ramps that runs very close to obstacles or along the collision areas of the costmap and therefore sometimes led to problematic situations, two targets are now assigned for the outward and return paths on the ramps. On the straights, one target is sufficient in each case.

##### Initial-position:

The X/Y position is determined by the distances of certain laser beams (straight ahead, straight back, right and left) measured by the laser scanner relative to the walls of the parqour.

Depending on the current position on the track, different walls are used for distance measurement and therefore for maintaining the robot position.
Preferred are walls that show more of a perpendicular angle to the plane the robot is on. This allows minimizing the error generated by the robot due to its own motion around pitch and roll axes.

Straight 1:    Wall right, when moving in forward dircection & wall straight backwards
Ramp 1:    Wall right, when moving in forward direction & wall sraight forwards
Ramp 2:    Wall left, when moving in forward direction & wall straight backwards  
Straight 2:    Wall left, when moving in forward direction & wall straight forwards  

Depending on the origin point of the map (see [map-config-file](map/map_1.yaml)), an offset is added to the measured distance to get the true pose relative to the global coordinate system (map). The determination of the laser beams pointing in the necessary direction (perpendicular to the wall) regardless of the current orientation around the yaw-axis of the robot is done with the help of the Yaw angle of the IMU. The IMU also provides the information of the robot's orientation (X, Y, Z, W) for the initialization pose.

#### ROS Navigation Stack 2:

The actual navigation through the course is based on the ROS [Navigation Stack 2](https://navigation.ros.org/). A path planner subscribes to the published target positions and then plans a path through a costmap to the target. Because of large differences in height between specific areas of the two ramps, the secure driving from one ramp the the other is not everywere possible. For that reason a keepout-filter above the costmap ensures that the transition takes place at a safe place. Once a valid path is found, a "FollowPath" plugin controls the robot movement along the path. A Behaviour Tree controls the regeneration behaviour of the robot in special cases, such as the robot deviating from the path or getting stuck.

##### Path-Planner:

Since time is very important for the runs of the Robocup, the path planning has to be designed in a way that a backward movement of the robot is possible and the time for turning around on the narrow course can be saved. Only two path planners in the Nav2 function library provide this feature. The Smac Hybrid Planner (Hybrid A* Planner) and the Smac Lattice Planner (State Lattice Planner). Since the Smac Hybrid Planner is not suitable for differentially driven robots, the Smac Lattice Planner was chosen.  

The Smac Lattice Planner is built on a grid-based data structure called "Sparse-map". The robot's ability to rotate continuously around the vertical axis ensures that, at certain rotation angles, straight trajectories do land on endpoints that are not aligned with the grid. To solve this problem, an individually parameterizable [„Lattice Primitiv Generator“](https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner/lattice_primitives) generates standard trajectories in advance.  For this purpose, this discretizes the possible angles of the robot's motion directions in such a way that straight trajectories land on endpoints that coincide with the discrete grid points of the grid. The so-called "minimum control set", which contains these trajectories, has to be generated manually for each robot individually with the "Lattice Primitive Generator". [source](https://navigation.ros.org/configuration/packages/smac/configuring-smac-lattice.html)


##### Controller-server

The controller server is a task server which is used to execute navigation tasks consisting of multiple steps and actions. It implements the follow-path plugin, which generates the command velocities for following the trajectory generated by the path planner. The Nav2 function library contains several [path following plugins](https://navigation.ros.org/plugins/index.html). However, most of them only allow target tracking while the robot is moving forward. This results in the robot having to either turn in place to the target or turn around in several moves. Since, as already described in the "Path Planner" section, this time needs to be saved, the work uses the Regulated Pure Pursuit Controller, which allows trajectory tracking in reverse.[source](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)

The tracking algorithm has the ability to regulate speed based on collision, trajectory curvature, or high cost in the costmap. In practice it has been shown that these regulatory functions are contra-productive in this use case and that the robot is more inclined to get stuck on the course as a result. An example from practice is that in order to move from a straight plane to a ramp, the robot must move to a target position that is at the end of the map and thus close to the inflation layer (high cost). If the robot receives regulated speed commands, its speed is reduced so that it is too slow to drive up the ramp in one go and therefore gets stuck briefly. This can cause a drift in the localization, which has a negative effect on the further travel. For this reason, the regulations are switched off. [source](https://navigation.ros.org/configuration/packages/configuring-regulated-pp.html) 

Besides the path following plugin, the controller server implements progress checker and goal checker plugins. For this purpose, the Nav2 default plugins Simple Progress Checker and Simple Goal Checker are in use. The stateful parameter of the Goal Checker can be used to determine whether it saves the progress of the robot over multiple planning runs. Since storage only provides an advantage in dynamic environments and otherwise only consumes computing power, this feature is disabled. [source](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)


##### Costmap

The base of the costmap are two-dimensional raster maps generated by the slam_toolbox. By using various layers and filters, the Costmap is able to integrate the information from the laser scanner into the map. This allows the representation of the real dynamic environment of the robot. Based on the information contained in the costmap, the path planner plans the trajectories. [source](https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d)

Although SmacLatticePlanner is a global path planner, it is possible to include a local costmap in addition to the global costmap, resulting in an improvement in the accuracy of the trajectories.

The global costmap is used for long-term planning through the entire map and gives a rough overview of the entire environment of the robot. Due to the size of the planning area and thus the environment to be analyzed, which the global costmap covers, it has a slow execution and update time. This is why in this case only layers and filters are integrated that refer to static obstacles. An inflation layer, a static layer and a keepout filter.

The inflation-layer is the most important layer of the costmap. It is an "obstacle magnification layer" in the map that is used to inflate obstacles to create a safe distance around them. This safety distance must be set manually by the "inflation-radius"-parameter. The inflation of the obstacles is necessary because the path planner considers the robot simplified as a point model during the trajectory calculation, which in turn leads to the neglect of the actual dimensions. The path planner treats the inflated area of obstacles as a collision area and sets the trajectories to avoid collisions with the robot's wheels. In general, it is recommended to keep the parameters "infaltion-radius" and "cost-scaling-factor" large, as this tends to push the path planner to travel over free areas in the map. It has been shown in practice and especially on this track that if the "inflation-radius" is large, the robot is often within the "inflation-layer" and so inside the collision area after the jump. If the robot stands in this area, the planner cannot plan a valid trajectory to the target position and the robot stops. For this reason, the "inflation radius" is kept small. The radius of the robot is used as a rough orientation. [source](https://navigation.ros.org/configuration/packages/costmap-plugins/inflation.html)

The static-layer contains information about the position and shape of known and static obstacles registered in the course of creating the maps with the slam_toolbox. [source](https://navigation.ros.org/configuration/packages/costmap-plugins/static.html)

The use of a keepout filter is essential on this route when the robot is controlled by a path planner. The reason for this is that when planning a straight trajectory from one ramp to the other, the path planner makes the robot drive over a high landing, which normally causes it to roll over. A keepout filter is a filter that allows the user to manually create zones that appear as collision areas in the costmap, causing the path planner to avoid these zones when creating the trajectory.

###### Keepout-Filter:

![keepout_filter](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/keepout_filter_90.png)

###### Globale-Costmap mit überlagertem Keepout-Filter:

![global_costmap](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/global_costmap.png)

As it can be seen in the keepout filter image above, the design is such that the path planner has only a very small area through which to plan the trajectory. This area is located a little bit behind the intersection of the ramp planes, so that the robot only has to drive down a small ledge and therefore reach the following level effortlessly and safely. Since the keepout filter is fixed, the robot would have to drive back up the ledge as part of the return trip. Since this usually leads to problems, the following maps are shifted in the direction of the positive X-axis. The shift allows the keepout filter to be placed over the map exactly so that the robot will move down the landing on the other ramp when it returns. The origin position of the map in the global coordinate System can be set in the map configuration file. The slam_toolbox creates the map configuration file in the course of creating the map and assigns the origin position of it automatically. This origin position is often incorrect and should be checked when generating maps with the slam_toolbox. Also seen in the picture of the keepout filter above is that it is somewhat bulbous in shape on both sides of the opening. Tests have shown that a slightly transverse crossing is best suited both in terms of stable driving and subsequent onward travel. The bulbous shape of the keepout filter at this point introduces this cross travel. For creating the keepout filter as well as editing the maps, the image editing program [gimp](https://www.gimp.org/) has proven to be good.[source](https://navigation.ros.org/configuration/packages/costmap-plugins/keepout_filter.html)

The local costmap adjusts the rough path planning, which was initially based on the global costmap, according to the situation. It covers only a small area immediately around the robot compared to the global costmap (entire map). This area contains very detailed real-time information about obstacles in the robot's nearby environment, allowing for more precise path planning. Same as in the global costmap an inflation layer and a keepout filter is used. In addition, an obstacle layer is added specifically for obtaining information about dynamic events.

###### Lokale-Costmap:

![local_costmap](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/local_costmap.png)

The obstacle layer obtains its information about the robot's environment via the laser scanner. It detects and stores the shape and position of obstacles in real time and continuously updates the associated costmap, allowing the path planner to adjust the trajectory according to the situation. Although the obstacle layer is more suitable for dynamic environments and the parqour in which the robot moves is purely static, the use of the obstacle-layer offers a significant advantage. At the moment of jumping from one ramp to the other, the robot partially tilts so much that the laser scanner detects the ground as an immediate obstacle and transmits this to the costmap through the obstacle layer. This ensures that the robot first brakes and stops for a moment. On the one hand, braking ensures a reduced slip in contrast to a strongly oscillating to bouncing further travel and, on the other hand, makes it possible for the robot to stand there relatively still during the determination of the initialization pose. [source](https://navigation.ros.org/configuration/packages/costmap-plugins/obstacle.html)

#### Problems and Errors:

The following is a list and description of problems and errors that occurred at irregular intervals during the test runs.

##### TF_NAN_INPUT-Error

Original-error:  
Error: Ignoring transform for child_frame_id “odom” from authority “Authority undetectable” because of nan value in the transform (nan nan nan) (0.000000 0.000000 -0.770181 0.637825)

During the test runs on the obstacle course in Gazebo, the error occurred more often when two or more wheels of the robot were not in contact with the road, for example when "jumping" from one ramp to the other or when braking too hard on the ramp, which lifted the rear wheels.

In general, a TF_NAN_INPUT error occurs if a nan (not a number) value, i.e. an invalid or non-computable value, appears within a transformation. Transformations are used to transfer positions and orientations from one coordinate system to another. Dynamic transformations that need to be performed in this work while the robot is moving are the transformation from odom_frame to base_frame generated by the robot model (.sdf-file) itself and the transformation from odom_frame to map (-frame) published by the AMCL.

Since the TF_NAN_INPUT error is related to odometry, a first possible cause of the error would be that it does not receive valid or reliable sensor data from the wheel speed sensors during the loss of ground contact, which causes the NAN values in the transformation to occur.

Parallel to the TF_NAN_INPUT error the following error occurs:

Original-error:  
[WARN] [amcl]: AMCL covariance or pose is NaN, likely due to an invalid configuration or faulty sensor measurements! Pose is not availabl!

This error could also result from the faulty odometry, since the AMCL's position detection is based on the odometry information in addition to the data from the 2D laser scanner.

##### Incorrect localization

The final estimation of the pose and thus the localization of the robot is performed by the AMCL. Since the AMCL's position estimation is based on the detection of features in the 2D laser scanner's sensor data, it could be that the symmetrical path, especially on the straights 1 and 2, which have the walls and corners as the only features, causes difficulties for the AMCL. However, the finding that the localization problems are more evident on ramps, which have much more specific characteristics, suggests another main cause. Nevertheless, a contribution to the deterioration of localization cannot be ruled out.

Due to the large amount of slip that occurs at the robot's wheels at many points along the track, it can be assumed that the odometry, on which AMCL's position estimation is based to a significant extent, is a major cause of faulty localization. A particularly large amount of slippage occurs in the following situations:

- Turning in place on the ramp in the direction of the target position often causes the robot to slide down the ramp a bit due to the simultaneous rotation of the wheels in opposite directions.

- During heavy braking, the robot sometimes swings up sharply due to the soft suspension and makes several movements across the roadway. This behavior can be seen mainly during downhill travel on the ramps.

- Driving the landing from one ramp to the other occasionally results in loss of ground contact of individual or all wheels. 

→ Odometry can register the movement in the described situations only poorly or not at all.

To improve the localization, the algorithm publishes an initialization position after each loading of a new map. The determination of this position is based on the distance to the walls in front of or behind and next to the robot measured by the laser scanner. This distance measurement is precise as long as the robot has no roll and/or pitch angle. However, if this is the case, it distorts the distance measurement slightly and a systematic error occurs. At this point, the algorithm can calculate this error for a very small yaw angle or for a yaw angle close to pi.


##### Stuck in inflation-layer or keepout-filter

Both the inflation-layer and the keepout-filter are represented as collision regions in the costmap. Since the "jump" from one ramp to the other is very uncontrolled, it has happened that the robot lands within these collision areas. If this is the case, the path planner cannot calculate a path to the target position and the robot stops at that position. One approach to solving this problem would be to adjust the recovery mode of the robot, which is defined in the Nav2 behavour-tree.


### Algorithm that does not use third-party navigation library

In addition to the algorithm described above, another algorithm has been developed which (apart from using Nav2 functions for localisation) does not use any functions of the Nav2 libraries to navigate the roboter. The source code for this algorithm is available in branch [4maps](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/tree/4maps). In this algorithm there are 4 partial maps as well as 14 navigation-steps (the reason for using 4 maps can be read [here](README.md#problems-amcl)). The robot moves through the defined points one after the other. These points are located on the route model for the TER1 route and are permanently implemented, which is why the algorithm can primarily only be used for this route. The points are located on the ramp as follows:

![Track Points](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/4MapsDestPos.jpg?raw=true "Track and used Points")
The robot travels to the points one after the other. Note that point 2 is offset from point 7 and point 3 is offset from point 8 to ensure that the robot does not hit a wall on the way from (point 2 to 3) or on the way back (point 7 to 8). The roboter crosses the entanglement a little further up the ramp and then drops slightly down.

![Rampe Maps](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/4MapsMaps.jpg?raw=true "Rampe mit Maps")

#### Maps:
1st map: green  
2nd map: red  
3rd map: blue  
4th map: yellow  

#### Navigationsteps:
0. step: drive to 1st point, the driving direction is forward and 1st map is loaded.  
1st step: drive to 2nd point, observe pitch angle and change state when the robot has driven onto first ramp.  
2nd step: load 2nd map and continue driving to 2nd point.  
3rd step: drive to 3rd point, observe pitch and roll angle and change state when robot has changed ramp.  
4th step: load 3rd map and drive to 4th point, observe roll and pitch angle and change state when robot moved from ramp to straight part.  
5th step: load 4th map and continue to 4th point.  
6th step: drive to the 5th point.  
7th step: change driving direction backwards and drive to the 6th point.  
8th step: drive to 7th point, observe pitch angle and change state when robot goes on the ramp.  
9th step: load 3th map and continue to drive towards 7th point.  
10th step: drive to 2th point, watch pitch and roll angle to change state when robot changed ramps.  
Step 11: Load 2nd map and drive towards 1st point, observe roll and pitch angle to change state when robot moves from the ramp to the straight part.  
Step 12: load 1st map and continue driving towards 1st point.  
Step 13: move to the starting point.  

The algorithm assumes that the robot was set to a fixed starting point (start of track) at the beginning. The algorithm essentially consists of two states: turning (on the spot) and driving (straight forward). In addition, it continuously checks (regardless of the state) whether the robot has turned in the direction of the current target point. If this is not the case, the robot stops and starts turning.

#### Rotation:
During a rotation, the robot turns until it can just move towards this next point. The rotation stops when it is within a tolerance range of the angle initially calculated for the rotation. A tolerance range was defined, which on the one hand is as precise as possible, but not too fine, since the angle difference is only calculated in a fixed cycle and the robot should therefore turn as little as possible beyond the target. This tolerance was determined by tests in the simulation. During the rotation, the robot is turned on the spot by the algorithm. As a result, the turning circle remains very small and the robot does not run the risk of unintentionally hitting a wall.

#### Driving:
After the robot has turned in the correct direction, it starts to move forward.  The closer the robot gets to the current target point, the slower it moves. As soon as the robot is within a certain tolerance radius around the current target point, the target point is updated and the robot starts moving towards the next target point. 

#### Special case when crossing the ramp entanglement:
The robot needs the odometry data from the localization libraries for this algorithm. On the way from point 2 to point 3, the robot drives until it has crossed the edge and tilts downward. The algorithm notices this because then the pitch angle is greater than or equal to 0.1 and the roll angle is greater than 0.
This allows the robot to change ramps at an optimal angle. 

#### Implementation details:
The algorithm has been tested so far with the Gazebo Robot Model eduard_offroad and the implementation is specific to it. To control the Gazebo robot model eduard_offroad, a 'geometry_msgs::msg::Twist' must be sent under '/cmd_vel'. In the 'linear.x' value the forward speed and in the 'angular.z' value the rotation speed around the yaw angle of the robot is passed.

#### Problems :

The navigation steps are very error-prone. It happened that the algorithmen jumped to the next navigation step too early or not at all. Additionally, there were problems with the localization, as already described for the alogrithm that used Nav2. (see [here](README.md#Incorrect-localization) ). If the robot is in a different place, which was not considered during the planning of the respective navigation step, the further travel of the robot is not possible and it leads ob to accidents.
In addition, the physical properties of the robot model are not yet fully developed, which is why the robot may have tipped into an unstable position when crossing the ramp and, for example, hopped or tipped completely onto its side.

## Results
Both algorithms are currently based on features, such as angles and hard-coded target positions, which must be specifically adapted to a parqour.

The self-written algorithm without Nav2 navigation libraries is very error-prone. Since no external plugins are used in the algorithm, parts of the algorithm are transferable to other systems. Problems caused by virtual collision areas are completely eliminated. It also makes the algorithm more resource efficient. Both the target selection and the adjustment of the target postions to an optimal run is time consuming. Also, as the complexity of the parqour increases, so does the number of targets to be assigned, since the robot can only travel from one target to another on a straight trajectory. This makes the adaptation of the algorithm for other parqours very complex and is therefore not recommended. Furthermore, the algorithm does not include environmental information into the control of the robot, which means that situational decisions are not possible.

The algorithm using the Nav2 navigation libraries is currently not very robust, due to many plugins and complex calculations, the function is dependent on the currently available computing power. Programs running in the background during simulation or many test runs in close succession often led to a deterioration in performance. The situations described under the section "incorrect localization" were mostly problematic during the execution. If one of these situations occurred early in the simulation, it led to a significant degradation of the localization already at the beginning, as described, and the robot usually got stuck after less than two runs from one of the errors also described under the section "incorrect localization". If there were few or none of these extraordinary situations that resulted in increased slippage, the robot was able to make more than five runs in a stable manner in many cases. If the localization works, the shape of the keepout filter is used to force a trajectory at the correct location, allowing the robot to cross from one ramp to the other with very little slip. Even though, due to the path planner and the keepout filter, the target positions to be predefined do not have to be adjusted as precisely as with the other algorithm, it still takes some time to find the optimal form of the keepout filter. However, the question is also whether such a precise adjustment is still necessary with a more robust localization. Especially the obstacle layer on the costmap in combination with the path planner offers the additional advantage of being able to react situationally to the environment. If an obstacle appears in front of the robot, the obstacle layer maps it in real time in the costmap. Based on this information, the path planner can plan a trajectory around the obstacle. The determination of the initialization position in each newly loaded map, despite its tendency to a small systematic error, contributes to a significant improvement in localization. A very useful side effect is also that, due to the self-determination of the initialization position at the beginning, the robot can be set to any start position on the first straight in the parqour, from where it then starts the run.

Based on its advantages, we recommend to use the algorithm using Nav2 for further improvements.

## Outlook

The main finding of this work is that situations in which a lot of slippage occurs at the wheels of the robot lead to a significant degradation of the localization. A more solid functioning of the algorithms, with less occurrence of these situations, shows that the problem is currently not with the control, but with the localization.

In view of this finding, it is recommended to focus on the localization, or more precisely on the improvement of the odometry, in the context of a possible further work. A possible solution approach would be the fusion of the odometry with the IMU using a Kalman filter. The AMCL can then perform a better position estimation based on the filtered odometry. For example, a pre-implemented Kalman filter is provided by [robot_localisation](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html), for which a first idea of a config-file can be found in the folder [config_ekf](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/tree/main/config_ekf). Furthermore, an improvement of the initialization position would be important. This estimate could be made more stable, for example, by a RANSAC algorithm. 

## Team

Katarina, Jakob, Daniel
