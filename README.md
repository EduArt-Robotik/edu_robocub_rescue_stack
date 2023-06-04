# edu_robotcub_rescue_stack

## setup:

### Prerequisites:

- For simulation, the [edu_simulation-repository](https://github.com/EduArt-Robotik/edu_simulation) needs to be cloned to your local computer.

- In order to use the Navigation Stack 2 (Nav2) function library, the [navigation2-repostiory](https://github.com/ros-planning/navigation2) needs to be cloned to your local computer. 

- The [lattice_primitves](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/tree/main/lattice_primitives) which are generated specifically for theeduard offroad robot and are located in the "lattice_primitives" folder in this repository, must be placed in the cloned navigation2 repository under the path `/navigation2/nav2_smac_planner/lattice_primitves`.

### Launch:

1. Launch Gazebo:  
    `ros2 launch edu_simulation eduard.launch.py`

2. Select parqour „TER0_ramp“ in the „Insert“ tab and place it in the free space.

3. Click on parqour and set pose in left bar:  
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

## Einleitung / Aufgabenstellung

Ziel ist es ein Konzept für das autonome Fahren des Eduard-Offroad-Roboters durch einen simulierten Parqour zu entwerfen. 
Der Roboter soll einen vordefinierten Parqour ([(TER 1)Sand/Grave](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf)) so oft und so schnell wie möglich durchqueren. Wenn er am Ende des (TER 1)Sand/Grave Parqour angekommen ist, soll der Roboter rückwärts (ohne eine 180° Drehung) zurückfahren. Der Roboter muss zuerst ein flaches Element überfahren, dann ein Rampe hinauffahren, auf eine andere Rampe gelangen, diese dann wieder hinunterfahren und zuletzt auf einem flaches Element bis zu dessen Ende fahren. Eine besondere Schwierigkeit ist dass Überqueren der Rampe.

## Orientierung und Lokalisierung
Um den Roboter durch den Parqour zu Steuern wird zu jedem Zeitpunkt die aktuelle Position benötigt. Um die Position zu erhalten, wurde sich für die Adaptive Monte Carlo Localization (AMCL) entschieden. Der für die Lokalisierung verwendete Algorithmus wird von der open-source Bibliothek Nav2 der Navigation Community bereitgestellt. Er benötigt einen 2D Laserscanner und eine zuvor erstellte Map. Der Laserscanner wurde dafür in dem Gazebo Robotermodell hinzugefügt. [source: Github nav2_amcl](https://github.com/ros-planning/navigation2/tree/main/nav2_amcl)

### AMCL - Adaptive Monte Carlo Localisation

The AMCL is an algorithm that uses a particle filter to estimate the position and orientation of a robot in a known map, based on information from a 2D laser scanner and odometry. The AMCL has to initialize the particle filter with the initial pose of the robot, otherwise it starts at the origin of the reference coordinate system (Here: map_frame). The particles that should represent the pose of the robot are randomly distributed in space at the beginning. If the robot moves (motion-update), the algorithm uses the initialization position and the odometry information to make a prediction for the robot position more specifically the new position of the particles. Subsequently, the laser scanner performs a measurement that is compared to the predicted particle positions. Then the algorithm weights the particles according to the match with the measurement. Particles with higher weights have a higher probability of representing the actual pose. A process called resampling rejects particles with low weighting on it. This again adjusts the particle set more to the more likely outcome and thus improves the position estimate. The algorithm repeats these steps several times. The final estimate of the pose is calculated based on the weights of the particles, with higher weighted particles having a greater impact on the result. In addition to the pose, AMCL also publishes the transformation from the odometry coordinate system (odom_frame) to the global coordinate system (map_frame). [source_1](https://roboticsknowledgebase.com/wiki/state-estimation/adaptive-monte-carlo-localization/), [source_2](https://tams.informatik.uni-hamburg.de/lectures/2020ss/seminar/ir/doc/jtb_7328242_final.pdf)

### Probleme AMCL

Der Parkour ist dreidimensional. Neben ebenen Flächen ist auch eine Rampe enthalten. Die Positionserkennung mit AMCL ist aber nur für eine ebene Fläche entwickelt worden. Bei der Entwicklung eines Robotersteuerungsalgorithmus mit nur einer Map sind deshalb Probleme bei der Positionserkennung aufgetreten. Zum Beispiel erkannte der Roboter, wenn er auf der Ebene steht die Rampe als Wand und wenn er auf der Rampe steht Ebene als Wand. Um dem Abhilfe zu schaffen wurde hier für beide Rampen und beide ebene Fläche je eine Map implementiert.
Außerdem konnte der verwendete AMCL-Algorithmus durch die Symmetrie der Strecke nicht unterscheiden, auf welchem Teil der Strecke er sich befindet. 

### Map Generieren 

Zum Generieren der Maps wurde die [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
von SteveMacenski verwendet. Die Generierung wurde mit dem [online_async_launch.py](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py) gestartet. Eine [Kopie](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py) des Source Codes liegt in diesem Repository.

### Maps wechseln

Um die Map zu wechseln wurden zwei Implementationsmöglichkeiten verglichen. Eine neue Map kann entweder in einem Map Servers geladen werden und es kann für jede Map einen komplett neuen Map Server gestartet werden. Beide Möglichkeiten haben funktioniert.

Um eine Map für das Nav2 System bereit zu stellen, wird der Map Server verwendet. Der Map Server Node stellt die Map beim Starten bereit. Mit load_map kann auch die Map ausgetauscht werden, während der Map Server Node schon läuft. Statt die Map zu tauschen kann wie oben erwähnt auch der ganze Map Server "getauscht" werden, indem die Map Server aktiviert und deaktiviert werden. Dies ist möglich, da ein Map Server ein LifecycleNode ist. [source: Map_server.cpp line 65](https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/src/map_server/map_server.cpp)

Eine Liefecycle Node hat vier "primary states": Unconfigured, Inactivate, Active, Finalized. Zu Beginn ist jeder LifecyleNode in dem Unconfigured State, der zu dem Inactive State transformiert werden kann. Von dem State aus kann der Node Active werden, indem die Map gepuplished wird und für andere Nodes - wie zum Beispiel dem AMCL Node - zu Verfügung steht.
Um die Maps zu wechseln werden beim Starten (mit amcl_4maps.launch.py) vier Map Server gelaunched, einen für jede Map. Diese werden aber nicht vom lifecycle_manager gemanagt, da dies nun im edu_robocup_rescue_stack_node passiert. [source](https://design.ros2.org/articles/node_lifecycle.html)


#### Terminal
Zum Testzwecken wurde zuerst ein Map Wechsel durch Kommandos in dem Terminal ausgeführt. Um eine neue Map zu laden kann `ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}"`ausgeführt werden. [source](https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/README.md) 

Um die Map durch aktivieren und deaktivieren der Map-Server zu tauschen, ist es nötig den aktuellen State der Nodes abzufragen und den State Wechsel triggern zu können. Um den aktuellen Lifecycle State für den Beispiel Node mit dem Namen map_server zu erhalten kann ros2 service call `/map_server/get_state lifecycle_msgs/GetState` ausgeführt werden. Um den State von Unconfigured zu Inactive zu wechseln kann `ros2 lifecycle set /map_server configure` oder `ros2 service call /lc_talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 2}}"` ausgeführt werden. [source](https://index.ros.org/p/lifecycle/)

### Implementierung des Map Wechsels

##### Implementierung des Map wechsels mittels Load Map

Um einen Map mittels Load Map zu wecheln wird ein Client kreiert, mithilfe dessen an dem Map Server ein asyncRequest mit der gewünschten Rampe geschickt wird. Neben der gewünschten Rampe wird eine callback Funktion übergeben, durch die ein erfolgreiecher Wechsel festgestellt werden kann.

##### Implementierung des Map Server Wechsel zu Testzwecken
Zur Implementierung des Mapwechsel mittels Lifecycle Managements wurde eine neue Klasse erstellt. Dort werden die Funktionen activeServices und deactiveService zu Verfügung gestellt, mit denen die Map_server aktiviert oder deaktiviert werden können. Die Implementierung  wurde an dem service_client von thehummingbird angelehnt. [source](https://github.com/thehummingbird/robotics_demos/blob/main/lifecycle_node/src/demo_lifecycle/src/service_client.cpp)

Um den aktuellen State zu erhalten und eine State Wechsel zu triggern wird je ein asyncRequest gesendet. Im Gegensatz zur Implementierung von thehummingbird wurde dabei eine Callbackfunktion übergeben, durch die eine Rückmeldung über den erfolg/Scheitern, des wechsel der Lifecycle Node zusände übergeben wird.

###### Probleme bei der Implementierung der 4 Map Server
Bei der Implementierung der 4 Map Server ist es zu dem Problem gekommen, dass beim Starten oft nicht alle Map Server Nodes und die AMCL Node vollständig gestartet wurden. Um dieses Problem zu lösen, wartet der Thread nun zu Beginn (im Konstruktor des LocalisationControlNode ) für 2 Sekunden. Wenn nicht gewartet wird kann es zu Fehlern führen, da auf dem Map Server zugegriffen werden kann, obwohl der Node nicht bereit ist.

#### Vergleich 
Durch beide Implementierung kann die Map gewechselt werden. Die Implementierung mittels MapServer Wechsel ist deutlich aufwändiger als die Implementierung Load Map. Deshalb wurde sich Letztendlich für die Implementierung mittels Load Map entschieden.

## Control

This repository branch contains an algorithm that uses navigation stack 2 (Nav2). As part of the search for the concept of traversing the course, two algorithms were developed and tested. One algorithm does not use ROS Nav2 or any other third party navigation library, the other is the algorithm mentioned earlier, which is using ROS Nav2. Both are developed in C++. All tests of the two algorithms were performed in Gazebo with an [Eduard offroad robot](https://github.com/EduArt-Robotik/edu_simulation/tree/feature/sand_gravel_ramp/model/eduard_offroad) on the [TER0_ramp](https://github.com/EduArt-Robotik/edu_simulation/tree/feature/sand_gravel_ramp/model/TER0_ramp) track.

### Algorithm using Nav2
This branch contains the source code of the algorithmen using Navigation Stack 2.

#### Navigation:

##### Process:

As explained earlier, position detection based on two dimensions with AMCL does not provide the possibility of continuous localization over the entire three-dimensional course. Due to this, depending on the current level (straight 1, ramp 1, ramp 2 oder straight 2), a specific map needs to be loaded ([LoadMap](README.md#Implementierung-des-Map-Wechsels)) in which the robot position has to be reinitialized every time.

![Strecke: Ter0_Ramp](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/bev_strecke.png)

The robot must also be given a new target, which it must approach on the map. An advantageous procedure is to load the layer-specific map at the beginning, then to send the target position and finally to initialize the robot pose.

Intuitively, one would send the target position to the path planner after the robot position has been initialized, but since the robot should preferably have no pitch and roll angle for an accurate determination of the initialization position, the program has to wait until it is straight again after jumping over the ramp. The waiting-duration is set to 1.0 seconds. The algorithm then checks the status of the robot. Immediately assigning the new target prevents the robot from orienting itself back to the previous target (old map) in the meantime and instead allows it to move directly in the right direction (new target).

Based on different angle constellations as well as the current X/Y-position, the currently traveled plane can be identified. The registration of a change of the plane is followed by the loading of a new map, the navigation to a new destination selected depending on the plane as well as on the current direction of movement, and the initialization of the new robot pose. This procedure is identical for each map.

##### Goal-position: 

For the way to the end of the track and back, one given target position (incl. orientation) per map would be sufficient. Due to the tendency of the path planner to calculate a path for the target on the ramps that runs very close to obstacles or along the collision areas of the costmap and therefore sometimes led to problematic situations, two targets are now assigned for the outward and return paths on the ramps. On the straights, one target is sufficient in each case.

##### Initial-position:

The X/Y position is determined by the distances of certain laser beams (straight ahead, straight back, right and left) measured by the laser scanner relative to the walls of the obstacle course.

Abhängig von der aktuellen Position auf der Strecke finden unterschiedliche Wände Verwendung zur Abstandsmessung und somit zum Erhalt der Roboter-Position. 
Depending on the current position on the track, different walls are used for distance measurement and therefore for maintaining the robot position.
Preferred are walls that show more of a perpendicular angle to the plane the robot is on. This allows minimizing the error generated by the robot due to its own motion around pitch and roll axes.

Straight 1:    Wall right, when moving in forward dircection & wall straight backwards
Ramp 1:    Wall right, when moving in forward direction & wall sraight forwards
Ramp 2:    Wall left, when moving in forward direction & wall straight backwards  
Straight 2:    Wall left, when moving in forward direction & wall straight forwards  

Depending on the origin point of the map (siehe [Karten-Konfigurations-Datei](map/map_1.yaml)), an offset is added to the measured distance to get the true pose relative to the global coordinate system (map_frame).The determination of the laser beams pointing in the necessary direction (perpendicular to the wall) regardless of the current orientation of the robot is done with the help of the Yaw angle of the IMU. The IMU also provides the information of the orientation (X, Y, Z, W) of the robot for the initialization pose.

#### ROS Navigation Stack 2:

The actual navigation through the course is based on the ROS Nav2. A path planner subscribes to the published target positions and then plans a path through a costmap to the target. Because of large differences in height between specific areas of the two ramps, the secure driving from one ramp the the other is not everywere possible. For that reason a keepout-filter above the costmap ensures that the transition takes place at a safe place. Once a valid path is found, a "FollowPath" plugin controls the robot movement along the path. A Behaviour Tree controls the regeneration behaviour of the robot in special cases, such as the robot deviating from the path or getting stuck.

##### Path-Planner:

Since time is very important for the runs of the Robocup, the path planning has to be designed in a way that a backward movement of the robot is possible and the time for turning around on the narrow course can be saved. Only two path planners in the Nav2 function library provide this feature. The Smac Hybrid Planner (Hybrid A* Planner) and the Smac Lattice Planner (State Lattice Planner). Since the Smac Hybrid Planner is not suitable for differentially driven robots, the Smac Lattice Planner was chosen.  

The Smac Lattice Planner is built on a grid-based data structure called "Sparse-map". The robot's ability to rotate continuously about the vertical axis ensures that, at certain rotation angles, straight trajectories do land on endpoints that are not aligned with the grid. To solve this problem, an individually parameterizable [„Lattice Primitiv Generator“](https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner/lattice_primitives) generates standard trajectories in advance.  For this purpose, this discretizes the possible angles of the robot's motion directions in such a way that straight trajectories land on endpoints that coincide with the discrete grid points of the grid. The so-called "minimum control set", which contains these trajectories, has to be generated manually for each robot individually with the "Lattice Primitive Generator". [source](https://navigation.ros.org/configuration/packages/smac/configuring-smac-lattice.html)


##### Controller-server

The controller server is a task server which is used to execute navigation tasks consisting of multiple steps and actions. It implements the follow-path plugin, which generates the command velocities for following the trajectory generated by the path planner. The Nav2 function library contains several [path following plugins](https://navigation.ros.org/plugins/index.html). However, most of them only allow target tracking while the robot is moving forward. This results in the robot having to either turn in place to the target or turn around in several moves. Since, as already described in the "Path Planner" section, this time needs to be saved, the work uses the Regulated Pure Pursuit Controller, which allows trajectory tracking in reverse.[source](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)

The tracking algorithm has the ability to regulate speed based on collision, trajectory curvature, or high cost in the costmap. In practice it has been shown that these regulatory functions are contra-productive in this use case and that the robot is more inclined to get stuck on the course as a result. An example from practice is that in order to move from a straight plane to a ramp, the robot must move to a target position that is at the end of the map and thus close to the inflation layer (high cost). If the robot receives regulated speed commands, its speed is reduced so that it is too slow to drive up the ramp in one go and thus gets stuck briefly. This can cause a drift in the localization, which has a negative effect on the further travel. For this reason, the regulations are switched off. [source](https://navigation.ros.org/configuration/packages/configuring-regulated-pp.html) 

Besides the path following plugin, the controller server implements progress checker and goal checker plugins. For this purpose, the Nav2 standard plugins Simple Progress Checker and Simple Goal Checker are in use. The stateful parameter of the Goal Checker can be used to determine whether it saves the progress of the robot over multiple planning runs. Since storage only provides an advantage in dynamic environments and otherwise only consumes computing power, this feature is disabled. [source](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)


##### Costmap

The basis of the costmap are the two-dimensional raster maps generated by the slam_toolbox. Using various layers and filters, the Costmap integrates the information from the laser scanner into the map. This allows the representation of the real dynamic environment of the robot. Based on the information contained in the costmap, the path planner plans the trajectories. [source](https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d)

Although SmacLatticePlanner is a global path planner, it is possible to include a local costmap in addition to a global costmap, resulting in an improvement in the accuracy of the trajectories.

The global costmap is used for long-term planning through the entire map and gives a rough overview of the entire environment of the robot. Due to the size of the planning area and thus the environment to be analyzed, which the global costmap covers, it has a slow execution and update time, which is why in this case only layers and filters are integrated, which refer to static obstacles. An inflation layer, a static layer and a keepout filter.

The inflation-layer is the most important layer of the costmap. It is an "obstacle magnification layer" in the map that is used to inflate obstacles to create a safe distance around them. This safety distance must be set manually by the "inflation-radius" parameter. The inflation of the obstacles is necessary because the path planner considers the robot simplified as a point model during the trajectory calculation, which in turn leads to the neglect of the actual dimensions. The path planner treats the inflated area of obstacles as a collision area and sets the trajectories to avoid collisions with the robot's wheels. In general, it is recommended to keep the parameters "infaltion-radius" and "cost-scaling-factor" large, as this tends to push the path planner to travel over free areas in the map. It has been shown in practice and especially on this track that if the "inflation-radius" is large, the robot is often within the "inflation-layer" and so inside the collision area after the jump. If the robot stands in this area, the planner cannot plan a valid trajectory to the target position and the robot stops. For this reason, the "inflation radius" is kept small. The radius of the robot is used as a rough orientation. [source](https://navigation.ros.org/configuration/packages/costmap-plugins/inflation.html)

The static-layer contains information about the position and shape of known and static obstacles registered in the course of creating the maps with the slam_toolbox. [source](https://navigation.ros.org/configuration/packages/costmap-plugins/static.html)

The use of a keepout filter is essential on this route when the robot is controlled by a path planner. The reason for this is that when planning a straight trajectory from one ramp to the other, the path planner makes the robot drive over a high landing, which normally causes it to roll over. A keepout filter is a filter that allows the user to manually create zones that appear as collision areas in the costmap, causing the path planner to avoid these zones when creating the trajectory.

###### Keepout-Filter:

![keepout_filter](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/keepout_filter_90.png)

###### Globale-Costmap mit überlagertem Keepout-Filter:

![global_costmap](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/global_costmap.png)

As it can be seen in the keepout filter image above, the design is such that the path planner has only a very small area through which to plan the trajectory. This area is located a little bit behind the intersection of the ramp planes, so that the robot only has to drive down a small ledge and thus reach the following level effortlessly and safely. Since the keepout filter is fixed, the robot would have to drive back up the heel as part of the return trip. Since this usually leads to problems, the following maps are shifted in the direction of the positive X-axis. The shift allows the keepout filter to be placed over the map exactly so that the robot will move down the landing on the other ramp when it returns. The position of the map in the Global Coordinate System can be set in the map configuration file. The slam_toolbox creates the map configuration file in the course of creating the map and assigns the origin position of it automatically. This origin position is often incorrect and should be checked when generating maps with the slam_toolbox. Also seen in the picture of the keepout filter above is that it is somewhat bulbous in shape on both sides of the opening. Tests have shown that a slightly transverse crossing is best suited both in terms of stable driving and subsequent onward travel. The bulbous shape of the keepout filter at this point introduces this cross travel. For creating the keepout filter as well as editing the maps, the image editing program [gimp](https://www.gimp.org/) has proven to be good.[source](https://navigation.ros.org/configuration/packages/costmap-plugins/keepout_filter.html)

The local costmap adjusts the rough path planning, which was initially based on the global costmap, according to the situation. It covers only a small area immediately around the robot compared to the global costmap (entire map). This area contains very detailed real-time information about obstacles in the robot's nearby environment, allowing for more precise path planning. In addition, an obstacle layer is added specifically for obtaining information about dynamic events.
###### Lokale-Costmap:

![local_costmap](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/local_costmap.png)

The obstacle-layer obtains its information about the robot's environment via the laser scanner. It detects and stores the shape and position of obstacles in real time and continuously updates the associated costmap, allowing the path planner to adjust the trajectory according to the situation. Although the obstacle-layer is more suitable for dynamic environments and the parqour in which the robot moves is purely static, the use of the obstacle-layer offers a significant advantage. At the moment of jumping from one ramp to the other, the robot partially tilts so much that the laser scanner detects the ground as an immediate obstacle and transmits this to the costmap through the obstacle-layer. This ensures that the robot first brakes and stops for a moment. On the one hand, braking ensures a reduced slip in contrast to a strongly oscillating to bouncing further travel and, on the other hand, makes it possible for the robot to stand there relatively still during the determination of the initialization pose. [source](https://navigation.ros.org/configuration/packages/costmap-plugins/obstacle.html)

#### Problems and Errors:

The following is a list and description of problems and errors that occurred at irregular intervals during the test runs.

##### TF_NAN_INPUT-Error

Original-error:  
Error: Ignoring transform for child_frame_id “odom” from authority “Authority undetectable” because of nan value in the transform (nan nan nan) (0.000000 0.000000 -0.770181 0.637825)

During the test runs on the obstacle course in Gazebo, the error occurred more often when two or more wheels of the robot were not in contact with the road, for example when "jumping" from one ramp to the other or when braking too hard on the ramp, which lifted the rear wheels.

In general, a TF_NAN_INPUT error occurs if a nan (not a number) value, i.e. an invalid or non-computable value, appears within a transformation. Transformations are used to transfer positions and orientations from one coordinate system to another. Dynamic transformations that need to be performed in this work while the robot is moving are the transformation from odom_frame to base_frame generated by the robot model (.sdf-file) itself and the transformation from odom-frame to map-frame published by AMCL.

Since the TF_NAN_INPUT error is related to odometry, a first possible cause of the error would be that it does not receive valid or reliable sensor data from the wheel speed sensors during the loss of ground contact, which causes the NAN values in the transformation to occur.

Parallel to the TF_NAN_INPUT error the following error occurs:

Original-error:  
[WARN] [amcl]: AMCL covariance or pose is NaN, likely due to an invalid configuration or faulty sensor measurements! Pose is not availabl!

This error could also result from the faulty odometry, since the AMCL's position detection is based on the odometry information in addition to the data from the 2D laser scanner.

##### Incorrect localization

The final estimation of the pose and thus the localization of the robot is performed by the AMCL. Since the AMCL's position estimation is based on the detection of features in the 2D laser scanner's sensor data, it could be that the symmetrical path, especially on the straight lines 1 and 2, which have the walls and corners as the only features, causes difficulties for the AMCL. However, the finding that the localization problems are particularly evident on ramps, which have much more specific characteristics, suggests another main cause. Nevertheless, a contribution to the deterioration of localization cannot be ruled out.

Due to the large amount of slip that occurs at the robot's wheels at many points along the track, it can be assumed that the odometry, on which AMCL's position estimation is based to a significant extent, is a major cause of faulty localization. A particularly large amount of slippage occurs in the following situations:

- Turning the robot on the ramp while standing in the direction of the target position often causes it to slide down the ramp a bit due to the simultaneous rotation of the wheels in opposite directions.

- During heavy braking, the robot sometimes swings up sharply due to the soft suspension and makes several movements across the roadway. This behavior can be seen mainly during downhill travel on the ramps.

- Driving the landing from one ramp to the other occasionally results in loss of ground contact of individual or all wheels. 

→ Odometry can register the movement in the described situations only poorly or not at all.

To improve the localization, the algorithm publishes an initialization position after each loading of a new map. The determination of this position is based on the distance to the walls in front of or behind and next to the robot measured by the laser scanner. This distance measurement is precise as long as the robot has no roll and/or pitch angle. However, if this is the case, it distorts the distance measurement slightly and a systematic error occurs. At this point, the algorithm can calculate this error for a very small yaw angle or for a yaw angle close to pi.


##### Stuck in inflation-layer or keepout-filter

Both the inflation-layer and the keepout-filter are represented as collision regions in the costmap. Since the "jump" from one ramp to the other is very uncontrolled, it has happened that the robot lands within these collision areas. If this is the case, the path planner cannot calculate a path to the target position and the robot stops at that position. One approach to solving this problem would be to adjust the recovery mode of the robot, which is defined in the Nav2 behavour-tree.


### Selbstgeschriebener Algorithmus ohne Navigationsbibliotheken


Neben dem oben beschriebenen Algorithmus, wurde ein weiterer Algorithmus entwickelt, der (abgesehen von Verwendung von Funktionen der Lokalisierungsbibliotheken) komplett selbstgeschrieben ist und keine Funktionen der Navigationsbibliotheken verwendet. Der Source Code für diesen Algorithmus liegt im branch [4maps](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/tree/4maps).
In diesem Algorithmus gibt es 4 Teilkarten sowie 14 navigation-steps (der Grund für die Verwendung von 4 Karten kann [hier](README.md#probleme-amcl) nachgelesen werden). Der Roboter fährt dabei sukzessive die definierten Punkte ab. Diese Punkte befinden sich auf dem Streckenmodell für die TER1 Strecke und sind fest implementiert, weshalb der Algorithmus primär nur für diese Strecke eingesetzt werden kann. Die Punkte befinden sich folgendermaßen auf der Rampe:

![Rampe Punkte](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/4MapsDestPos.jpg?raw=true "Rampe mit von dem Algorithmus verwendet Punkten")

Die Roboter fährt dabei die Punkte der Reihe nach ab. Zu bemerken ist, dass Punkt 2 im Vergleich zu Punkt 7 versetzt ist und Punkt 3 im Vergleich zu Punkt 8. Damit wurde sichergestellt, dass der Roboter beim Überqueren der Rampe weder auf dem Hinweg (Punkt 2 zu 3) noch auf dem Rückweg (Punkt 7 zu 8) gegen eine Wand fährt sondern etwas weiter rampen-aufwärts die Verschränkung überquert und dann leicht nach unten fällt.

![Rampe Maps](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/4MapsMaps.jpg?raw=true "Rampe mit Maps")

#### Maps:
1. Map: grün,
2. Map: rot
3. Map: blau
4. Map: gelb

#### Navigationsteps:
0. Step: fahre auf Punk 1 zu, die Fahrtrichtung ist forwärts und Map 1 ist geladen
1. Step: fahre auf Punkt 2 zu, beobachte pitch-Winkel und wechsle Zustand, wenn der Roboter auf die erste Rampe gefahren ist
2. Step: lade Map2 und fahre auf Punkt 2 zu
3. Step: fahre auf Punkt 3 zu, beobachte pitch und roll-Winkel wechsle Zustand, wenn der Roboter über die "Klippe" gefahren ist.
4. Step: lade Map3 und fahre auf Punkt 4 zu. Beobachte roll und Pitch winkel um Zustand zu wechseln, wenn Roboter von der Rampe auf, den geraden Teil fährt.
5. Step: lade Map4 und fahre weiter auf Punkt 4 zu
6. Step: fahre auf Punkt 5 zu.
7. Step: ändere Fahrrichtung Rückwärts und fahre auf Punkt 6 zu.
8. Step: fahre auf Punkt 7 zu, beobachte pitch-Winkel um zustand zu Wechseln, wenn Roboter auf Rampe fährt
9. Step: lade Map3 und fahre weiter auf Punkt 7 zu
10. Step: Fahre auf Punkt 2 zu, beobachte pitch und roll-Winkel wechsel Zustand, wenn der Roboter über die "Klippe" gefahren ist.
11. Step: lade Map2 und fahre auf Punkt 1 zu. Beobachte roll und Pitch Winkel um Zustand zu wechseln, wenn Roboter von der Rampe auf, den geraden Teil fährt.
12. Step: lade Map1 und fahre weiter auf Punkt 1 zu.
13. Step: fahre auf den Startpunkt zu.

Der Algorithmus geht davon aus, dass der Roboter zu Beginn auf einen fest definierten Startpunkt (Streckenanfang) gesetzt wurde. Der Algorithmus besteht im Wesentlichen aus zwei Zustände: Drehen (auf der Stelle) und Fahren (geradeaus vorwärts). Außerdem wird kontinuierlich (unabhängig vom Zustand) geprüft, ob der Roboter in Richtung des aktuellen Zielpunktes gedreht ist. Sollte dies nicht der Fall sein, stoppt der Roboter und beginnt sich zu drehen.

#### Drehen:
Bei einer Drehung dreht sich der Roboter so weit, bis er gerade auf diesen nächsten Punkt zufahren kann. Er stoppt mit der Drehbewegung, wenn er sich innerhalb eines Toleranzbereiches des anfangs für die Drehung berechneten Winkels befindet. Es wurde ein Toleranzbereich festgelegt, der einerseits möglichst genau ist, allerdings nicht zu fein, da die Winkeldifferenz nur in einem festdefinierten Takt berechnet wird und sich der Roboter deshalb möglichst wenig über das Ziel hinausdrehen soll. Diese Toleranz wurde durch Tests in der Simulation ermittelt. Während der Drehung wird der Roboter vom Algorithmus auf der Stelle gedreht und legt keinen Weg mit der Drehung zurück. Dadurch bleibt der Wendekreis sehr klein und der Roboter läuft nicht in Gefahr, ungewollt gegen eine Wand zu fahren.

#### Fahren:
Nachdem der Roboter in die jeweils richtige Richtung gedreht ist, beginnt er mit der Vorwärtsfahrt.  Je näher der Roboter dem aktuellen Zielpunkt kommt, desto langsamer fährt er. Sobald sich der Roboter in einem bestimmten Toleranzradius um den aktuellen Zielpunkt befindet, wird der Zielpunkt aktualisiert und der Robotor beginnt auf den nächsten Zielpunkt zuzufahren. 

#### Besonderheit beim Überqueren der Rampenverschränkung:
Der Roboter benötigt für diesen Algorithmus die Odometrie Daten aus den Lokalisierungsbibliotheken. Auf dem Weg von Punkt 2 zu Punkt 3 fährt der Roboter solange, bis er die Kante überfahren hat und nach unten kippt. Dies merkt der Algorithmus, weil dann der Pitch-Winkel größer oder gleich 0,1 und der Roll-Winkel größer als 0 ist.
Dies ermöglicht es, dass der Roboter in einem optimalen Winkel die Rampen wechselt. 

#### Implementierungsdetails:
Der Algorithmus wurde bisher mit dem Gazebo Roboter Model eduard_offroad getestet und die Implementation ist speziell darauf zugeschnitten. Um das Gazebo Roboter Model eduard_offroad zu steuern, muss eine 'geometry_msgs::msg::Twist' unter '/cmd_vel' gesendet werden. In dem 'linear.x' Wert wird die Forwärtsgeschwindigkeit und in dem 'angular.z' Wert die Drehgeschwindigkeit um den Yaw Winkel des Roboters übergeben.

#### Probleme :
Der Algorithmus hat prinzipiell funktioniert, solange es keine Probleme mit der Lokalisierung gab. Die Lokalisierung hatte an den selben Stellen besonders viele Probleme wie der  Algorithmus, der die Navigationsbibliothek verwendet (siehe [hier](README.md#Fehlerhafte-Lokalisierung) ). Die Navigationsschritte sind sehr fehleranfällig. Es kam zu dem Fehler, dass zu früh, oder gar nicht in den nächsten Navigationsschritt gesprungen wurde. Wenn der Roboter sich an einer anderen Stelle befindet, die nicht bei der Planung des jeweiligen Navigationsschritt beachtet worden ist, ist die weiterfahrt des Roboter nicht möglich und es führt ob zu unfällen.
Zusätzlich sind die physikalischen Eigenschaften des Robotermodells noch nicht komplett ausgereift, weshalb der Roboter bei der Rampenüberquerung unter Umständen in eine instabile Lage gekippt ist und z.B. gehüpft oder ganz auf die Seite gekippt ist.

#### Results
Both algorithms are currently based on features, such as angles and hard-coded target positions, which must be specifically adapted to a parqour.

The self-written algorithm without Nav2 navigation libraries is very error-prone. Since no external plugins are used in the algorithm, parts of the algorithm are transferable to other systems. Problems caused by virtual collision areas are completely eliminated. It also makes the algorithm more resource efficient. Both the target selection and the adjustment of the target postions to an optimal run is time consuming. Also, as the complexity of the parqour increases, so does the number of targets to be assigned, since the robot can only travel from one target to another on a straight trajectory. This makes the adaptation of the algorithm for other parqours very complex and is therefore not recommended. Furthermore, the algorithm does not include environmental information into the control of the robot, which means that situational decisions are not possible.

The algorithm using the Nav2 navigation libraries is currently not very robust, due to many plugins and complex calculations, the function is dependent on the currently available computing power. Programs running in the background during simulation or many test runs in close succession often led to a deterioration in performance. The situations described under the section "incorrect localization" were mostly problematic during the execution. If one of these situations occurred early in the simulation, it led to a significant degradation of the localization already at the beginning, as described, and the robot usually got stuck after two runs from one of the errors also described under the section " incorrect localization". If there were few or none of these extraordinary situations that resulted in increased slippage, the robot was able to make more than five runs in a stable manner in many cases. If the localization works, the shape of the keepout filter is used to force a trajectory at the correct location, allowing the robot to cross from one ramp to the other with very little slip. Even though, due to the path planner and the keepout filter, the target positions to be predefined do not have to be adjusted as precisely as with the other algorithm, it still takes some time to find the optimal form of the keepout filter. However, the question is also whether such a precise adjustment is still necessary with a more robust localization. Especially the obstacle layer on the costmap in combination with the path planner offers the additional advantage of being able to react situationally to the environment. If an obstacle appears in front of the robot, the obstacle layer maps it in real time in the costmap. Based on this information, the path planner can plan a trajectory around the obstacle. The determination of the initialization position in each newly loaded map, despite its tendency to a small systematic error, contributes to a significant improvement in localization. A very useful side effect is also that, due to the self-determination of the initialization position at the beginning, the robot can be set to any start position on the first straight in the parqour, from where it then starts the run.

It was decided to use the algorithm using Nav2, and it is recommended to use this algorithm for all further improvements.

### outlook

The main finding of the work is that situations in which a lot of slippage occurs at the wheels of the robot lead to a significant degradation of localization. A solid functioning of the algorithms, with less occurrence of these situations, shows that the problem is currently not with the control, but with the localization.

In view of this finding, it is recommended to focus on the localization, or more precisely on the improvement of the odometry, in the context of a possible further work. A possible solution approach would be the fusion of the odometry with the IMU using a Kalman filter. The AMCL can then perform a better position estimation based on the filtered odometry. For example, a pre-implemented Kalman filter is provided by [robot_localisation](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html). Furthermore, an improvement of the initialization position would be important. This estimate could be made more stable, for example, by a RANSAC algorithm.

## Team

Katarina, Jakob, Daniel
