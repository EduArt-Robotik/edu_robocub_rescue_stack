# edu_robotcub_stack develop

## launch
ros2 launch edu_robocup_rescue_stack edu_control.launch.py 

## Team

Katarina, Jakob, Daniel

## Steuerung

Die Steuerung wurde bisher mit dem Gazebo Roboter Model eduard_offroad getestet und die Roboteransteuerung ist dem entsprechend implementiert. Um das Gazebo Roboter Model eduard_offroad zu steuern, muss eine geometry_msgs::msg::Twist unter /cmd_vel gepushed werden. In dem linear.x Wert wird die Forwärtsgeschwindigkeit und in angular.z die Drehgeschwindigkeit um den Yaw Winkel des Roboters, übergeben. 

## Orientierung und Lokalisierung

Um eine Steurung zu implementieren ist es wichtig zu wissen an welcher Position sich der Roboter zu jeden Zeitpunkt befindet. Um die Position des Roboters zu erhalten wurde sich für die Adaptive Monte Carlo Localization (AMCL) entschieden. Der hier verwende Algorithmus wird von Nav2 bereitgestellt. Er benötigt zur Lokalisation einen 2D Laserscanner und eine zuvor erstellte Map. [1]: https://github.com/ros-planning/navigation2/tree/main/nav2_amcl "Github nav2_amcl"

### Map Generieren 

Zum Generieren der Maps wurde die slam_toolbox von SteveMacenski verwendet. Die Generierung wurde mit dem online_async_launch.py gestartet, dass in https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py gefunden werden kann und eine Kopie in diesem Repository unter /launch/online_async_launch.py gefunden werden kann.

### Probleme AMCL

- Der Parkour ist 3 Dimensional. Neben Ebenen Flächen ist auch eine Rampe enthalten. Die Positionserkennung mit AMCL ist aber für eine ebene Fläche entwickelt worden. Bei der Entwicklung mit nur einer Map sind deshalb Probleme bei der Positionserkennung aufgetreten. Zum Beispiel erkannte der Roboter, wenn er auf der Eben steht, in der Rampe und wenn er auf der Rampe steht in der ebenen eine Mauer. Um dem Abhilfe zu schaffen wurde hier für jede Rampen- oder Ebenefläche je eine Map implementiert. Für diese Rampe ergibt es insgesamt 4. Die Rampen sind entsprechen des Roboterdurchlauf nummeriert.
Ein weiteres Problem, dass mit nur einer Map aufgetreten ist, war der Verwendete AMCL Algorithmus die Zweit Teile der Rampe nicht unterscheiden konnte. (Fläche 1+2 mit 3+4 verwechselt)

### Maps wechseln

Um eine Map für das Nav2 System bereit zu stellen wird der Map Server verwendet. Der Map Server Node stellt die Map beim Starten bereit. Mit load_map kann auch die Map während der Map Server Node schon läuft, ausgetauscht werden. Statt die Map zu tauschen kann auch statt die Map der ganze MapServer "getauscht" werden, indem die Maps Server aktiviert und deaktiviert werden. Dies ist Möglich da Map Server ein LifecycleNode ist. source https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/src/map_server/map_server.cpp line 65  Eine Liefecycle Node hat vier "primary states": Unconfigured, Inactivate, Active, Finalized. Zu Beginn ist jeder LifecyleNode in dem Unconfigured State der zu dem Inactive State transformiert werden kann. Von dem State aus kann der Node Active werden, indem die Map gepuplished wird und für andere Nodes, wie dem AMCL Node, zu Verfügung steht.
Um die Maps zu wechseln werden beim Starten ( mit amcl_4maps.launch.py) vier Map Server, einen für jede Map, gelaunched. Diese werden aber nicht vom  lifecycle_manager gemanagt, da dies nun im edu_robocup_rescue_stack_node passiert. source: https://design.ros2.org/articles/node_lifecycle.html 


#### Terminal
source: https://index.ros.org/p/lifecycle/


#### Implementiert
