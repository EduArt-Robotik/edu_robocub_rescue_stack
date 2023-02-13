# edu_robotcub_stack develop

## launch
ros2 launch edu_robocup_rescue_stack edu_control.launch.py 

## Team

Katarina, Jakob, Daniel

## Steuerung

Die Steuerung wurde bisher mit dem Gazebo Roboter Model eduard_offroad getestet und die Roboteransteuerung ist dem entsprechend implementiert. Um das Gazebo Roboter Model eduard_offroad zu steuern, muss eine geometry_msgs::msg::Twist unter /cmd_vel gepushed werden. In dem linear.x Wert wird die Forwärtsgeschwindigkeit und in angular.z die Drehgeschwindigkeit um den Yaw Winkel des Roboters, übergeben. Der Algorithmus der Implementiert ist ist ein Punktfolgealgorithmus. Je näher der Roboter dem Punkt kommt, desto langsamer fährt der Roboter. Der Roboter muss sich in einem bestimmten Radius um den Punkt befinden damit der nächste Punkt angesteuert wird. Wenn der Roboter die Rampe wechselt soll der Roboter nicht den Zielpunkt ansteuer, da dieser ersetzt wird sobald der Roboter die Rampe heruntergefallen ist. Um zu gewährleisten, dass der Roboter im Richtigen Winkel die Rampen wechselt.

## Orientierung und Lokalisierung

Um eine Steurung zu implementieren ist es wichtig zu wissen an welcher Position sich der Roboter zu jeden Zeitpunkt befindet. Um die Position des Roboters zu erhalten wurde sich für die Adaptive Monte Carlo Localization (AMCL) entschieden. Der hier verwende Algorithmus wird von Nav2 bereitgestellt. Er benötigt zur Lokalisation einen 2D Laserscanner und eine zuvor erstellte Map. [1]: https://github.com/ros-planning/navigation2/tree/main/nav2_amcl "Github nav2_amcl"

### Map Generieren 

Zum Generieren der Maps wurde die slam_toolbox von SteveMacenski verwendet. Die Generierung wurde mit dem online_async_launch.py gestartet, dass in https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py gefunden werden kann und eine Kopie in diesem Repository unter /launch/online_async_launch.py gefunden werden kann.

### Probleme AMCL

Der Parkour ist 3 Dimensional. Neben Ebenen Flächen ist auch eine Rampe enthalten. Die Positionserkennung mit AMCL ist aber für eine ebene Fläche entwickelt worden. Bei der Entwicklung mit nur einer Map sind deshalb Probleme bei der Positionserkennung aufgetreten. Zum Beispiel erkannte der Roboter, wenn er auf der Eben steht, in der Rampe und wenn er auf der Rampe steht in der ebenen eine Mauer. Um dem Abhilfe zu schaffen wurde hier für jede Rampen- oder Ebenefläche je eine Map implementiert. Für diese Rampe ergibt es insgesamt 4. Die Rampen sind entsprechen des Roboterdurchlauf nummeriert.
Ein weiteres Problem, dass mit nur einer Map aufgetreten ist, war der Verwendete AMCL Algorithmus die Zweit Teile der Rampe nicht unterscheiden konnte. (Fläche 1+2 mit 3+4 verwechselt)

### Maps wechseln

Um eine Map für das Nav2 System bereit zu stellen wird der Map Server verwendet. Der Map Server Node stellt die Map beim Starten bereit. Mit load_map kann auch die Map während der Map Server Node schon läuft, ausgetauscht werden. Statt die Map zu tauschen kann auch statt die Map der ganze MapServer "getauscht" werden, indem die Maps Server aktiviert und deaktiviert werden. Dies ist Möglich da Map Server ein LifecycleNode ist. source https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/src/map_server/map_server.cpp line 65  Eine Liefecycle Node hat vier "primary states": Unconfigured, Inactivate, Active, Finalized. Zu Beginn ist jeder LifecyleNode in dem Unconfigured State der zu dem Inactive State transformiert werden kann. Von dem State aus kann der Node Active werden, indem die Map gepuplished wird und für andere Nodes, wie dem AMCL Node, zu Verfügung steht.
Um die Maps zu wechseln werden beim Starten ( mit amcl_4maps.launch.py) vier Map Server, einen für jede Map, gelaunched. Diese werden aber nicht vom  lifecycle_manager gemanagt, da dies nun im edu_robocup_rescue_stack_node passiert. source: https://design.ros2.org/articles/node_lifecycle.html 
Request

#### Terminal
Zum Testzwecken wurde zuerst ein Map Wechsel durch Kommandos in dem Terminal ausgeführt. Um eine neue Map zu laden kann `ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}"`ausgeführt werden. source: https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/README.md Um die Map durch aktivieren und deaktivieren der Map-Server zu tauschen, ist es nötig den aktuellen State der Nodes abzufragen und den State wechsel triggern zu können. Um den aktuellen Lifecycle State für den Beispiel Node mit dem Namen map_server zu erhalten kann ros2 service call `/map_server/get_state lifecycle_msgs/GetState` ausgeführt werden. Um den State von Unconfigured zu Inactive zu wechseln kann `ros2 lifecycle set /map_server configure` oder `ros2 service call /lc_talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 2}}"` ausgeführt werden. source: https://index.ros.org/p/lifecycle/


#### Implementiert
Zur Implementierung des Mapwechsel mittels Lifecycle Managements wurde die Klasse `clientService` erstellt. Dort werden die Funktionen activeServices und deactiveService zu Verfügung gestellt, mit denen die Map_server aktiviert oder deaktiviert werden können. Bei der Implementierung  wurde sich an dem service_client von thehummingbird inspiriert. source: https://github.com/thehummingbird/robotics_demos/blob/main/lifecycle_node/src/demo_lifecycle/src/service_client.cpp Um den aktuellen State zu erhalten und eine State Wechsel zu triggern wird je ein asyncsenRequest aufgefürht. Endgegengesetzt zur Implementierung von thehummingbird wurde dabei eine Callbackfunktion übergeben die das Ergebnis enthält. Um die Map zu wechseln indem eine neue Map geladen wurde die Klasse `clientService` implementiert.

#### Probleme
Damit die Map Server Node und die AMCL Node follständig gestarten wurden wartet der Thread zu Beginn (im Konstruktor des LocalisationControlNode ) für 2 Sekunden. Wenn nicht gewartet wird kann es zu Fehlern führen, indem auf dem Map Server zugegriffen wird, obwohl der Node nicht bereit ist.