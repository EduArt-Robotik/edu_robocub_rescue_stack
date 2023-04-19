# edu_robotcub_stack

## launch
ros2 launch edu_robocup_rescue_stack edu_control.launch.py 

## Team

Katarina, Jakob, Daniel


## Steuerung

### Nav


### Alternative Steuerung

Zusätzlich wurde getestet, ob der Roboter den Parkour mit einem von diesem Team eigens implementierten Algorithmus durchfährt, der keine Navigationsbibliotheken verwendet. In dem Algorithmus fährt der Roboter definierte Punkte ab. Der Roboter dreht sich bis er unter einer Toleranz von x den Winkel erreicht hat indem er gerade auf dem nächsten Punkt zufahren kann. Die Toleranz wurde durch Tests in der Simulation ermittelt. Es wurde eine Toleranz gesucht die möglichst genau ist, die der Roboter in einem Drehbewegungsschritt trotzdem eintritt. Während sich der Roboter dreht fährt er nicht vorwärts, damit er einen kleinen Wendekreis hat und nicht an Wände stürzt. Damit der Roboter stabiler fährt wurde er .. Wenn er gegen eine Wand gefahren ist, sollte der Roboter wieder zurückfahren. Je näher der Roboter dem Punkt kommt, desto langsamer fährt der Roboter. Der Roboter muss sich in einem bestimmten Radius um den Punkt befinden damit der nächste Punkt angesteuert wird, ausgenommen an der Stelle wo der Roboter die Rampe herunterfällt, da wird der Zielpunkt nach dem Fall schon ersetzt. Dies ermöglicht es, dass der Roboter in einem optimalen Winkel die Rampen wechselt. 
Die Steuerung wurde bisher mit dem Gazebo Roboter Model eduard_offroad getestet und die Roboteransteuerung ist dem entsprechend implementiert. Um das Gazebo Roboter Model eduard_offroad zu steuern, muss eine geometry_msgs::msg::Twist unter /cmd_vel gepushed werden. In dem linear.x Wert wird die Forwärtsgeschwindigkeit und in angular.z die Drehgeschwindigkeit um den Yaw Winkel des Roboters, übergeben. Dieser Algorithmus hat funktioniert, solange es keine Probleme mit der Lokalisierung gab. Im Durchschnitt gab es jeden 3 Durchlauf an einer Stelle ein Problem mit der Lokalisierung, die dem Roboter eine weiterfahrt nicht mehr möglich gemacht hat. Im Vergleich zu der Steuerung mit Nav ist dies .... Ein weiterer Nachteil zu der Steuerung mit Nav ist, dass in dem Algorithmus mehr Punkte vom Entwickler definiert werden müssen, wodurch eine Anpassung zu einem anderen Rampenlayout länger dauert.

## Orientierung und Lokalisierung

Um eine Steuerung zu implementieren ist es wichtig zu wissen an welcher Position sich der Roboter zu jeden Zeitpunkt befindet. Um die Position des Roboters zu erhalten wurde sich für die Adaptive Monte Carlo Localization (AMCL) entschieden. Der hier verwende Algorithmus wird von Nav2 bereitgestellt. Er benötigt zur Lokalisation einen 2D Laserscanner und eine zuvor erstellte Map. [1]: https://github.com/ros-planning/navigation2/tree/main/nav2_amcl "Github nav2_amcl"

### Map Generieren 

Zum Generieren der Maps wurde die slam_toolbox von SteveMacenski verwendet. Die Generierung wurde mit dem online_async_launch.py gestartet, dass in https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py gefunden werden kann und eine Kopie in diesem Repository unter /launch/online_async_launch.py gefunden werden kann.

### Probleme AMCL

Der Parkour ist 3 Dimensional. Neben Ebenen Flächen ist auch eine Rampe enthalten. Die Positionserkennung mit AMCL ist aber für eine ebene Fläche entwickelt worden. Bei der Entwicklung mit nur einer Map sind deshalb Probleme bei der Positionserkennung aufgetreten. Zum Beispiel erkannte der Roboter, wenn er auf der Eben steht, in der Rampe und wenn er auf der Rampe steht in der ebenen eine Mauer. Um dem Abhilfe zu schaffen wurde hier für jede Rampen- oder Ebene Fläche je eine Map implementiert. Für diese Rampe ergibt es insgesamt 4. Die Rampen sind entsprechen des Roboterdurchlauf nummeriert.
Ein weiteres Problem, dass mit nur einer Map aufgetreten ist, war der Verwendete AMCL Algorithmus die Zweit Teile der Rampe nicht unterscheiden konnte. (Fläche 1+2 mit 3+4 verwechselt)

### Maps wechseln

Um die Map zu wechseln wurden zwei Möglichkeiten dies zu implementieren verglichen. Die erste Möglichkeit ist die Map in einem Map Servers zu wechseln und die andere ist, den kompletten Map Server zu wechseln.

Um eine Map für das Nav2 System bereit zu stellen wird der Map Server verwendet. Der Map Server Node stellt die Map beim Starten bereit. Mit load_map kann auch die Map während der Map Server Node schon läuft, ausgetauscht werden. Statt die Map zu tauschen kann auch statt die Map der ganze MapServer "getauscht" werden, indem die Maps Server aktiviert und deaktiviert werden. Dies ist Möglich da Map Server ein LifecycleNode ist. source https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/src/map_server/map_server.cpp line 65  Eine Liefecycle Node hat vier "primary states": Unconfigured, Inactivate, Active, Finalized. Zu Beginn ist jeder LifecyleNode in dem Unconfigured State der zu dem Inactive State transformiert werden kann. Von dem State aus kann der Node Active werden, indem die Map gepuplished wird und für andere Nodes, wie dem AMCL Node, zu Verfügung steht.
Um die Maps zu wechseln werden beim Starten ( mit amcl_4maps.launch.py) vier Map Server, einen für jede Map, gelaunched. Diese werden aber nicht vom  lifecycle_manager gemanagt, da dies nun im edu_robocup_rescue_stack_node passiert. source: https://design.ros2.org/articles/node_lifecycle.html 
Request

#### Terminal
Zum Testzwecken wurde zuerst ein Map Wechsel durch Kommandos in dem Terminal ausgeführt. Um eine neue Map zu laden kann `ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}"`ausgeführt werden. source: https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/README.md Um die Map durch aktivieren und deaktivieren der Map-Server zu tauschen, ist es nötig den aktuellen State der Nodes abzufragen und den State Wechsel triggern zu können. Um den aktuellen Lifecycle State für den Beispiel Node mit dem Namen map_server zu erhalten kann ros2 service call `/map_server/get_state lifecycle_msgs/GetState` ausgeführt werden. Um den State von Unconfigured zu Inactive zu wechseln kann `ros2 lifecycle set /map_server configure` oder `ros2 service call /lc_talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 2}}"` ausgeführt werden. source: https://index.ros.org/p/lifecycle/

#### Implementierung

##### Implementierung des Map wechsels mittels Load Map


##### Implementierung des Map Server Wechsel zu Testzwecken
Zur Implementierung des Mapwechsel mittels Lifecycle Managements wurde die Klasse `clientService` erstellt. Dort werden die Funktionen activeServices und deactiveService zu Verfügung gestellt, mit denen die Map_server aktiviert oder deaktiviert werden können. Bei der Implementierung  wurde sich an dem service_client von thehummingbird inspiriert. source: https://github.com/thehummingbird/robotics_demos/blob/main/lifecycle_node/src/demo_lifecycle/src/service_client.cpp Um den aktuellen State zu erhalten und eine State Wechsel zu triggern wird je ein asyncRequest ausgeführt. Im Gegensatz zur Implementierung von thehummingbird wurde dabei eine Callbackfunktion übergeben die das Ergebnis enthält. Um die Map zu wechseln indem eine neue Map geladen wurde die Klasse `clientService` implementiert.

##### Probleme 
Bei der Implementierung der 4 Map Server ist es zu dem Problem gekommen, dass beim Starten oft nicht alle Map Server Nodes und die AMCL Node vollständig gestartet wurden. Um dieses Problem zu lösen, wartet der Thread nun zu Beginn (im Konstruktor des LocalisationControlNode ) für 2 Sekunden. Wenn nicht gewartet wird kann es zu Fehlern führen, da auf dem Map Server zugegriffen werden kann, obwohl der Node nicht bereit ist.

#### Vergleiche 
Die Implementierung mittels MapServer Wechsel

