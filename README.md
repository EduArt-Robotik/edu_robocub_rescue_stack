# edu_robotcub_stack

## launch
ros2 launch edu_robocup_rescue_stack edu_control.launch.py 

## Team

Katarina, Jakob, Daniel

## Steuerung

### ALgorithmus mit Verwendung der Navigationsbibliotheken

TODO: Beschreibung von Daniel

### Selbstgeschriebener Algorithmus ohne Navigationsbibliotheken

Neben dem oben beschriebenen Algorithmus, wurde ein weiterer Algorithmus entwickelt, der (abgesehen von Verwendung von Funktionen der Lokalisierungsbibliotheken) komplett selbstgeschrieben ist und keine Funktionen der Navigationsbibliotheken verwendet. 
In diesem Algorithmus gibt es 4 Teilkarten sowie 14 navigation-steps (der Grund für die Verwendung von 4 Karten kann [hier](README.md#probleme-amcl) nachgelesen werden). Der Roboter fährt dabei sukzessive die definierten Punkte ab. Diese Punkte befinden sich auf dem Streckenmodell für die TER1 Strecke und sind fest implementiert, weshalb der Algorithmus primär nur diese Strecke eingesetzt werden kann. Die Punkte befinden sich folgendermaßen auf der Rampe:

![Rampe Punkte](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/4MapsDestPos.jpg?raw=true "Rampe mit von dem Algorithmus verwendet Punkten")

Die Roboter fährt dabei die Punkte der Reihe nach ab. Zu bemerken ist, dass Punkt 2 im Vergleich zu Punkt 7 versetzt ist und Punkt 3 im Vergleich zu Punkt 8. Damit wurde sichergestellt, dass der Roboter beim Überqueren der Rampe weder auf dem Hinweg (Punkt 2 zu 3) noch auf dem Rückweg (Punkt 7 zu 8) gegen eine Wand fährt sondern etwas weiter rampen-aufwärts die Verschränkung überquert und dann leicht nach unten fällt.

![Rampe Maps](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/4MapsMaps.jpg?raw=true "Rampe mit Maps")

Maps:
1. Map: grün,
2. Map: rot
3. Map: blau
4. Map: gelb

Navigationsteps:
0. Step: fahre auf Punk 1 zu, die Fahrtrichtung ist forwärts und Map 1 ist geladen
1. Step: fahre auf Punkt 2 zu, beobachte pitch-Winkel und wechsle Zustand, wenn der Roboter auf die erste Rampe gefahren ist
2. Step: lade Map2 und fahre auf Punkt 2 zu
3. Step: fahre auf Punkt 3 zu, beobachte pitch und roll-Winkel wechsle Zustand, wenn der Roboter über die Klippe gefahren ist.
4. Step: lade Map3 und fahre auf Punkt 4 zu. Beobachte roll und Pitch winkel um Zustand zu wechseln, wenn Roboter von der Rampe auf, den geraden Teil fährt.
5. Step: lade Map4 und fahre weiter auf Punkt 4 zu
6. Step: fahre auf Punkt 5 zu.
7. Step: ändere Fahrrichtung Rückwärts und fahre auf Punkt 6 zu.
8. Step: fahre auf Punkt 7 zu, beobachte pitch-Winkel um zustand zu Wechseln, wenn Roboter auf Rampe fährt
9. Step: lade Map3 und fahre weiter auf Punkt 7 zu
10. Step: Fahre auf Punkt 2 zu, beobachte pitch und roll-Winkel wechsel Zustand, wenn der Roboter über die Klippe gefahren ist.
11. Step: lade Map2 und fahre auf Punkt 1 zu. Beobachte roll und Pitch winkel um Zustand zu wechseln, wenn Roboter von der Rampe auf, den geraden Teil fährt.
12. Step: lade Map1 und fahre weiter auf Punkt 1 zu.
13. Step: fahre auf den Startpunkt zu.

Der Algorithmus geht davon aus, dass der Roboter zu Beginn auf einen fest definierten Startpunkt (Streckenanfang) gesetzt wurde. Der Algorithmus besteht im Wesentlichen aus zwei Zustände: Drehen (auf der Stelle) und Fahren (geradeaus vorwärts). Außerdem wird kontinuierlich (unabhängig vom Zustand) geprüft, ob der Roboter in Richtung des aktuellen Zielpunktes gedreht ist. Sollte dies nicht der Fall sein, stoppt der Roboter und beginnt sich zu drehen.

Drehen:
Bei einer Drehung dreht sich der Roboter so weit, bis er gerade auf diesen nächsten Punkt zufahren kann. Er stoppt mit der Drehbewegung, wenn er sich innerhalb eines Toleranzbereiches des anfangs für die Drehung berechneten Winkels befindet. Es wurde ein Toleranzbereich festgelegt, der einerseits möglichst genau ist, allerdings nicht zu fein, da die Winkeldifferenz nur in einem festdefinierten Takt berechnet wird und sich der Roboter deshalb möglichst wenig über das Ziel hinausdrehen soll. Diese Toleranz wurde durch Tests in der Simulation ermittelt. Während der Drehung wird der Roboter vom Algorithmus auf der Stelle gedreht und legt keinen Weg mit der Drehung zurück. Dadurch bleibt der Wendekreis sehr klein und der Roboter läuft nicht in Gefahr, ungewollt gegen eine Wand zu fahren.

Fahren:
Nachdem der Roboter in die jeweils richtige Richtung gedreht ist, beginnt er mit der Vorwärtsfahrt.  Je näher der Roboter dem aktuellen Zielpunkt kommt, desto langsamer fährt er. Sobald sich der Roboter in einem bestimmten Toleranzradius um den aktuellen Zielpunkt befindet, wird der Zielpunkt aktualisiert und der Robotor beginnt auf den nächsten Zielpunkt zuzufahren. 

Besonderheit beim Überqueren der Rampenverschränkung:
Der Roboter benötigt für diesen Algorithmus die Odometrie Daten aus den Lokalisierungsbibliotheken. Auf dem Weg von Punkt 2 zu Punkt 3 fährt der Roboter solange, bis er die Kante überfahren hat und nach unten kippt. Dies merkt der Algorithmus, weil dann der Pitch-Winkel größer oder gleich 0,1 und der Roll-Winkel größer als 0 ist.
Dies ermöglicht es, dass der Roboter in einem optimalen Winkel die Rampen wechselt. 

Implementierungsdetails:
Der Algorithmus wurde bisher mit dem Gazebo Roboter Model eduard_offroad getestet und die Implementation ist speziell darauf zugeschnitten. Um das Gazebo Roboter Model eduard_offroad zu steuern, muss eine geometry_msgs::msg::Twist unter /cmd_vel gesendet werden. In dem linear.x Wert wird die Forwärtsgeschwindigkeit und in dem angular.z Wert die Drehgeschwindigkeit um den Yaw Winkel des Roboters übergeben.

Probleme bei der Simulation:
Der Algorithmus hat prinzipiell funktioniert, solange es keine Probleme mit der Lokalisierung gab. Im Durchschnitt gab es jeden 3 Durchlauf an einer Stelle ein Problem mit der Lokalisierung, die dem Roboter eine Weiterfahrt nicht mehr möglich gemacht hat. 
Zusätzlich sind die physikalischen Eigenschaften des Robotermodells noch nicht komplett ausgereift, weshalb der Roboter bei der Rampenüberquerung unter Umständen in eine instabile Lage gekippt ist und z.B. gehüpft oder ganz auf die Seite gekippt ist.


## Orientierung und Lokalisierung

Beide oben genannten Algorithmen benötigen für die Steuerung des Roboters dessen aktuelle Position. Um die Position zu erhalten, wurde sich für die Adaptive Monte Carlo Localization (AMCL) entschieden. Der für die Lokalisierung verwendete Algorithmus wird von der open-source Bibliothek Nav2 der Navigation Community bereitgestellt. Er benötigt einen 2D Laserscanner und eine zuvor erstellte Map. Der Laserscanner wurde dafür in dem Gazebo Robotermodell hinzugefügt. [1]: https://github.com/ros-planning/navigation2/tree/main/nav2_amcl "Github nav2_amcl"

### Map Generieren 

Zum Generieren der Maps wurde die [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
von SteveMacenski verwendet. Die Generierung wurde mit dem online_async_launch.py gestartet, dass in https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py gefunden werden kann und eine Kopie in diesem Repository unter /launch/online_async_launch.py liegt.

### Probleme AMCL

Der Parkour ist dreidimensional. Neben ebenen Flächen ist auch eine Rampe enthalten. Die Positionserkennung mit AMCL ist aber nur für eine ebene Fläche entwickelt worden. Bei der Entwicklung eines Robotersteuerungsalgorithmus mit nur einer Map sind deshalb Probleme bei der Positionserkennung aufgetreten. Zum Beispiel erkannte der Roboter, wenn er auf der Ebene steht die Rampe als Wand und wenn er auf der Rampe steht Ebene als Wand. Um dem Abhilfe zu schaffen wurde hier für beide Rampen und beide ebene Fläche je eine Map implementiert.
Außerdem konnte der verwendete AMCL-Algorithmus durch die Symmetrie der Strecke nicht unterscheiden, auf welchem Teil der Strecke er sich befindet. 

### Maps wechseln

Um die Map zu wechseln wurden zwei Implementationsmöglichkeiten verglichen. Eine neue Map kann entweder in einem Map Servers geladen werden und es kann für jede Map einen komplett neuen Map Server gestartet werden. Beide Möglichkeiten haben funktioniert.

Um eine Map für das Nav2 System bereit zu stellen, wird der Map Server verwendet. Der Map Server Node stellt die Map beim Starten bereit. Mit load_map kann auch die Map ausgetauscht werden, während der Map Server Node schon läuft. Statt die Map zu tauschen kann wie oben erwähnt auch der ganze Map Server "getauscht" werden, indem die Map Server aktiviert und deaktiviert werden. Dies ist möglich, da ein Map Server ein LifecycleNode ist. [Quelle](https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/src/map_server/map_server.cpp) line 65 
Eine Liefecycle Node hat vier "primary states": Unconfigured, Inactivate, Active, Finalized. Zu Beginn ist jeder LifecyleNode in dem Unconfigured State, der zu dem Inactive State transformiert werden kann. Von dem State aus kann der Node Active werden, indem die Map gepuplished wird und für andere Nodes - wie zum Beispiel dem AMCL Node - zu Verfügung steht.
Um die Maps zu wechseln werden beim Starten (mit amcl_4maps.launch.py) vier Map Server gelaunched, einen für jede Map. Diese werden aber nicht vom lifecycle_manager gemanagt, da dies nun im edu_robocup_rescue_stack_node passiert. source: https://design.ros2.org/articles/node_lifecycle.html 
Request

#### Terminal
Zum Testzwecken wurde zuerst ein Map Wechsel durch Kommandos in dem Terminal ausgeführt. Um eine neue Map zu laden kann `ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}"`ausgeführt werden. source: https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/README.md. Um die Map durch aktivieren und deaktivieren der Map-Server zu tauschen, ist es nötig den aktuellen State der Nodes abzufragen und den State Wechsel triggern zu können. Um den aktuellen Lifecycle State für den Beispiel Node mit dem Namen map_server zu erhalten kann ros2 service call `/map_server/get_state lifecycle_msgs/GetState` ausgeführt werden. Um den State von Unconfigured zu Inactive zu wechseln kann `ros2 lifecycle set /map_server configure` oder `ros2 service call /lc_talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 2}}"` ausgeführt werden. source: https://index.ros.org/p/lifecycle/

#### Implementierung

##### Implementierung des Map wechsels mittels Load Map


##### Implementierung des Map Server Wechsel zu Testzwecken
Zur Implementierung des Mapwechsel mittels Lifecycle Managements wurde die Klasse `clientService` erstellt. Dort werden die Funktionen activeServices und deactiveService zu Verfügung gestellt, mit denen die Map_server aktiviert oder deaktiviert werden können. Die Implementierung  wurde an dem service_client von thehummingbird angelehnt. source: https://github.com/thehummingbird/robotics_demos/blob/main/lifecycle_node/src/demo_lifecycle/src/service_client.cpp Um den aktuellen State zu erhalten und eine State Wechsel zu triggern wird je ein asyncRequest ausgeführt. Im Gegensatz zur Implementierung von thehummingbird wurde dabei eine Callbackfunktion übergeben die das Ergebnis enthält. Um die Map zu wechseln indem eine neue Map geladen wurde die Klasse `clientService` implementiert.

##### Probleme 
Bei der Implementierung der 4 Map Server ist es zu dem Problem gekommen, dass beim Starten oft nicht alle Map Server Nodes und die AMCL Node vollständig gestartet wurden. Um dieses Problem zu lösen, wartet der Thread nun zu Beginn (im Konstruktor des LocalisationControlNode ) für 2 Sekunden. Wenn nicht gewartet wird kann es zu Fehlern führen, da auf dem Map Server zugegriffen werden kann, obwohl der Node nicht bereit ist.

#### Vergleiche 
Die Implementierung mittels MapServer Wechsel
