# edu_robotcub_stack

## launch
ros2 launch edu_robocup_rescue_stack edu_control.launch.py 

## Team

Katarina, Jakob, Daniel

## Einleitung / Aufgabenstellung

Ein Roboter soll autonom einen vordefinierten Parqour ([TER 1)Sand/Grave](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf)) so oft und so schnell wie möglich durchqueren. Wenn der Roboter am Ende des (TER 1)Sand/Grave Parqour angekommen ist, soll der Roboter Rückwärts (ohne eine 180° Drehung) zurückfahren. Der Roboter muss zuerst ein flaches Element überfahren, dann ein Rampe hinauffahren, auf eine andere Rampe gelangen, diese dann wieder hinunterfahren und zuletzt auf einem flaches Element bis zu dessen Ende fahren. Eine besondere Schwierigkeit ist dass Überqueren der Rampe.

## Orientierung und Lokalisierung
Um den Roboter durch den Parqour zu Steuern wird zu jedem Zeitpunkt die aktuelle Position benötigt. Um die Position zu erhalten, wurde sich für die Adaptive Monte Carlo Localization (AMCL) entschieden. !Mehr infos über Amcl!.Der für die Lokalisierung verwendete Algorithmus wird von der open-source Bibliothek Nav2 der Navigation Community bereitgestellt. Er benötigt einen 2D Laserscanner und eine zuvor erstellte Map. Der Laserscanner wurde dafür in dem Gazebo Robotermodell hinzugefügt. [source: Github nav2_amcl](https://github.com/ros-planning/navigation2/tree/main/nav2_amcl)

### Map Generieren 

Zum Generieren der Maps wurde die [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
von SteveMacenski verwendet. Die Generierung wurde mit dem [online_async_launch.py](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py) gestartet. Eine [Kopie](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py) des Scource Codes liegt in diesem Repository.

### Probleme AMCL

Der Parkour ist dreidimensional. Neben ebenen Flächen ist auch eine Rampe enthalten. Die Positionserkennung mit AMCL ist aber nur für eine ebene Fläche entwickelt worden. Bei der Entwicklung eines Robotersteuerungsalgorithmus mit nur einer Map sind deshalb Probleme bei der Positionserkennung aufgetreten. Zum Beispiel erkannte der Roboter, wenn er auf der Ebene steht die Rampe als Wand und wenn er auf der Rampe steht Ebene als Wand. Um dem Abhilfe zu schaffen wurde hier für beide Rampen und beide ebene Fläche je eine Map implementiert.
Außerdem konnte der verwendete AMCL-Algorithmus durch die Symmetrie der Strecke nicht unterscheiden, auf welchem Teil der Strecke er sich befindet. 

### Maps wechseln

Um die Map zu wechseln wurden zwei Implementationsmöglichkeiten verglichen. Eine neue Map kann entweder in einem Map Servers geladen werden und es kann für jede Map einen komplett neuen Map Server gestartet werden. Beide Möglichkeiten haben funktioniert.

Um eine Map für das Nav2 System bereit zu stellen, wird der Map Server verwendet. Der Map Server Node stellt die Map beim Starten bereit. Mit load_map kann auch die Map ausgetauscht werden, während der Map Server Node schon läuft. Statt die Map zu tauschen kann wie oben erwähnt auch der ganze Map Server "getauscht" werden, indem die Map Server aktiviert und deaktiviert werden. Dies ist möglich, da ein Map Server ein LifecycleNode ist. [source: Map_server.cpp line 65](https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/src/map_server/map_server.cpp)

Eine Liefecycle Node hat vier "primary states": Unconfigured, Inactivate, Active, Finalized. Zu Beginn ist jeder LifecyleNode in dem Unconfigured State, der zu dem Inactive State transformiert werden kann. Von dem State aus kann der Node Active werden, indem die Map gepuplished wird und für andere Nodes - wie zum Beispiel dem AMCL Node - zu Verfügung steht.
Um die Maps zu wechseln werden beim Starten (mit amcl_4maps.launch.py) vier Map Server gelaunched, einen für jede Map. Diese werden aber nicht vom lifecycle_manager gemanagt, da dies nun im edu_robocup_rescue_stack_node passiert. [source](https://design.ros2.org/articles/node_lifecycle.html)


#### Terminal
Zum Testzwecken wurde zuerst ein Map Wechsel durch Kommandos in dem Terminal ausgeführt. Um eine neue Map zu laden kann `ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}"`ausgeführt werden. [source](https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/README.md) 

Um die Map durch aktivieren und deaktivieren der Map-Server zu tauschen, ist es nötig den aktuellen State der Nodes abzufragen und den State Wechsel triggern zu können. Um den aktuellen Lifecycle State für den Beispiel Node mit dem Namen map_server zu erhalten kann ros2 service call `/map_server/get_state lifecycle_msgs/GetState` ausgeführt werden. Um den State von Unconfigured zu Inactive zu wechseln kann `ros2 lifecycle set /map_server configure` oder `ros2 service call /lc_talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 2}}"` ausgeführt werden. [source](https://index.ros.org/p/lifecycle/)

### Implementierung des Map Wechsel

##### Implementierung des Map wechsels mittels Load Map

Um einen Map mittels Load Map zu wecheln wird ein Client kreiert, mithilfe dessen an dem Map Server ein asyncRequest mit der gewünschten Rampe geschickt wird. Neben der gewünschten Rampe wird eine callback Funktion übergeben, durch die ein erfolgreiecher Wechsel festgestellt werden kann.

##### Implementierung des Map Server Wechsel zu Testzwecken
Zur Implementierung des Mapwechsel mittels Lifecycle Managements wurde eine neue Klasse erstellt. Dort werden die Funktionen activeServices und deactiveService zu Verfügung gestellt, mit denen die Map_server aktiviert oder deaktiviert werden können. Die Implementierung  wurde an dem service_client von thehummingbird angelehnt. [source](https://github.com/thehummingbird/robotics_demos/blob/main/lifecycle_node/src/demo_lifecycle/src/service_client.cpp)

Um den aktuellen State zu erhalten und eine State Wechsel zu triggern wird je ein asyncRequest gesendet. Im Gegensatz zur Implementierung von thehummingbird wurde dabei eine Callbackfunktion übergeben, durch die eine Rückmeldung über den erfolg/Scheitern, des wechsel der Lifecycle Node zusände übergeben wird.

###### Probleme bei der Implementierung der 4 Map Server
Bei der Implementierung der 4 Map Server ist es zu dem Problem gekommen, dass beim Starten oft nicht alle Map Server Nodes und die AMCL Node vollständig gestartet wurden. Um dieses Problem zu lösen, wartet der Thread nun zu Beginn (im Konstruktor des LocalisationControlNode ) für 2 Sekunden. Wenn nicht gewartet wird kann es zu Fehlern führen, da auf dem Map Server zugegriffen werden kann, obwohl der Node nicht bereit ist.

#### Vergleiche 
Durch beide Implementierung kann die Map gewechselt werden. Die Implementierung mittels MapServer Wechsel ist deutlich aufwändiger als die Implementierung Load Map. Deshalb wurde sich Letztendlich für die Implementierung mittels Load Map entschieden.

## Steuerung

### ALgorithmus mit Verwendung der Navigationsbibliotheken

#### Navigation:

##### Ablauf:

Wie bereits erklärt, bietet die auf zwei Dimensionen basierende Positionserkennung mit AMCL nicht die Möglichkeit einer kontinuierlichen Lokalisierung über den gesamten drei-dimensionalen Kurs. Aufgrund dessen ist abhängig von der aktuellen Ebene (Gerade 1, Rampe 1, Rampe 2 oder Gerade 2) eine spezifische Karte zu laden in der die Roboter-Position jeweils neu zu initialisieren ist. 

![Strecke: Ter0_Ramp](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/bev_strecke.png)

Dem Roboter ist zudem ein neues Ziel vorzugeben, welches dieser in der Karte anzufahren hat. Als vorteilhafter Ablauf hat sich hier ergeben zu Beginn die Ebenen-spezifische Karte zu laden, anschließend die Ziel-Position zu senden und zuletzt die Roboter-Pose zu initialisieren. 

Intuitiv würde man die Ziel-Position erst nach der Initialisierung der Roboter-Position an den Pfad-Planer senden, da der Roboter jedoch für eine genaue Bestimmung der Initialisierungs-Position möglichst keinen Pitch- und Roll-Winkel besitzen sollte, hat das Programm zu warten, bis dieser nach dem Sprung über die Rampe wieder gerade steht. Die Wartezeit ist auf 1.0 Sekunde eingestellt. Das Programm prüft im Anschluss den Stand des Roboters. Das sofortige Vergeben des neuen Zieles verhindert, dass der Roboter sich in der Zwischenzeit wieder zurück zum vorherigen Ziel (alte Karte) orientiert und ermöglicht stattdessen, dass er sich direkt in die richtige Richtung (neues Ziel) bewegt.

Aufgrund unterschiedlicher Winkel-Konstellationen sowie der aktuellen X/Y-Postion lässt sich die gegenwärtig befahrene Ebene identifizieren. Der Registrierung einer Änderung der Ebene folgt das Laden einer neuen Karte, die Navigation zu einem neuen, abhängig von der Ebene und der derzeitigen Bewegungsrichtung gewählten Ziel sowie die Initialisierung der neuen Roboter-Pose. Dieser Ablauf ist für jede Karte identisch. 

##### Ziel-Position: 

Für Hin- und Rückweg wäre jeweils eine vorgegebene Ziel-Position (inkl. Orientierung) pro Karte ausreichend. Aufgrund der Tendez des Pfad-Planers für das Ziel auf den Rampen einen Pfad zu berechnen, der sehr nahe an Hindernissen bzw. den Kollisionsbereichen der Costmap entlang verläuft und deshalb teilweise zu problematischen Situationen geführt hat, sind auf den Rampen nun für Hin- und Rückweg jeweils zwei Ziele vergeben. Auf den Geraden ist jeweils ein Ziel ausreichend.

##### Initialisierungs-Position:

Die Bestimmung der X/Y-Position erfolgt mittels der durch den Laser-Scanner gemessenen Abstände bestimmter Laser-Strahlen (Gerade aus nach vorne, gerade aus nach hinten, rechts und links) relativ zu den Wänden der Hindernisstrecke. Abhängig von der aktuellen Position auf der Strecke finden unterschiedliche Wände Verwendung zur Abstandsmessung und somit zum Erhalt der Roboter-Position. Bevorzugt sind Wände die eher einen senkrechten Winkel zur Ebene auf dem sich der Roboter befindet, aufzeigen. Dies erlaubt eine Minimierung des Fehlers, den der Roboter aufgrund seiner Eigenbewegung um Pitch- und Roll-Winkel erzeugt. 

Gerade 1: 	Wand rechts & Wand gerade aus nach hinten  
Rampe 1:	Wand rechts & Wand gerade aus nach vorne  
Rampe 2: 	Wand links & Wand gerade aus nach hinten  
Gerade 2: 	Wand links & Wand gerade aus nach vorne  

Abhängig von dem Origin-Punkt der Karte (siehe [Karten-Konfigurations-Datei](map/map_8.3.yaml)) wird auf den gemessenen Abstand ein Offset aufaddiert, um die echte Pose relativ zum map_frame, also dem Globalen-Koordinatensystem zu erhalten.

Die Ermittlung der Laser-Strahlen, die unabhängig der aktuellen Orientierung des Roboters in die notwendige Richtung (senkrecht zur Wand) zeigen, erfolgt mittels des Yaw-Winkels der IMU. Die IMU stellt darüber hinaus die Informationen der Orientierung (X, Y, Z, W) des Roboters für die Initialisierungs-Pose zur Verfügung.

#### ROS Navigation 2:

Die eigentliche Navigation durch den Kurs basiert auf dem ROS Navigation Stack 2 (Nav2). Ein Pfad-Planer abonniert die publizierten Ziel-Positionen und plant daraufhin einen Pfad durch eine Costmap hin zum Ziel. Da aufgrund großer Höhenunterschiede zwischen einzelnen Bereichen der beiden Rampen das Fahren von der einen auf die andere Rampe nicht überall unfallfrei möglich ist, sorgt ein „Keepout“-Filter über der Costmap dafür, dass der Übergang an einer sicheren Stelle stattfindet. Ist ein valider Pfad gefunden, steuert ein „FollowPath“-Plugin die Roboterbewegung entlang des Pfades. Ein „Behaviour-Tree“ regelt das Regenerierungsverhalten des Roboters in Sonderfällen, wie dem Abkommen vom Pfad oder dem Steckenbleiben des Roboters.

##### Pfad-Planer:

Da für die Durchläufe des Robocups die Zeit eine große Rolle spielt, ist die Pfad-Planung so auszulegen, dass eine Rückwärtsfahrt des Roboters möglich ist und die Zeit für das umdrehen auf dem engen Kurs eingespart werden kann. Diese Funktion bieten nur zwei Pfad-Planer in der Nav2 Funktionsbibliothek. Der Smac Hybrid Planner (Hybrid-A* Planer) und der Smac Lattice Planer (State Lattice Planer). Da sich der Smac Hybrid Planner nicht für differential angetriebene Roboter eignet fiel die Wahl auf den Smac Lattice Planner.  

Der Smac Lattice Planner baut auf einer Gitter-basierten Datenstruktur names „Sparse-map“ auf. Die Möglichkeit des Roboters sich stufenlos um die Hochachse zu drehen, sorgt dafür, dass bei bestimmten Drehwinkeln gerade Trajektorien nicht auf Endpunkten landen, die nicht auf das Gitter ausgerichtet sind. Um diesem Problem entgegen zu wirken generiert ein individuell parametrierbarer [„Lattice Primitiv Generator“](https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner/lattice_primitives) im Voraus Standard Trajektorien. Dazu diskretisiert dieser die möglichen Winkel der Bewegungsrichtungen des Roboters so , dass gerade Trajektorien auf Endpunkten landen, die mit den diskreten Rasterpunkten des Gitters übereinstimmen. Das sogenannte „Minimum Control Set“, welches diese Trajektorien enthält, ist für jeden Roboter individuell mittels des „Lattice Primitiv Generators“ manuell zu erzeugen. [source](https://navigation.ros.org/configuration/packages/smac/configuring-smac-lattice.html)


##### Controller-Server

Der Controller-Server ist ein Task-Server, welcher zur Ausführung von Navigationsaufgaben dient, die aus mehreren Schritten und Aktionen bestehen. Er implementiert das Pfad-Folge-Plugin, welches die Befehlsgeschwindigkeiten zum Folgen der vom Pfad-Planer erzeugten Trajektorie generiert. Die Nav2 Funktionsbibliothek enthält mehrere [Pfad-Folge-Plugins](https://navigation.ros.org/plugins/index.html), die jedoch zum Großteil nur eine Zielverfolgung bei Vorwärtsfahrt des Roboters erlauben und sich entsprechend entweder im Stand zum Ziel drehen oder noch zeitintensiver in mehreren Zügen wenden. Da, wie bereits unter dem Punkt „Path-Planner“ beschrieben, diese Zeit einzusparen ist, verwendet die Arbeit den Regulated Pure Pursuit Controller, welcher eine Trajektorienverfolgung bei Rückwärtsfahrt erlaubt. [source](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)

Der Verfolgungsalgorithmus besitzt die Fähigkeit die Geschwindigkeit basierend auf Kollision, Trajektorienkrümmung oder hohen Kosten in der Costmap zu regulieren. In der Praxis hat sich jedoch gezeigt, dass diese Regulierungsfunktionen in diesem Anwendungsfall kontraproduktiv sind und der Roboter dadurch eher dazu geneigt ist auf dem Kurs stecken zu bleiben. Ein Beispiel aus der Praxis ist, dass der Roboter um von einer geraden Ebene auf eine Rampe zu gelangen eine Ziel-Position anfahren muss, die sich am Ende der Karte und somit nahe des Inflation-Layers (Hohe Kosten) befindet. Erhält der Roboter regulierte Geschwindigkeitsbefehle, so reduziert sich die Geschwindigkeit, sodass er zu langsam ist, um in einem Zug auf die Rampe aufzufahren und somit kurz hängen bleibt. Dadurch kann ein Drift in der Lokalisierung entstehen, der sich negativ auf die weiterfahrt auswirkt. Aus diesem Grund sind die Regulierungen ausgeschaltet. [source](https://navigation.ros.org/configuration/packages/configuring-regulated-pp.html) 

Neben dem Pfad-Folge-Plugin implementiert der Controller-Server Fortschrittskontroll- und Zielkontroll-Plugins. Dafür sind die Nav2 Standard-Plugins SimpleProgressChecker und SimpleGoalChecker im Einsatz. Über den Stateful-Parameter des GoalCheckers lässt sich bestimmen, ob dieser den Fortschritt des Roboters über mehrere Planungsdurchläufe speichert. Da die Speicherung nur einen Vorteil bei dynamischen Umgebungen erbringt und sonst nur Rechenleistung verbraucht, ist diese Funktion ausgeschaltet. [source](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)


##### Costmap

Basis der Costmap bilden die von der slam_toolbox erzeugten zwei-dimensionalen Rasterkarten. Mithilfe verschiedener Layer und Filter integriert die Costmap die Informationen des Laser-Scanners in die Karte. Dies erlaubt die Darstellung der realen dynamischen Umgebung des Roboters. Basierend auf den in der Costmap enthaltenen Informationen plant der Pfad-Planer die Trajektorien. [source](https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d)

Obwohl es sich beim SmacLatticePlanner um einen globalen Pfad-Planer handelt, besteht die Möglichkeit neben einer globalen costmap zusätzlich eine lokale costmap mit einzubeziehen, wodurch eine Verbesserung in der Genauigkeit der Trajektorien erreicht wurde. 

Die globale costmap dient der langfristigen Planung durch die gesamte Karte und gibt eine grobe Übersicht über die gesamte Umgebung des Roboters. Aufgrund der Größe des Planungsbereiches und somit der zu analysierenden Umgebung, die die globale Costmap abdeckt besitzt sie eine langsame Ausführungs- und Aktualisierungszeit, weshalb in diesem Fall nur Layer und Filter integriert sind, welche sich auf statische Hindernisse beziehen. Verwendet ist ein inflation-layer, ein static-layer und ein keepout-filter. 

Der inflation-layer ist der wichtigste layer in jeder costmap. Dabei handelt es sich um eine „Hindernis-Vergrößerungsschicht“ oder „Aufblähschicht“ in der Karte, die dazu dient Hindernisse so aufzublähen, dass ein Sicherheitsabstand um sie herum entsteht. Dieser Sicherheitsabstand ist manuell  durch den Parameter „inflation-radius“ einzustellen. Das Aufblähen der Hindernisse ist notwendig, da der Pfad-Planer den Roboter bei der Trajektorienberechnung vereinfacht als Punktmodell betrachtet, was wiederum zur Vernachlässigung der tatsächlichen Ausmaße führt. Der Pfad-Planer behandelt den aufgeblähten Bereich der Hindernisse als Kollisionsbereich und setzt die Trajektorien so an, dass keine Kollisionen mit den Rädern des Roboters entstehen. Allgemein empfiehlt es sich die Parameter „infaltion-radius“ und „cost-scaling-factor“ groß zu halten, da dies den Pfad-Planer eher dazu drängt freie Flächen in der Karte zu befahren. Es hat sich jedoch in der Praxis und speziell auf dieser Strecke gezeigt, dass sich der Roboter bei einem großen „inflation-radius“ nach dem Sprung oftmals innerhalb des Kollisionsbereiches im „inflation-layer“ befindet. Steht der Roboter in diesem Bereich, kann der Planer keine gültige Trajektorie zur Ziel-Position planen und der Roboter bleibt stehen. Aus diesem Grund ist der „inflation-radius“ klein gehalten. Als grobe Orientierung ist der Radius des Roboters verwendet. [source](https://navigation.ros.org/configuration/packages/costmap-plugins/inflation.html)

Der static-layer enthält Informationen über die Position und Form bekannter und statischer Hindernisse, die im Zuge der Erstellung der Karten mit der slam_toolbox registriert wurden. [source](https://navigation.ros.org/configuration/packages/costmap-plugins/static.html)

Die Verwendung eines Keepout-Filters ist auf dieser Strecke essentiell, wenn die Steuerung des Roboters durch einen Pfad-Planer erfolgt. Der Grund hierfür ist, dass der Pfad-Planer bei Planung einer geraden Trajektorie von der einen Rampe auf die Andere den Roboter über einen hohen Absatz fahren lässt, wodurch sich dieser im Normalfall überschlägt. Ein Keepout-Filter ist ein Filter mit dem der Benutzer manuell Zonen erzeugen kann, die in der Costmap als Kollisionsbereiche erscheinen, wodurch der Pfad-Planer diese Zonen bei der Erstellung der Trajektorie meidet.

###### Keepout-Filter:

![keepout_filter](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/keepout_filter_90.png)

###### Globale-Costmap mit überlagertem Keepout-Filter:

![global_costmap](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/global_costmap.png)

Wie im Bild des Keepout-Filters oben zu erkennen ist, ist die Gestaltung so gewählt, dass dem Pfad-Planer nur ein sehr kleiner Bereich zur Verfügung steht, durch den er die Trajektorie planen kann. Dieser Bereich befindet sich ein Stück hinter dem Schnittpunkt der Rampen-Ebenen, sodass der Roboter nur einen kleinen Absatz herunterfahren muss und so mühelos und sicher auf die nachfolgende Ebene gelangt. Da der Keepout-Filter fixiert ist, müsste der Roboter im Rahmen der Rückfahrt den Absatz wieder hoch fahren. Da dies in der Regel zu Problemen führt, sind die nachfolgenden Karten in Richtung der positiven X-Achse verschoben. Die Verschiebung ermöglicht die Platzierung des Keepout-Filters über der Karte genau so, dass der Roboter bei der Rückfahrt den Absatz auf der anderen Rampe herunterfährt. Die Position der Karte im Globalen-Koordinatensystem lässt sich in der Karten-Konfigurations-Datei einstellen. Die slam_toolbox erstellt die Karten-Konfigurations-Datei im Zuge der Erstellung der Karte und vergibt die Ursprungsposition dieser automatisch. Diese Ursprungsposition ist oftmals nicht korrekt und sollte bei der Generierung von Karten mit der slam_toolbox überprüft werden. Ebenfalls im Bild des Keepout-Filters oben zu sehen ist, dass dieser an beiden Seiten der Öffnung etwas bauchig geformt ist. Versuche haben gezeigt, dass sich sowohl im Hinblick auf eine stabile Fahrweise als auch auf die nachfolgende Weiterfahrt eine leicht quere Überfahrt am besten eignet. Die bauchige Form des Keepout-Filters an dieser Stelle leitet diese Querfahrt ein. Zur Erstellung des Keepout-Filters, aber auch zur Bearbeitung der Karten hat sich das Bildbearbeitungsprogramm [gimp](https://www.gimp.org/) als gut erwiesen. [source](https://navigation.ros.org/configuration/packages/costmap-plugins/keepout_filter.html)

Die lokale costmap passt die grobe Pfad-Planung, die zunächst basierend auf der globalen costmap stattgefunden hat, situativ an. Sie deckt im Vergleich zur globalen costmap (gesamte Karte) nur einen kleinen Bereich unmittelbar um den Roboter ab. Dieser Bereich enthält sehr detaillierte Echtzeit-Informationen über Hindernisse im nahegelegenen Umfeld des Roboters, wodurch eine präzisere Pfad-Planung möglich ist. Standardmäßig implementiert die lokale costmap ebenfalls einen inflation-layer sowie einen keepout-filter. Speziell für den Erhalt der Informationen über dynamische Ereignisse ist ein obstacle-layer verwendet. 

###### Lokale-Costmap:

![local_costmap](https://github.com/EduArt-Robotik/edu_robocub_rescue_stack/blob/main/docs/local_costmap.png)

Der obstacle-layer bezieht seine Informationen über die Umgebung des Roboters über den Laser-Scanner. Er erfasst und speichert die Form und Position der Hindernisse in Echtzeit und aktualisiert die damit costmap kontinuierlich, wodurch der Pfad-Planer die Trajektorie situativ anpassen kann. Obwohl der obstacle-layer sich eher für dynamische Umgebungen eignet und die Strecke in der sich der Roboter bewegt rein statisch ist, bietet die Verwendung des obstacle-layers einen wesentlichen Vorteil. [source](https://navigation.ros.org/configuration/packages/costmap-plugins/obstacle.html)

Im Moment des Sprunges von einer Rampe auf die Andere, neigt sich der Roboter teilweise so stark, dass der Laser-Scanner den Boden als unmittelbares Hindernis erfasst und dieses durch den obstacle-layer in die costmap überträgt. Dies sorgt dafür, dass der Roboter zunächst abbremst und einen Moment stehen bleibt. Das Abbremsen sorgt zum Einen für einen reduzierten Schlupf im Gegensatz zu einem stark schwingenden bis hüpfenden Weiterfahren und ermöglicht zum Anderen, dass der Roboter während der Bestimmung der Initialisierungs-Pose relativ ruhig da steht. 

#### Probleme und Fehler:

Nachfolgend sind Probleme und Fehler, die während der Test-Durchläufe in unregelmäßigen Abständen aufgekommen sind, aufgelistet und beschrieben.

##### TF_NAN_INPUT-Error

Original-Fehler:  
Error: Ignoring transform for child_frame_id “odom” from authority “Authority undetectable” because of nan value in the transform (nan nan nan) (0.000000 0.000000 -0.770181 0.637825)

Im Rahmen der Test-Durchläufe auf der Hindernis-Strecke in Gazebo ist der Fehler vermehrt aufgetreten, wenn zwei oder mehr Räder des Roboters keinen Kontakt mit der Fahrbahn hatten, wie zum Beispiel beim „Sprung“ von der einen Rampe auf die Andere oder bei zu starkem abbremsen auf der Rampe, wodurch die Hinterräder angehoben wurden.

Allgemein tritt ein TF_NAN_INPUT-Error auf, wenn ein nan (not a number)-value, also ein ungültiger bzw. nicht berechenbarer Wert, innerhalb einer Transformation auftaucht. Transformationen dienen dazu Positionen und Orientierungen von einem Koordinatensystem auf ein Anderes zu übertragen. Dynamische Transformationen, die in dieser Arbeit während der Fahrt des Roboters ausgeführt werden müssen, sind die Transformation vom odom_frame auf den base_frame, die das Roboter-Modell (.sdf-file) selbst generiert sowie die Transformation vom odom-frame auf den map-frame, welche der AMCL publiziert.

Da sich der TF_NAN_INPUT-Error auf die Odometrie bezieht, wäre eine erste mögliche Fehlerursache, dass diese während dem Verlust des Bodenkontaktes keine gültigen bzw. zuverlässigen Sensor-Daten von den Raddrehzahl-Sensoren erhält, wodurch die NAN-Werte in der Transformation auftreten. 

Parallel zum TF_NAN_INPUT-Error tritt folgender Fehler auf:  

Original-Fehler:  
[WARN] [amcl]: AMCL covariance or pose is NaN, likely due to an invalid configuration or faulty sensor measurements! Pose is not availabl!

Dieser Fehler könnte ebenfalls von der fehlerhaften Odometrie resultieren, da die Positionserkennung des AMCL’s neben den Daten des 2D-Laserscanners auch auf den Informationen der Odometrie basiert.

##### Fehlerhafte Lokalisierung

Die letztendliche Schätzung der Pose und somit die Lokalisierung des Roboters wird vom AMCL durchgeführt. Da die Positionsschätzung des AMCL’s auf der Erkennung von Merkmalen in den Sensor-Daten des 2D-Laserscanners basiert, könnte es sein, dass die symmetrische Strecke, besonders auf den Geraden 1 und 2, welche als einzige Merkmale die Wände und Ecken aufweisen, dem AMCL Schwierigkeiten bereitet. Die Erkenntnis, dass sich die Lokalisierungs-Probleme jedoch besonders auf den Rampen zeigen, die wesentlich spezifischere Merkmale aufweisen, lässt auf eine andere Hauptursache schließen. Ein Beitragen zur Verschlechterung der Lokalisierung ist trotzdem nicht auszuschließen. 

Aufgrund des großen Schlupfes, der an vielen Stellen der Strecke an den Rädern des Roboters entsteht ist davon auszugehen, dass die Odometrie, auf welcher die Positions-Schätzung des AMCL’s zu einem wesentlichen Teil basiert, eine große Ursache für eine fehlerhafte Lokalisierung ist. Besonders viel Schlupf entsteht in folgenden Situationen:

- Das Drehen des Roboters auf der Rampe im Stand in Richtung der Ziel-Position sorgt dafür, das dieser oftmals durch die gleichzeitige Rotation der Räder die Rampe etwas herunterrutscht. 

- Bei starkem Abbremsen schaukelt der Roboter aufgrund der weichen Federung teilweise stark auf und macht mehrere Sätze über die Fahrbahn. Dieses Verhalten ist überwiegend während der Talfahrt auf den Rampen zu erkennen.

- Das Herunterfahren des Absatzes von der einen Rampe auf die Andere führt gelegentlich zu Verlust des Bodenkontaktes einzelner oder aller Räder. 

→ Die Odometrie kann die Bewegung in den beschriebenen Situationen nur unzureichend oder garnicht registrieren. 

Zur Verbesserung der Lokalisierung publiziert der Algorithmus nach jedem Laden einer neuen Karte eine Initialisierungs-Position. Die Bestimmung dieser Position basiert auf dem mittels Laserscanner gemessenen Abstand zu den Wänden vor bzw. hinter und neben dem Roboter. Diese Abstandsmessung ist präzise solange der Roboter keinen Roll- und/oder Pitch-Winkel besitzt. Ist dies der Fall, so verfälscht dies die Abstandsmessung etwas und es entsteht ein systematischer Fehler.  Zum jetzigen Zeitpunkt kann der Algorithmus diesen Fehler für einen sehr kleinen Yaw-Winkel bzw. für einen Yaw-Winkel nahe Pi raus rechnen. 


##### Steckenbleiben im inflation-layer oder keepout-filter

Sowohl der inflation-layer als auch der keepout-filter sind in der costmap als Kollisionsbereiche repräsentiert. Da der „Sprung“ von der einen Rampe auf die Andere sehr unkontrolliert stattfindet, ist es vorgekommen, dass der Roboter innerhalb dieser Kollisionsbereiche landet. Ist dies der Fall, kann der Pfad-Planer keinen Pfad zur Ziel-Position berechnen und der Roboter bleibt an der Stelle stehen. Ein Ansatz zur Lösung dieses Problems wäre die Anpassung des Recovery-Modus des Roboters, welcher im  Nav2 behavour-tree definiert ist. 


### Selbstgeschriebener Algorithmus ohne Navigationsbibliotheken

Neben dem oben beschriebenen Algorithmus, wurde ein weiterer Algorithmus entwickelt, der (abgesehen von Verwendung von Funktionen der Lokalisierungsbibliotheken) komplett selbstgeschrieben ist und keine Funktionen der Navigationsbibliotheken verwendet. 
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
Der Algorithmus hat prinzipiell funktioniert, solange es keine Probleme mit der Lokalisierung gab. Die Lokalisierung hatte an den selben Stellen Probleme wie bei dem Algorithmus, der die Navigationsbibliothek verwendet(siehe [hier](README.md#probleme#localisierung)). Im Durchschnitt gab es jeden 3 Durchlauf an einer Stelle ein Problem mit der Lokalisierung, die dem Roboter eine Weiterfahrt nicht mehr möglich gemacht hat. 
Zusätzlich sind die physikalischen Eigenschaften des Robotermodells noch nicht komplett ausgereift, weshalb der Roboter bei der Rampenüberquerung unter Umständen in eine instabile Lage gekippt ist und z.B. gehüpft oder ganz auf die Seite gekippt ist.

### Vergleich der Algorithmen
