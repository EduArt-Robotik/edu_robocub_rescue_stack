# edu_robotcub_stack develop

## launch
ros2 launch edu_robocup_rescue_stack edu_control.launch.py 

## Team

Katarina, Jakob, Daniel

## Steuerung
Die Steuerung wurde bisher mit dem Gazebo Roboter Model eduard_offroad getestet und die Roboteransteuerung ist dem entsprechend implementiert. Um das Gazebo Roboter Model eduard_offroad zu steuern, muss eine geometry_msgs::msg::Twist unter /cmd_vel gepushed werden. In dem linear.x Wert wird die Forwärtsgeschwindigkeit und in angular.z die Drehgeschwindigkeit um den Yaw Winkel des Roboters, übergeben. 

## Orientierung und Lokalisierung
Um eine Steurung zu implementieren ist es wichtig zu wissen an welcher Position sich der Roboter zu jeden Zeitpunkt befindet. Um die Position des Roboters zu erhalten wurde sich für die Adaptive Monte Carlo Localization (AMCL) entschieden. Der hier verwende Algorithmus wird von Nav2 bereitgestellt. Er benötigt zur Localisation einen 2D Laserscanner und eine zuvor erstellte Map. source: https://github.com/ros-planning/navigation2/tree/main/nav2_amcl 

### Probleme AMCL
- Der Parkour ist 3 Dimensional. Neben Ebenen Flächen ist auch eine Rampe enthalten. Die Positionserkennung mit AMCL ist aber für eine ebene Fläche entwickelt worden. Bei der Entwicklung mit nur einer Map sind deshalb Probleme bei der Positionserkennung aufgetreten. Zum Beispiel erkannte der Roboter, wenn er auf der Eben steht, in der Rampe und wenn er auf der Rampe steht in der ebenen eine Mauer. Um dem Abhilfe zu schaffen wurde hier für jede Rampen- oder Ebenefläche je eine Map implementiert. Für diese Rampe ergibt es insgesamt 4. Die Rampen sind entsprechen des Roboterdurchlauf nummeriert.
Ein weiters Problem das mit nur einer Map aufgetreten ist, war der Verwendete AMCL Algorithmus die Zweit Teile der Rampe nicht unterscheiden konnte. (Fläche 1+2 mit 3+4 verwechselt)