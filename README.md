# edu_robotcub_stack develop

## launch
ros2 launch edu_robocup_rescue_stack edu_control.launch.py 

## Team

Katarina, Jakob, Daniel

##Steuerung
Die Steuerung wurde bisher mit dem Gazebo Roboter Model eduard_offroad getestet und die Roboteransteuerung ist dem entsprechend implementiert. Um das Gazebo Roboter Model eduard_offroad zu steuern, muss eine geometry_msgs::msg::Twist unter /cmd_vel gepushed werden. In dem linear.x Wert wird die Forwärtsgeschwindigkeit und in angular.z die Drehgeschwindigkeit um den Yaw Winkeldes Roboters, übergen. 

## Orientierung und Lokalisierung
Um eine Stuerung zu implementieren ist es wichtig zu wissen an welcher Position sich der Roboter zu jeden Zeitpunkt befindet. Um die Positon des Roboters zu erhalten wurde sich für die Adaptive Monte Carlo Localization (AMCL) entschieden. Der hier verwende Algorithumus wird von Nav2 bereitgestellt. Er beötigt zur Localisation einen 2D Laserscanner und eine zuvor erstellte Map. source: https://github.com/ros-planning/navigation2/tree/main/nav2_amcl Die
## Probleme AMCL
- Der Paqour ist 3 Dimensional. Neben Ebenen Flächen ist auch eine Rampe entahlten. Die Positionserkennung mit AMCL ist aber für eine ebene Flächen entwickelt worden. Bei der Entwicklung mit nur einer Map sind deshalb Probleme bei der Positionserkennung aufgetrete. Zum Beispiel erkannte der Roboter, wenn er auf der Eben steht, in der Rampe und wenn er auf der Rampe steht in der ebenen eine Mauer. Um dem Abhilfe zu Schaffen wurde hier für jede Rampen- oder Ebenenfläche je eine Map implementiert. Für diese Rampe ergibt es insgesamt 4. Die Rampen sind entsprechen des Roboterdurchlauf nummerriert.

- Fehlerkennung der Position nachdem der Roboterüber die Schräge gefallen ist, mit der Position befor der Roboter über die Schräge gefallen ist.
