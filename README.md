# edu_robotcub_stack develop

## launch
ros2 launch edu_robocup_rescue_stack edu_control.launch.py 

## Team

Katarina, Jakob, Daniel

## Orientierung und Lokalisierung

## Probleme AMCL
- Der Paqour ist 3 Dimensional. Neben Ebenen Flächen ist auch eine Rampe entahlten. Die Positionserkennung mit AMCL ist aber für eine ebene Flächen entwickelt worden. Bei der Entwicklung mit nur einer Map sind deshalb Probleme bei der Positionserkennung aufgetrete. Zum Beispiel erkannte der Roboter, wenn er auf der Eben steht, in der Rampe und wenn er auf der Rampe steht in der ebenen eine Mauer. Um dem Abhilfe zu Schaffen wurde hier für jede Rampen- oder Ebenenfläche je eine Map implementiert. Für diese Rampe ergibt es insgesamt 4. Die Rampen sind entsprechen des Roboterdurchlauf nummerriert.

- Fehlerkennung der Position nachdem der Roboterüber die Schräge gefallen ist, mit der Position befor der Roboter über die Schräge gefallen ist.
