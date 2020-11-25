# behavior_tree_ros
Behavior Tree applications in ROS

### Baxter pick-and-place com FSM

Primeiro, inicie a simulação no gazebo com o Baxter:
```
  roslaunch baxter_gazebo baxter_world.launch
```
Em seguida, em um novo terminal, spawne a mesa e o bloco, habilite o robô e suba os Services necessários 
para o controle:
```
roslaunch baxter_bt start_baxter.launch
```
Para executar a Máquina de Estados:
```
rosrun baxter_bt pick_and_place_fsm.py
```
O robô deverá iniciar o pick and place utilizando Máquina de Estados. Para uma visualização gráfica da FSM, utilize o smach_viewer:
```
rosrun smach_viewer smach_viewer
```
![Baxter pick-and-place com FSM.](/img/baxter_fsm.jpeg "Baxter pick-and-place com FSM.")


### Baxter pick-and-place com BT
Inicie o mundo da simulação com os Services:
```
  roslaunch baxter_gazebo baxter_world.launch
  roslaunch baxter_bt start_baxter.launch
```
Para executar a Árvore de Estados:
```
rosrun baxter_bt pick_and_place_bt
```
O robô deverá iniciar o pick and place utilizando Árvore de Estados. Para uma visualização gráfica da BT, utilize o Groot:
```
cd
./groot
```
![Baxter pick-and-place com FSM.](/img/groot.jpeg "Baxter pick-and-place com FSM.")

Clique em "Monitor" e em "Start", para monitorar o funcionamento da BT.
![BT Tree.](/img/groot.jpeg "BT Tree.")
