# behavior_tree_ros

## Instalando pacotes e dependências
#### Behavior Tree
Instale algumas dependências:
``` 
sudo apt-get install libzmq3-dev libboost-dev
```
Para instalar a biblioteca [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP):
```
git clone https://github.com/BehaviorTree/BehaviorTree.CPP
cd BehaviorTree.CPP
mkdir build; cd build
cmake .. && make
sudo make install
```

Para utilizar a biblioteca em um projeto, o arquivo **CMakeLists.txt** deverá ser parecido com isso:
```cmake
cmake_minimum_required(VERSION 3.5)

project(hello_BT)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(BehaviorTreeV3)

add_executable(${PROJECT_NAME} "hello_BT.cpp")
target_link_libraries(${PROJECT_NAME} BT::behaviortree_cpp_v3)
```

#### Groot
Instale algumas dependências:
```
sudo apt-get install qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
```
Para instalar o [Groot](https://github.com/BehaviorTree/Groot):

```
git clone https://github.com/BehaviorTree/Groot.git
cd Groot
git submodule update --init --recursive
mkdir build; cd build
cmake .. && make
```
O Groot já está instalado. Para facilitar seu uso, vamos copiar o executável para a pasta home:
```
cd Groot/build
cp Groot ~/groot
```
Para utilizar o Groot:
```
cd && ./groot
```

#### Pacotes baxter:
Para instalar os pacotes de simulação do Baxter para ROS Kinetic, siga o passo a passo do [Dave Coleman](https://github.com/davetcoleman):

[Instalação BaxterSimulator](https://hub.docker.com/r/davetcoleman/baxter_simulator)

Adicione o workspace do baxter ao seu arquivo .bashrc:
```
sudo echo "source ~/ws_baxter/devel/setup.bash" >> ~/.bashrc
```

#### Instale este repositório
Para utilizar este repositório, crie um workspace catkin, clone o repositório e instale as dependências e compile:
``` 
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/felipe18mohr/behavior_tree_ros
rosdep install -y --from-paths . --ignore-src --rosdistro kinetic --as-root=apt:false
cd .. && catkin build

```

## Simulando o robô Baxter
### Baxter pick-and-place com FSM

Quando abrir um novo terminal:
```
cd catkin_ws
source devel/setup.bash
```
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
![BT Tree.](/img/baxter_tree.jpeg "BT Tree.")
