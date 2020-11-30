# behavior_tree_ros

Este repositório traz um exemplo de implementação
Para mais informações sobre a BT, acesse o repositório [behavior_tree_ros](https://github.com/felipe18mohr/behavior_tree_ros)
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

O código para simulação do Baxter foi retirado do repositório [baxter_smacha](https://github.com/abr-ijs/baxter_smacha)

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


### Funcionamento baxter_bt

Há três arquivos principais que compõem o projeto, contidos na pasta */src*:

- **pick_and_place_bt.cpp:** Esse é o código principal. Nele, importamos as classes criadas, criamos a Árvore de Comportamento, registramos os Nós Folhas (a árvore é criada e sua estrutura é montada a partir de um arquivo XML, gerada de forma visual pelo Groot) e executamos o "tick" no root da Árvore, que será propagado para os Nós filhos, até que seja retornado SUCCCESS ou FAILURE, além de adicionar algumas opções de log.
  
- **baxter_class.cpp:** Possui a implementação de uma Classe para o controle do robô, através do ROS. Uma forma de se utilizar Behavior Trees em conjunto com o ROS é fazendo uso das  ROS Actions (da biblioteca actionlib). Isso porque, assim com as ROS Actions, muitos ActionNodes utilizados pela BT são assíncronos. 
  
  Porém, a aplicação não se limita a isso, é possível implementar o que se queira relacionado a ROS dentro de uma BT: em um dos métodos da classe, por exemplo, é utilizado um ROS Service, que calcula as posições de cada junta do robô através de Cinemática Inversa. Em outro, há um ROS Subscriber que retorna a posição no espaço que se encontra o gripper. 
  
  Note que, nessa classe criada, não há absolutamente nada relacionado a BT, tornando-a uma classe genérica, podendo ser utilizada em outras aplicações, mesmo que sem uso de Behavior Trees.
  
- **nodes.cpp:** É nesse arquivo que são criados todos os ActionNodes (e ConditionNodes, se fossem necessários). Existem diversas formas de se criar os Nodes de uma BT, nesse caso eles foram criados a partir de uma Herança Múltipla, herdando tanto a classe de Action da BT quanto a classe criada para o controle do robô, unindo os dois mundos. Note que foi criada uma classe para cada um dos Nodes, de forma que no arquivo principal (*pic_and_place_bt.cpp*) eles possam ser registrados de maneira mais simples e genérica.

  Além disso, é nesse arquivo que estão também todos os templates de Conversão de String, necessários para ler os valores de Entrada da Árvore. Esses Inputs serão retornados pelo método providedPorts() de cada ActionNode (não foram utilizadas portas de Output nesse exemplo).

  O método tick() existe dentro da classe do ActionNode herdada, e deve ser sobrescrito dentro de cada uma das novas classes criadas. É ele que será executado quando o ActionNode receber um "tick". Como os ActionNodes criados são Assíncronos, será criada uma nova thread quando esse Node receber o primeiro tick, e ele retornará o NodeStatus RUNNING. Quando o Node receber um tick novamente, ele continuará retornando RUNNING, enquanto a thread ainda estiver rodando. 

  Assim como o método tick(), o método halt() é sobrescrito dentro de cada classe, e é chamado automaticamente pela biblioteca caso ocorra algo que deva interromper a execução do Node.

Note que essa é apenas uma forma de se implementar uma Behavior Tree em conjunto com o ROS. Não há nenhuma regra que especifique exatamente como isso deve ser feito, tornando as coisas bastante flexíveis.
