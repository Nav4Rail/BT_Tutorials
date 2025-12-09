## QUICKSTART – BT + ROS 2 Fibonacci

Ce mini‑projet montre deux façons de connecter **BehaviorTree.CPP** à un serveur d’action ROS 2 Fibonacci :

- **Approche 1 – Client manuel** (`ros2_integration`)  
- **Approche 2 – Client via BehaviorTree.ROS2** (`ros2_integration_wrappers`)

---

### 1. Pré‑requis

- ROS 2 Humble correctement installé (`/opt/ros/humble`).
- Un workspace ROS 2 principal pour le **serveur d’action Fibonacci**, par ex. `~/ros2_ws`, qui suit le tutoriel officiel ROS 2 Humble C++ :  
  [Writing an Action Server and Client (C++)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- **BehaviorTree.CPP** déjà installé au niveau système (comme décrit dans `ros2_integration_bt_ws/README.md`).

#### 1.1. Installer BehaviorTree.ROS2

BehaviorTree.ROS2 fournit le package CMake `behaviortree_ros2` utilisé par `ros2_integration_wrappers`.  
Il doit être présent dans *un* workspace ROS 2 (par exemple le workspace principal `~/ros2_ws`) :

```bash
cd ~/ros2_ws/src
git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git


source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select btcpp_ros2_interfaces behaviortree_ros2
source install/setup.bash
```

Après cette étape, CMake peut faire `find_package(behaviortree_ros2 REQUIRED)` dans les autres workspaces.

---

### 2. Workspace BT – Compilation

Workspace BT :

```bash
cd <path>/BT_Tutorials/ros2_integration_bt_ws
```

S'assurer d’abord que ton environnement ROS 2 (et éventuellement `~/ros2_ws`) est sourcé dans ce terminal :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Puis compiler les deux packages BT :

```bash
colcon build --packages-select ros2_integration ros2_integration_wrappers
source install/setup.bash
```

---

### 3. Lancer le serveur d’action Fibonacci

Dans un **autre terminal** (sourcé avec ROS 2 + `~/ros2_ws`) :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run action_tutorials_cpp fibonacci_action_server
```

Ce serveur est celui du tutoriel officiel et fournit l’action `action_tutorials_interfaces/action/Fibonacci` (https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html).

---

### 4. Approche 1 – Client BT manuel

Dans un terminal où **le workspace BT est sourcé** :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws
source install/setup.bash

ros2 run ros2_integration bt_node
```

- L’arbre XML utilisé est `ros2_integration/trees/fibonacci_manual.xml`.  
- Le nœud d’action est `FibonacciActionNode` (client `rclcpp_action` « écrit à la main »).

---

### 5. Approche 2 – Client BT via BehaviorTree.ROS2

Dans un terminal également sourcé (ROS 2 + `~/ros2_ws` + workspace BT) :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws
source install/setup.bash

ros2 run ros2_integration_wrappers bt_node_wrappers
```

- L’arbre XML utilisé est `ros2_integration_wrappers/trees/fibonacci_ros2.xml`.  
- Le nœud d’action est `FibonacciRosActionNode`, dérivé de `BT::RosActionNode` (BehaviorTree.ROS2).

Les deux approches devraient envoyer un goal à `/fibonacci` et terminer avec `SUCCESS` si le serveur renvoie correctement la séquence.


