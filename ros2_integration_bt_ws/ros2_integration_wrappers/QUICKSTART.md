## QUICKSTART – Client BT Fibonacci via BehaviorTree.ROS2

Ce package montre comment utiliser **BehaviorTree.ROS2** pour créer un **client d’action BehaviorTree.CPP** qui appelle le serveur d’action ROS 2 **Fibonacci** (du tutoriel officiel ROS 2).

- **Idée générale**  
  - Un nœud BT (`bt_node_wrappers`) charge un arbre XML (`fibonacci_ros2.xml`).  
  - L’arbre contient un nœud d’action `FibonacciRosActionNode` (wrapper BehaviorTree.ROS2).  
  - Ce nœud envoie un goal à `/fibonacci`, attend le résultat et renvoie SUCCESS / FAILURE à l’arbre BT.

---

### 1. Pré‑requis

- ROS 2 Humble installé (`/opt/ros/humble`).
- Workspace principal `~/ros2_ws` avec :
  - le tutoriel **action Fibonacci** compilé :
    - package `action_tutorials_cpp`,
    - action `action_tutorials_interfaces/action/Fibonacci`.
  - **BehaviorTree.ROS2** installé :
    - packages `btcpp_ros2_interfaces` et `behaviortree_ros2` buildés, comme décrit dans `ros2_integration_bt_ws/QUICKSTART.md`.
- Ce workspace BT : `.../BT_Tutorials/ros2_integration_bt_ws` avec le package `ros2_integration_wrappers`.

---

### 2. Compilation du package

Dans un terminal sourcé ROS 2 + workspace principal :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws

colcon build --packages-select ros2_integration_wrappers
source install/setup.bash
```

---

### 3. Lancer le serveur d’action Fibonacci (tutoriel ROS 2)

Dans un **autre terminal**, aussi sourcé avec ROS 2 + `~/ros2_ws` :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run action_tutorials_cpp fibonacci_action_server
```

Ce nœud fournit l’action :
- Type : `action_tutorials_interfaces/action/Fibonacci`  
- Nom : `/fibonacci`

---

### 4. Lancer le client BT basé sur BehaviorTree.ROS2

Dans un terminal où le workspace BT est sourcé :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws
source install/setup.bash

ros2 run ros2_integration_wrappers bt_node_wrappers
```

- Le nœud crée un `rclcpp::Node` et un `BT::BehaviorTreeFactory`.
- Il enregistre `FibonacciRosActionNode` comme nœud d’action BT.
- Il charge `ros2_integration_wrappers/trees/fibonacci_ros2.xml` via `ament_index_cpp`.
- L’arbre ticke jusqu’à SUCCESS / FAILURE.

---

### 5. Fichiers clés

- **`CMakeLists.txt`**
  - Déclare l’exécutable `bt_node_wrappers` :
    - `src/bt_node_wrappers.cpp`.
  - Dépendances :
    - `rclcpp`, `rclcpp_action`, `ament_index_cpp`,
    - `behaviortree_cpp`, `behaviortree_ros2`,
    - `action_tutorials_interfaces`.
  - Lie explicitement la lib BehaviorTree.CPP :
    - `target_link_libraries(... behaviortree_cpp::behaviortree_cpp)`.

- **`package.xml`**
  - Dépendances runtime :
    - `rclcpp`, `rclcpp_action`, `ament_index_cpp`,
    - `behaviortree_cpp`, `behaviortree_ros2`,
    - `action_tutorials_interfaces`.
  - Type de build : `ament_cmake`.

- **`include/ros2_integration_wrappers/fibonacci_ros_action_node.hpp`**
  - Classe `FibonacciRosActionNode` :
    - dérive de `BT::RosActionNode<action_tutorials_interfaces::action::Fibonacci>`,
    - utilise les **wrappers BehaviorTree.ROS2** pour :
      - création du client d’action,
      - envoi de goal,
      - gestion des callbacks / états.
  - Ports BT :
    - `InputPort<int>("order", 10, "Length of the Fibonacci sequence")`.
  - Méthodes clés :
    - `setGoal(Goal & goal)` :
      - lit le port `order` et assigne `goal.order`.
    - `onResultReceived(const WrappedResult & result)` :
      - loggue la séquence renvoyée,
      - renvoie `NodeStatus::SUCCESS` / `FAILURE` selon `result.code`.

- **`src/bt_node_wrappers.cpp`**
  - Fonction `main` :
    - initialise ROS 2,
    - crée `rclcpp::Node` + `SingleThreadedExecutor` dans un thread,
    - crée un `BT::BehaviorTreeFactory`,
    - configure `BT::RosNodeParams params` :
      - `params.nh = node;`
      - `params.default_port_value = "fibonacci";` (nom de l’action),
      - `params.server_timeout = 5s;`.
    - enregistre `FibonacciRosActionNode` avec un builder lambda capturant `params`,
    - résout et charge `trees/fibonacci_ros2.xml`,
    - exécute l’arbre avec `tickWhileRunning(10ms)`.

- **`trees/fibonacci_ros2.xml`**
  - Arbre XML minimal :
    - **Séquence** `MainTree` avec un seul nœud :
      - `<Action ID="FibonacciRosActionNode" order="10"/>`
    - L’ID `FibonacciRosActionNode` correspond à la classe enregistrée dans la factory.

---

### 6. Flow ROS 2 – BehaviorTree.ROS2 en action

1. **Serveur d’action Fibonacci** (`fibonacci_action_server`) tourne sur `/fibonacci`.
2. **Client BT** (`bt_node_wrappers`) :
   - crée un nœud ROS 2 et un `BT::BehaviorTreeFactory`,
   - enregistre `FibonacciRosActionNode` (wrapper BehaviorTree.ROS2).
3. **Behavior Tree** (XML) :
   - décrit la logique : “appeler Fibonacci avec `order=10`”.
4. **Nœud BT `FibonacciRosActionNode`** :
   - construit un goal avec le port d’entrée `order`,
   - envoie le goal via BehaviorTree.ROS2,
   - reçoit le résultat, loggue la séquence,
   - renvoie SUCCESS ou FAILURE à l’arbre.
5. L’arbre se termine en SUCCESS si l’action réussi, en FAILURE sinon.

---

### 7. Extensions possibles

- Modifier le port `order` dans le XML pour tester différentes longueurs de séquence.
- Ajouter des nœuds BT de log ou de contrôle autour de `FibonacciRosActionNode` (séquences, fallback, répétitions, etc.).
- S’inspirer de ce pattern pour appeler d’autres actions ROS 2 avec des wrappers `BT::RosActionNode<YourAction>` :
  - implémenter `setGoal(...)` + `onResultReceived(...)`,
  - enregistrer le nœud dans la factory,
  - l’utiliser dans tes arbres XML.


