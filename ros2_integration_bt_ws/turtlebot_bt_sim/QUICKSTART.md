## QUICKSTART – Simulation TurtleBot3 + Gazebo orchestrée par un Behavior Tree

Ce package met en place une **simulation TurtleBot3 sous Gazebo** et une **mission orchestrée par un Behavior Tree** qui publie sur `/cmd_vel`.

- **Idée générale**  
  - `turtlebot_bt_sim` ne gère pas la simulation lui‑même, mais **inclut** les launch files officiels de `turtlebot3_gazebo`.  
  - Un nœud BT (`turtlebot_bt_node`) charge un arbre XML (`turtlebot_mission.xml`) qui orchestre des actions atomiques :
    - `Idle`, `DriveForward`, `Rotate`, `StopRobot`.  
  - Ces nœuds BT publient des commandes `geometry_msgs/msg/Twist` sur `/cmd_vel`, ce qui fait bouger le robot dans Gazebo.

---

### 1. Pré‑requis

- ROS 2 Humble installé (`/opt/ros/humble`).
- Workspace principal (ex. `~/ros2_ws`) avec :
  - `turtlebot3_gazebo` installé et compilé,
  - les dépendances TurtleBot3 (modèles, descriptions, etc.),
  - Gazebo Classic fonctionnel.
- Variable d’environnement TurtleBot3 (souvent requise par les launch officiels) :

```bash
export TURTLEBOT3_MODEL=burger    # ou waffle, waffle_pi
```

- Ce workspace BT : `.../BT_Tutorials/ros2_integration_bt_ws` avec le package `turtlebot_bt_sim`.
- `BehaviorTree.CPP` disponible (paquet CMake `behaviortree_cpp`).

---

### 2. Compilation du package

Dans un terminal sourcé ROS 2 + workspace principal :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws

colcon build --packages-select turtlebot_bt_sim
source install/setup.bash
```

---

### 3. Lancer la simulation + Behavior Tree

Dans un terminal sourcé (ROS 2 + `~/ros2_ws` + workspace BT) :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws
source install/setup.bash

export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot_bt_sim turtlebot_bt_sim.launch.py
```

Ce launch :
- inclut `turtlebot3_world.launch.py` depuis `turtlebot3_gazebo` (robot + monde Gazebo),
- démarre le nœud BT `turtlebot_bt_node` qui pilote `/cmd_vel`.

Pour vérifier la commande envoyée au robot :

```bash
ros2 topic echo /cmd_vel
```

---

### 4. Fichiers clés

- **`CMakeLists.txt`**
  - Exécutable `turtlebot_bt_node` :
    - `src/turtlebot_bt_main.cpp`,
    - `src/turtlebot_bt_nodes.cpp`.
  - Dépendances :
    - `rclcpp`, `geometry_msgs`, `ament_index_cpp`, `behaviortree_cpp`.
  - Installe :
    - headers `include/`,
    - arbres XML `trees/`,
    - launch files `launch/`.

- **`package.xml`**
  - `buildtool_depend` : `ament_cmake`.
  - `depend` :
    - `rclcpp`, `geometry_msgs`, `ament_index_cpp`, `behaviortree_cpp`.
  - `exec_depend` :
    - `gazebo_ros`, `turtlebot3_gazebo`.

- **`include/turtlebot_bt_sim/turtlebot_bt_nodes.hpp`**
  - Déclare :

    ```cpp
    struct CmdVelHandle {
      rclcpp::Node::SharedPtr node;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    };

    void register_turtlebot_nodes(BT::BehaviorTreeFactory & factory,
                                  const std::shared_ptr<CmdVelHandle> & handle);
    ```

  - `CmdVelHandle` est passé aux nœuds BT pour publier sur `/cmd_vel`.

- **`src/turtlebot_bt_nodes.cpp`**
  - Définit plusieurs nœuds BT dérivés de `BT::SyncActionNode` :
    - `IdleNode` :
      - port `duration` (s),
      - publie un `Twist` nul pendant `duration` secondes (robot immobile, mais le BT “occupe” ce temps).
    - `DriveForwardNode` :
      - ports `duration` (s) et `speed` (m/s),
      - publie `linear.x = speed` pendant `duration` secondes, puis remet à zéro.
    - `RotateNode` :
      - ports `duration` (s) et `angular_speed` (rad/s),
      - publie `angular.z = angular_speed` pendant `duration` secondes, puis remet à zéro.
    - `StopRobotNode` :
      - publie un `Twist` nul une fois (arrêt de sécurité),
      - loggue que le robot est arrêté.
  - `register_turtlebot_nodes(...)` :
    - enregistre ces classes avec des builders capturant `CmdVelHandle` :
      - IDs XML : `Idle`, `DriveForward`, `Rotate`, `StopRobot`.

- **`src/turtlebot_bt_main.cpp`**
  - Fonction `main` :
    - crée un `rclcpp::Node` `turtlebot_bt_node`,
    - lance un `SingleThreadedExecutor` dans un thread,
    - crée un `CmdVelHandle` avec :
      - `node`,
      - `cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);`
    - attend qu’au moins **un abonné** à `/cmd_vel` soit présent (Gazebo / contrôleur du robot) :

      ```cpp
      while (rclcpp::ok() && node->count_subscribers("cmd_vel") == 0) {
        RCLCPP_INFO(node->get_logger(), "Waiting for /cmd_vel subscriber (robot spawn)...");
        std::this_thread::sleep_for(500ms);
      }
      ```

    - crée un `BT::BehaviorTreeFactory`, appelle `register_turtlebot_nodes(factory, handle);`,
    - trouve `trees/turtlebot_mission.xml` via `ament_index_cpp`,
    - charge l’arbre, l’exécute avec `tickWhileRunning(10ms)`,
    - loggue SUCCESS / FAILURE, arrête l’executor, shutdown ROS 2.

- **`trees/turtlebot_mission.xml`**
  - Arbre XML pilotant la mission :

    ```xml
    <root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence name="MissionSequence">
          <Action ID="Idle" duration="10.0"/>
          <Action ID="DriveForward" duration="5.0" speed="0.2"/>
          <Action ID="Rotate" duration="4.0" angular_speed="0.6"/>
          <Action ID="DriveForward" duration="3.0" speed="0.2"/>
          <Action ID="StopRobot"/>
        </Sequence>
      </BehaviorTree>
    </root>
    ```

  - Les IDs (`Idle`, `DriveForward`, `Rotate`, `StopRobot`) correspondent aux nœuds enregistrés dans `register_turtlebot_nodes`.

- **`launch/turtlebot_bt_sim.launch.py`**
  - Déclare deux arguments :
    - `model` (par défaut `burger`),
    - `use_sim_time` (par défaut `true`).
  - Utilise `get_package_share_directory("turtlebot3_gazebo")` pour localiser :
    - `launch/turtlebot3_world.launch.py`.
  - Inclut ce launch via `IncludeLaunchDescription` :
    - passe `model` et `use_sim_time` en arguments.
  - Lance ensuite le nœud BT `turtlebot_bt_node` avec `use_sim_time`.

---

### 5. Flow ROS 2 – de Gazebo au Behavior Tree

1. **Gazebo + TurtleBot3** (via `turtlebot3_gazebo`) :
   - spawn le robot dans un monde standard,
   - met en place un abonné sur `/cmd_vel` (plugin de contrôle).
2. **Nœud BT `turtlebot_bt_node`** :
   - attend qu’un abonné `/cmd_vel` soit présent,
   - enregistre des nœuds BT (Idle/Drive/Rotate/Stop) reliés à un publisher `/cmd_vel`.
3. **Behavior Tree (`turtlebot_mission.xml`)** :
   - exécute en séquence :
     - une phase d’Idle,
     - une translation en avant,
     - une rotation sur place,
     - une autre translation,
     - un arrêt de sécurité.
4. Les messages `Twist` publiés sur `/cmd_vel` font bouger le robot dans Gazebo, jusqu’à la fin de la mission (SUCCESS ou FAILURE).

---

### 6. Extensions possibles

- Modifier `turtlebot_mission.xml` pour créer des **missions plus riches** :
  - patrouilles, boucles, séquences conditionnelles, etc.
- Ajouter de nouveaux nœuds BT :
  - par exemple, un nœud qui attend un certain **topic** (capteur),
  - un nœud qui vérifie une condition (distance à un obstacle, etc.).
- Intégrer BehaviorTree.ROS2 (comme dans `ros2_integration_wrappers`) :
  - par exemple, pour appeler des actions de navigation `nav2` directement depuis l’arbre BT.


