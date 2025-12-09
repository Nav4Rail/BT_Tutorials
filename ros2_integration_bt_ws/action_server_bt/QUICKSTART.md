## QUICKSTART – Serveur d’actions Behavior Tree `/run_tree`

Ce package fournit un **serveur d’actions ROS 2** qui exécute des **Behavior Trees (BehaviorTree.CPP)** à la demande, via l’action personnalisée `RunTree`.

- **Idée générale**  
  - Un client envoie un goal sur l’action `/run_tree` avec un `tree_id`.  
  - Le serveur charge le fichier XML correspondant (`trees/<tree_id>.xml`), exécute l’arbre, publie du feedback, puis renvoie un résultat (SUCCESS / FAILURE).

---

### 1. Pré‑requis

- ROS 2 Humble installé (`/opt/ros/humble`).
- Workspace principal (ex. `~/ros2_ws`) avec :
  - `BehaviorTree.CPP` installé au niveau système (paquet CMake `behaviortree_cpp`).
  - (Optionnel mais recommandé) BehaviorTree.ROS2 déjà installé, comme décrit dans `ros2_integration_bt_ws/QUICKSTART.md`.
- Ce workspace BT : `.../BT_Tutorials/ros2_integration_bt_ws` contenant ce package `action_server_bt`.

---

### 2. Compilation du package

Dans un terminal sourcé ROS 2 :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws

colcon build --packages-select action_server_bt
source install/setup.bash
```

Si tu rebuildes souvent, tu peux garder ce terminal comme **terminal “build + run”** du workspace BT.

---

### 3. Lancer le serveur d’action BT

Toujours dans un terminal où le workspace BT est sourcé :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws
source install/setup.bash

ros2 launch action_server_bt action_server_bt.launch.py
```

Le nœud `BtActionServerNode` démarre et crée un serveur d’actions ROS 2 sur `/run_tree`.

Pour tester avec un goal simple (utilisant l’arbre par défaut `simple_mission`) :

```bash
ros2 action send_goal /run_tree action_server_bt/action/RunTree "{tree_id: 'simple_mission'}"
```

Tu devrais voir dans les logs :
- des messages `[BT] SaySomething: ...`,
- un résultat final indiquant SUCCESS ou FAILURE.

---

### 4. Fichiers clés

- **`package.xml`**  
  - Déclare le package `action_server_bt`.  
  - Dépendances principales :
    - **buildtool** : `ament_cmake`
    - **run/build** : `rclcpp`, `rclcpp_action`, `ament_index_cpp`, `behaviortree_cpp`
    - **actions** : `rosidl_default_generators` (build), `rosidl_default_runtime` (exec)
    - membre du groupe `rosidl_interface_packages` pour l’action `RunTree`.

- **`CMakeLists.txt`**  
  - Génère l’interface d’action :
    - `action/RunTree.action` via `rosidl_generate_interfaces(...)`.
  - Construit l’exécutable :
    - `bt_action_server` (fichiers `src/bt_action_server_main.cpp` et `src/bt_action_server_node.cpp`).
  - Lie les dépendances :
    - `rclcpp`, `rclcpp_action`, `ament_index_cpp`, `behaviortree_cpp::behaviortree_cpp`.
  - Installe :
    - les headers `include/`,
    - les arbres XML `trees/`,
    - les fichiers de lancement `launch/`.

- **`action/RunTree.action`**  
  - Décrit l’action `/run_tree` :
    - **Goal** : `string tree_id` (nom logique de l’arbre à exécuter).  
    - **Result** : `bool success`, `string message`.  
    - **Feedback** : `string current_node` (info sur la progression).

- **`include/action_server_bt/bt_action_server_node.hpp`**  
  - Classe `BtActionServerNode : public rclcpp::Node` :
    - crée le serveur d’actions `rclcpp_action::Server<RunTree>`,
    - possède un `BT::BehaviorTreeFactory factory_`,
    - expose une méthode `init_action_server()` (appelée après la création du `shared_ptr` dans `main`).

- **`src/bt_action_server_node.cpp`**  
  - Implémente :
    - **nœuds BT personnalisés** :
      - `SaySomethingNode` (affiche un message, port `message`),
      - `WaitMsNode` (attend un délai, port `milliseconds`).
    - **`register_simple_nodes()`** :
      - enregistre ces types dans la factory (`factory_.registerNodeType<...>("ID")`).
    - **Cycle de vie de l’action** :
      - `handle_goal(...)` : logs, accepte le goal.
      - `handle_cancel(...)` : marque une annulation (proche de la fin du BT).
      - `handle_accepted(...)` : lance `execute_tree(...)` dans un thread.
      - `execute_tree(...)` :
        - résout le chemin de l’arbre XML (`trees/<tree_id>.xml`, fallback `simple_mission`),
        - charge l’arbre `BT::Tree tree = factory_.createTreeFromFile(tree_file);`,
        - exécute l’arbre avec `tickWhileRunning(10ms)`,
        - construit un `RunTree::Result` (SUCCESS / FAILURE / CANCELED),
        - publie un dernier feedback (`current_node`).

- **`src/bt_action_server_main.cpp`**  
  - Initialise ROS 2, crée `std::shared_ptr<BtActionServerNode>`, appelle `node->init_action_server()`, lance un `MultiThreadedExecutor` et le fait tourner jusqu’à `Ctrl+C`.

- **`trees/simple_mission.xml`**  
  - Arbre XML d’exemple :
    - **Séquence** :
      - `SaySomething` “Starting BT mission from action_server_bt”
      - `WaitMs` 1000 ms
      - `SaySomething` “BT mission finished”

- **`launch/action_server_bt.launch.py`**  
  - Lance un seul nœud :
    - package `action_server_bt`, exécutable `bt_action_server`, nom de nœud `bt_action_server`.

---

### 5. Flow ROS 2 – de l’action au Behavior Tree

- **Client**  
  - Envoie un goal de type `action_server_bt/action/RunTree` sur `/run_tree` avec un `tree_id`.

- **Serveur `/run_tree` (BtActionServerNode)**  
  - Accepte le goal (`ACCEPT_AND_EXECUTE`).
  - Résout le fichier XML via `ament_index_cpp::get_package_share_directory("action_server_bt")`.
  - Charge l’arbre dans un `BT::Tree`.
  - Exécute l’arbre (boucle interne BehaviorTree.CPP).
  - Publie des **feedbacks** (nom de nœud, progression).
  - Renvoie un **résultat** à la fin :
    - `success=true` si l’arbre termine en SUCCESS,
    - `success=false` si échec, erreur de fichier, ou annulation.

- **Behavior Tree**  
  - Définit l’ordonnancement logique (séquences, actions, etc.).
  - Peut être modifié sans recompiler tant que :
    - les IDs des nœuds utilisés existent côté C++,
    - la structure reste conforme à BTCPP v4.

---

### 6. Extensions possibles

- Ajouter d’autres fichiers XML dans `trees/` (ex. `navigation_mission.xml`) et les lancer via `tree_id`.
- Créer de nouveaux nœuds BT C++ :
  - dériver `BT::SyncActionNode` ou `BT::StatefulActionNode`,
  - les enregistrer dans `register_simple_nodes()`,
  - les utiliser dans les arbres XML.
- Intégrer ce serveur d’actions BT comme **backend de planification** pour d’autres nœuds ROS 2 (par ex. un planificateur haut niveau qui sélectionne le `tree_id` à exécuter).


