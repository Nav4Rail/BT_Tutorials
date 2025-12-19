## QUICKSTART – Simulation TurtleBot3 + Gazebo orchestrée par un Behavior Tree

Ce package met en place une **simulation TurtleBot3 sous Gazebo** et une **mission orchestrée par un Behavior Tree** qui publie sur `/cmd_vel`.

- **Idée générale**  
  - `turtlebot_bt_sim` ne gère pas la simulation lui‑même, mais **inclut** les launch files officiels de `turtlebot3_gazebo`.  
  - Un nœud BT (`turtlebot_bt_node`) charge un arbre XML (`turtlebot_mission.xml`) qui orchestre des actions atomiques :
    - `Idle`, `DriveForward`, `Rotate`, `StopRobot`.  
  - Ces nœuds BT publient des commandes `geometry_msgs/msg/Twist` sur `/cmd_vel`, ce qui fait bouger le robot dans Gazebo.

Depuis ce projet, un **script de génération de Behavior Tree par LLM** (`gen_turtlebot_bt.py`) permet de produire automatiquement un XML de mission à partir d’un prompt en langage naturel, puis de l’exécuter dans la simulation.

---

### 1. Pré‑requis ROS 2 / Simulation

- **ROS 2 Humble** installé (`/opt/ros/humble`).
- **Workspace principal** (ex. `~/ros2_ws`) avec :
  - `turtlebot3_gazebo` installé et compilé,
  - les dépendances TurtleBot3 (modèles, descriptions, etc.),
  - Gazebo Classic fonctionnel.
- **Variable d’environnement TurtleBot3** (souvent requise par les launch officiels) :

```bash
export TURTLEBOT3_MODEL=burger    # ou waffle, waffle_pi
```

- **Workspace BT** : `~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws` avec le package `turtlebot_bt_sim`.
- **BehaviorTree.CPP** disponible (paquet CMake `behaviortree_cpp`).

---

### 2. Pré‑requis LLM et script `gen_turtlebot_bt.py`

Le script `turtlebot_bt_sim/gen_turtlebot_bt.py` :

- reçoit un **prompt en langage naturel** décrivant la mission du TurtleBot,
- appelle un **LLM compatible API OpenAI** (ex. Mistral),
- récupère une liste d’étapes simples (JSON) : `Idle`, `DriveForward`, `Rotate`, `StopRobot`,
- génère un **fichier XML BehaviorTree.CPP** strictement compatible avec les nœuds BT de `turtlebot_bt_sim`.

#### 2.1. Paquets Python nécessaires

Dans l’environnement Python utilisé pour lancer le script :

```bash
pip install openai python-dotenv
```

- **`openai`** : client OpenAI v1, compatible avec l’API Mistral (mode OpenAI‑like).
- **`python-dotenv`** : permet de charger automatiquement un fichier `.env` (variables `LLM_API_KEY`, etc.).

#### 2.2. Variables d’environnement LLM

Le script lit les variables suivantes :

- **`LLM_API_KEY`** : clé API de votre provider (obligatoire si `--api-key` n’est pas passé en argument).
- **`LLM_API_BASE`** : URL de base de l’API (compatible OpenAI).
  - Pour Mistral :

    ```bash
    export LLM_API_BASE="https://api.mistral.ai/v1"
    ```

- **`LLM_MODEL`** : nom du modèle LLM à utiliser.
  - Par défaut, le script utilise `mistral-large-latest` si `LLM_MODEL` est absent.

Exemple de configuration Mistral dans le shell :

```bash
export LLM_API_KEY="YOUR_MISTRAL_KEY"
export LLM_API_BASE="https://api.mistral.ai/v1"
export LLM_MODEL="mistral-large-latest"
```

#### 2.3. Fichier `.env` (optionnel)

`gen_turtlebot_bt.py` charge automatiquement un fichier `.env` (grâce à `python-dotenv`) s’il est présent dans le répertoire courant.  
Cela permet d’éviter d’exporter les variables à chaque nouveau terminal.

Exemple de `.env` à placer dans `ros2_integration_bt_ws/turtlebot_bt_sim` :

```bash
LLM_API_KEY="YOUR_MISTRAL_KEY"
LLM_API_BASE="https://api.mistral.ai/v1"
LLM_MODEL="mistral-large-latest"
```

Les variables déjà présentes dans l’environnement **ne sont pas écrasées** par le `.env`.

#### 2.4. Utilisation simple du script

Depuis `ros2_integration_bt_ws/turtlebot_bt_sim` :

```bash
cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws/turtlebot_bt_sim

python3 gen_turtlebot_bt.py --prompt "Attendre 5s, avancer 2m, puis s'arrêter."
```

Le script :

- appelle le LLM,
- génère un fichier XML dans `trees/` nommé par défaut :
  - `trees/turtlebot_mission_generated_<timestamp>.xml`,
- écrit en tête de fichier des commentaires XML contenant :
  - la date/heure de génération,
  - le prompt,
  - le modèle et l’URL d’API utilisés.

---

### 3. Compilation du package

Dans un terminal **sourcé ROS 2 + workspace principal** :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws

colcon build --packages-select turtlebot_bt_sim
source install/setup.bash
```

---

### 4. Pipeline complète LLM → BT → ROS 2

Cette section décrit le **flux complet**, depuis le prompt utilisateur jusqu’à l’exécution du Behavior Tree dans Gazebo.

#### 4.1. Configurer les variables LLM

Dans un terminal :

```bash
export LLM_API_KEY="YOUR_MISTRAL_KEY"
export LLM_API_BASE="https://api.mistral.ai/v1"
export LLM_MODEL="mistral-large-latest"
```

ou bien définir ces variables dans un **fichier `.env`** placé dans `turtlebot_bt_sim/` (voir §2.3).

#### 4.2. Générer un Behavior Tree à partir d’un prompt

```bash
cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws/turtlebot_bt_sim

python3 gen_turtlebot_bt.py --prompt "Attendre 5s, avancer 2m, puis s'arrêter."
```

Un fichier du type :

```text
trees/turtlebot_mission_generated_20251219001714.xml
```

est créé.

#### 4.3. Sauvegarder l’ancien Behavior Tree

Avant de remplacer `turtlebot_mission.xml`, on crée un backup daté :

```bash
cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws/turtlebot_bt_sim

if [ -f trees/turtlebot_mission.xml ]; then
  cp trees/turtlebot_mission.xml \
     "trees/turtlebot_mission_backup_$(date +%Y%m%d%H%M%S).xml"
fi
```

#### 4.4. Remplacer le Behavior Tree principal

Supposons que le nouveau fichier généré soit `trees/turtlebot_mission_generated_20251219001714.xml` :

```bash
cp trees/turtlebot_mission_generated_20251219001714.xml \
   trees/turtlebot_mission.xml
```

À partir de là, le nœud `turtlebot_bt_node` exécutera ce nouvel arbre.

#### 4.5. Recompiler le package et sourcer l’environnement

```bash
cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws

colcon build --packages-select turtlebot_bt_sim
source install/setup.bash
```

#### 4.6. Lancer Gazebo + Behavior Tree

```bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot_bt_sim turtlebot_bt_sim.launch.py
```

---

### 5. Script bash de pipeline complet (LLM → BT → ROS 2)

Pour automatiser toute la chaîne, un script bash peut être utilisé (par exemple `run_turtlebot_bt_pipeline.sh` à la racine du workspace BT).

Exemple de script :

```bash
#!/usr/bin/env bash
set -euo pipefail

log() {
  echo "[$(date +'%Y-%m-%d %H:%M:%S')] [INFO] $*"
}

err() {
  echo "[$(date +'%Y-%m-%d %H:%M:%S')] [ERROR] $*" >&2
}

PROMPT="${1:-}"

if [ -z "$PROMPT" ]; then
  read -rp "Description de la mission (prompt LLM) : " PROMPT
fi

# Chemins par défaut (adaptables via variables d'environnement)
ROS_MAIN_WS="${ROS_MAIN_WS:-$HOME/ros2_ws}"
BT_WS="${BT_WS:-$HOME/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws}"

log "Sourcing ROS 2 Humble et workspace principal..."
source /opt/ros/humble/setup.bash
source "$ROS_MAIN_WS/install/setup.bash"

cd "$BT_WS"
log "Sourcing workspace BT..."
source install/setup.bash

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
log "TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"

cd "$BT_WS/turtlebot_bt_sim"

log "Génération du Behavior Tree via LLM..."
python3 gen_turtlebot_bt.py --prompt "$PROMPT"

LATEST_XML="$(ls -1t trees/turtlebot_mission_generated_*.xml | head -n 1)"

if [ -z "$LATEST_XML" ] || [ ! -f "$LATEST_XML" ]; then
  err "Aucun fichier généré trouvé dans trees/turtlebot_mission_generated_*.xml"
  exit 1
fi

log "Dernier Behavior Tree généré : $LATEST_XML"

if [ -f trees/turtlebot_mission.xml ]; then
  BACKUP="trees/turtlebot_mission_backup_$(date +'%Y%m%d%H%M%S').xml"
  log "Backup de trees/turtlebot_mission.xml vers $BACKUP"
  cp trees/turtlebot_mission.xml "$BACKUP"
else
  log "Pas de fichier trees/turtlebot_mission.xml existant, pas de backup créé."
fi

log "Remplacement de trees/turtlebot_mission.xml par $LATEST_XML"
cp "$LATEST_XML" trees/turtlebot_mission.xml

cd "$BT_WS"
log "Compilation de turtlebot_bt_sim..."
colcon build --packages-select turtlebot_bt_sim
source install/setup.bash

log "Lancement de Gazebo + Behavior Tree..."
ros2 launch turtlebot_bt_sim turtlebot_bt_sim.launch.py
```

Utilisation :

```bash
cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws
chmod +x run_turtlebot_bt_pipeline.sh

./run_turtlebot_bt_pipeline.sh "Attendre 5s, avancer 2m, puis s'arrêter."
```

ou sans argument, le script vous demandera le prompt au clavier.

---

### 6. Fichiers clés

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

### 7. Flow ROS 2 – de Gazebo au Behavior Tree

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

### 8. Extensions possibles

- Modifier `turtlebot_mission.xml` pour créer des **missions plus riches** :
  - patrouilles, boucles, séquences conditionnelles, etc.
- Ajouter de nouveaux nœuds BT :
  - par exemple, un nœud qui attend un certain **topic** (capteur),
  - un nœud qui vérifie une condition (distance à un obstacle, etc.).
- Intégrer BehaviorTree.ROS2 (comme dans `ros2_integration_wrappers`) :
  - par exemple, pour appeler des actions de navigation `nav2` directement depuis l’arbre BT.
