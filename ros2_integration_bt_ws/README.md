## Intégration de BehaviorTree.CPP avec ROS 2

Ce workspace `ros2_integration_bt_ws` illustre comment intégrer **BehaviorTree.CPP** dans l’écosystème **ROS 2** à travers deux approches complémentaires :

- **Approche 1 – Implémentation manuelle** : écrire un Action Node en C++ avec `BehaviorTree.CPP` et le connecter à un serveur d’action ROS 2 via `rclcpp_action::Client`.
- **Approche 2 – Utilisation de BehaviorTree.ROS2** : s’appuyer sur les nœuds et wrappers déjà fournis par `BehaviorTree.ROS2` pour manipuler actions, services, topics, etc.

L’objectif est de comprendre la mécanique interne (approche 1) puis de profiter des abstractions de plus haut niveau (approche 2), en particulier sur un exemple d’**action Fibonacci**.

---

## 1. Contexte et prérequis

- **Workspace ROS 2** :  
  `~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws`
- **Package principal** :  
  `ros2_integration` (type `ament_cmake`, dépendances `rclcpp`, `rclcpp_action`, `behaviortree_cpp_v3`, `behaviortree_ros2`).
- **Bibliothèques installées au niveau système** :
  - `BehaviorTree.CPP` (installée via `sudo make install`, headers et CMake config visibles dans `/usr/local`).
  - `BehaviorTree.ROS2` (soit via paquets binaires, soit installée depuis les sources).

Dans le `CMakeLists.txt` du package `ros2_integration`, on retrouve notamment :

```12:28:ros2_integration_bt_ws/ros2_integration/CMakeLists.txt
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
```

Ce bloc indique que pendant la configuration CMake, les headers et bibliothèques de ROS 2, BehaviorTree.CPP et BehaviorTree.ROS2 seront automatiquement découverts sur le système (grâce aux fichiers `*.cmake` installés par `sudo make install` ou par les paquets ROS 2).

### 1.1. Cycle de build typique

Depuis la racine du workspace :

```bash
cd ~/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws
colcon build --packages-select ros2_integration
source install/setup.bash
```

Après compilation, les exécutables du package sont accessibles via :

```bash
ros2 run ros2_integration <nom_exécutable>
```

---

## 2. Architecture générale de l’intégration BT ↔ ROS 2

Conceptuellement, l’intégration repose sur trois couches :

- **Moteur de décision** : BehaviorTree.CPP (arbre, nœuds, blackboard, ports, loggers…).
- **Nœuds d’interface** : Action Nodes, Condition Nodes, Decorators… qui appellent les API ROS 2 (`rclcpp`, `rclcpp_action`, etc.).
- **Infrastructure ROS 2** : action servers, topics, services, paramètres, timers.

Dans un node ROS 2 typique intégrant BehaviorTree.CPP :

1. On crée un `BehaviorTreeFactory` et on y enregistre les nœuds custom (dont les Action Nodes).
2. On charge un fichier XML (par ex. dans `ros2_integration/trees/`) décrivant l’arbre.
3. On crée la structure `BT::Tree` à partir du factory.
4. Dans une boucle (souvent un timer ou une boucle `while(rclcpp::ok())`), on :
   - appelle `rclcpp::spin_some(node)` pour traiter les callbacks ROS 2,
   - ticke l’arbre (`tree.tickRoot()` ou `tree.tickWhileRunning()`).

L’arbre devient alors **l’orchestrateur** du comportement : il décide quand envoyer des goals d’action, quand publier des messages, quand s’arrêter, etc.

---

## 3. Approche 1 – Action Node manuel avec BehaviorTree.CPP + rclcpp_action

Dans cette approche, on n’utilise que **BehaviorTree.CPP** et les API ROS 2 classiques.  
On écrit « à la main » un nœud BT qui encapsule un **client d’action** (`rclcpp_action::Client`) et se connecte à un **serveur d’action Fibonacci** implémenté séparément.

### 3.1. Architecture logique

On distingue trois éléments :

- **Serveur d’action Fibonacci** (node ROS 2 classique) :
  - Implémente `example_interfaces/action/Fibonacci` (ou `action_tutorials_interfaces/action/Fibonacci`) comme dans le tutoriel officiel ROS 2 Humble.
  - Reçoit un goal avec un `order`, publie la séquence en feedback, retourne la série complète en résultat.

- **Client d’action encapsulé dans un Action Node BT** :
  - Classe C++ dérivée d’un nœud d’action BehaviorTree.CPP (par exemple `BT::StatefulActionNode` ou `BT::AsyncActionNode`).
  - Crée un `rclcpp_action::Client<Fibonacci>` et envoie un goal lorsque le nœud est tické.
  - Traduit l’état de l’action (en attente, en cours, succès, échec) en statut BT (`RUNNING`, `SUCCESS`, `FAILURE`).

- **Node maître BT + ROS 2** :
  - Node `rclcpp::Node` qui possède le `BehaviorTreeFactory`, le `Tree` et éventuellement les objets ROS partagés (comme un `rclcpp::Node` que l’Action Node utilise).

### 3.2. Schéma de l’Action Node manuel

De manière conceptuelle, un Action Node pour Fibonacci ressemble à ceci :

```cpp
class FibonacciActionNode : public BT::StatefulActionNode
{
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  FibonacciActionNode(const std::string& name,
                      const BT::NodeConfiguration& config,
                      const rclcpp::Node::SharedPtr& ros_node)
    : BT::StatefulActionNode(name, config),
      node_(ros_node)
  {
    client_ = rclcpp_action::create_client<Fibonacci>(node_, "fibonacci");
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("order") };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Fibonacci>::SharedPtr client_;
  GoalHandleFibonacci::SharedPtr goal_handle_;
};
```

L’idée :

- `onStart()` est appelé au premier tick : on lit le port `order`, on envoie le goal à l’action server et on retourne `RUNNING`.
- `onRunning()` est appelé aux ticks suivants : on teste si le résultat est disponible.
  - Si le résultat est reçu avec succès ⇒ `SUCCESS`.
  - Si l’action échoue / est annulée ⇒ `FAILURE`.
  - Si l’action est encore en cours ⇒ `RUNNING`.
- `onHalted()` sert à annuler le goal si l’arbre stoppe ce nœud (par exemple parce qu’une autre branche a échoué).

### 3.3. Intégration dans l’arbre XML

Dans le fichier `trees/<quelque_chose>.xml`, on déclare un nœud d’action dont l’`ID` correspond au nom utilisé lors de l’enregistrement dans le factory :

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="ComputeFibonacciSequence">
      <Action ID="FibonacciActionNode" order="10"/>
      <!-- autres Actions / Conditions -->
    </Sequence>
  </BehaviorTree>
  <!-- possibilité de définir d'autres sous-arbres -->
  </root>
```

Dans le code C++ du node principal, on enregistre cette classe auprès du `BehaviorTreeFactory` :

```cpp
BT::BehaviorTreeFactory factory;

factory.registerBuilder<FibonacciActionNode>(
  "FibonacciActionNode",
  [&node](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<FibonacciActionNode>(name, config, node);
  });

auto tree = factory.createTreeFromFile("path/vers/trees/fibonacci_tree.xml");
```

### 3.4. Avantages et inconvénients de l’approche manuelle

- **Avantages** :
  - Compréhension fine de la mécanique interne (tick, états, cancel, timeouts).
  - Contrôle total sur la façon dont on gère les transitions ROS 2 ↔ BT.
  - Pas de dépendance supplémentaire autre que `BehaviorTree.CPP` et les libs ROS 2 de base.

- **Inconvénients** :
  - Plus verbeux : il faut écrire tout le code du client d’action (timings, callbacks, gestion d’erreurs).
  - Facile de réimplémenter plusieurs fois les mêmes patterns (action, service, topic, etc.).

Cette approche est idéale pour **apprendre** et pour des cas très spécifiques où on veut une logique d’intégration non standard.

---

## 4. Approche 2 – Utilisation de BehaviorTree.ROS2

Dans cette approche, on s’appuie sur la bibliothèque **BehaviorTree.ROS2**, qui fournit déjà des briques prêtes à l’emploi pour interfacer BehaviorTree.CPP avec ROS 2 (actions, services, topics, paramètres, etc.).

L’idée est de réutiliser des **templates de nœuds** génériques (par exemple `BT::RosActionNode`) qui encapsulent :

- La création du client d’action ROS 2.
- L’envoi du goal, la gestion des callbacks.
- La traduction des états de l’action (`ResultCode`, annulation, timeout) en statuts BehaviorTree.CPP.

### 4.1. Exemple conceptuel avec `BT::RosActionNode`

Pour une action Fibonacci, on peut dériver de `BT::RosActionNode` spécialisé sur le type d’action :

```cpp
class FibonacciRosActionNode
  : public BT::RosActionNode<example_interfaces::action::Fibonacci>
{
public:
  FibonacciRosActionNode(const std::string& name,
                         const BT::NodeConfig& conf,
                         const BT::RosNodeParams& params)
    : BT::RosActionNode<example_interfaces::action::Fibonacci>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("order") };
  }

  bool setGoal(Goal& goal) override
  {
    // Récupère la valeur `order` depuis le port et remplit le goal
    return getInput<int>("order", goal.order);
  }

  BT::NodeStatus onResultReceived(const WrappedResult& result) override
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};
```

La base `RosActionNode` se charge déjà de la majorité du travail (création du client, envoi du goal, monitoring).  
On ne fournit que la logique métier :

- comment construire le goal (méthode `setGoal`) ;
- comment interpréter le résultat (`onResultReceived`) ;
- éventuellement comment gérer les cas de timeout / annulation via les hooks fournis.

### 4.2. Dépendances côté CMake

Pour exploiter BehaviorTree.ROS2, il faut :

- Ajouter `behaviortree_ros2` dans `package.xml` (déjà le cas dans `ros2_integration`) :

```14:15:ros2_integration_bt_ws/ros2_integration/package.xml
  <depend>behaviortree_cpp_v3</depend>
  <depend>behaviortree_ros2</depend>
```

- Lier la cible C++ contre cette bibliothèque dans le `CMakeLists.txt` :

```20:28:ros2_integration_bt_ws/ros2_integration/CMakeLists.txt
add_executable(bt_node src/bt_node.cpp)

ament_target_dependencies(bt_node
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  # ajouter également:
  # behaviortree_ros2
)
```

Une fois cela en place, `#include <behaviortree_ros2/...>` fonctionne et les templates `RosActionNode`, `RosServiceNode`, etc. sont disponibles.

### 4.3. Intégration dans l’arbre XML

Comme pour l’approche manuelle, on enregistre la classe dans le `BehaviorTreeFactory` puis on la référence par son `ID` dans le XML.  
La différence est que beaucoup de nœuds BT spécifiques à ROS 2 peuvent déjà être fournis **en tant que plugins** par BehaviorTree.ROS2, ce qui permet d’écrire des arbres purement déclaratifs.

Exemple conceptuel de fragment XML :

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="ComputeFibonacciWithRosNode">
      <Action ID="FibonacciRosActionNode" order="10"/>
    </Sequence>
  </BehaviorTree>
</root>
```

Si l’Action Node est fourni par un plugin BehaviorTree.ROS2, il suffit de charger la bibliothèque de plugin (souvent via un paramètre ou un appel d’enregistrement spécifique) et d’utiliser l’`ID` correspondant dans le XML.

### 4.4. Avantages et inconvénients de l’approche BehaviorTree.ROS2

- **Avantages** :
  - Réduction du code « colle » entre ROS 2 et BehaviorTree.CPP (moins de répétition).
  - Approche plus déclarative : beaucoup de choses se décrivent dans le XML, les plugins étant déjà prêts.
  - Meilleure factorisation pour de grandes architectures (nombreux topics/actions/services).

- **Inconvénients** :
  - Ajoute une dépendance supplémentaire (`behaviortree_ros2`).
  - Abstraction plus élevée : certains détails internes (timing, callbacks) sont moins visibles au premier abord.

Cette approche est idéale pour des systèmes plus grands ou lorsque l’on souhaite rapidement assembler des comportements complexes en se concentrant sur la logique métier plutôt que sur la plomberie ROS 2.

---

## 5. Comparaison synthétique des deux approches

- **Approche manuelle (BT + rclcpp_action)** :
  - Recommandée pour :
    - Comprendre en profondeur comment fonctionnent les actions, les ticks et les transitions de statut dans BehaviorTree.CPP.
    - Les cas où tu as besoin d’un contrôle très fin sur la logique d’intégration.
  - Plus verbeuse, mais très pédagogique.

- **Approche BehaviorTree.ROS2** :
  - Recommandée pour :
    - Projets de taille moyenne à grande.
    - Architectures où l’on veut décrire le plus possible dans des arbres XML et des plugins réutilisables.
  - Diminue la quantité de code C++ à écrire et favorise la modularité.

Dans ce workspace, tu peux très bien :

- garder le package `ros2_integration` comme **terrain d’expérimentation commun**, avec plusieurs exécutables (par exemple `bt_fibonacci_manual`, `bt_fibonacci_ros2`), ou  
- créer un second package (ex. `ros2_integration_ros2_plugins`) dédié aux exemples basés uniquement sur BehaviorTree.ROS2.

---

## 6. Étapes pratiques recommandées à suivre

Pour exploiter ce workspace sur l’exemple Fibonacci, tu peux suivre cet enchaînement :

1. **Serveur d’action Fibonacci** :
   - Reprendre presque tel quel le code du tutoriel officiel ROS 2 Humble (C++) pour l’action Fibonacci.
   - Le placer dans un package (par exemple `fibonacci_action_server`) de ton workspace ROS 2 global.

2. **Approche 1 – Client BT manuel** :
   - Dans `ros2_integration`, ajouter un exécutable (par exemple `bt_fibonacci_manual`) qui :
     - crée un node ROS 2,
     - définit un `BehaviorTreeFactory`,
     - enregistre un `FibonacciActionNode` basé sur `rclcpp_action::Client`,
     - charge un XML simple utilisant cette action.

3. **Approche 2 – Client BT via BehaviorTree.ROS2** :
   - Dans le même package ou dans un second package, ajouter un exécutable (par exemple `bt_fibonacci_ros2`) qui :
     - crée un node ROS 2 et un `BehaviorTreeFactory`,
     - enregistre un `FibonacciRosActionNode` dérivé de `BT::RosActionNode`,
     - charge un XML similaire mais basé sur cet Action Node.

4. **Comparaison** :
   - Lancer successivement les deux exécutables BT (avec le serveur d’action actif) et observer :
     - le comportement de l’arbre (loggers, sorties),
     - la quantité de code spécifique à ROS 2 que tu as dû écrire dans chaque approche.

Ce README sert de fil conducteur théorique. Les fichiers C++ et XML associés (serveur d’action, Action Nodes, arbres de comportement) peuvent être ajoutés progressivement dans `ros2_integration/src/` et `ros2_integration/trees/` en suivant les modèles fournis par les tutoriels BehaviorTree.CPP et ROS 2.


