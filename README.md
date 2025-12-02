# BT_tutorials

## Objectif

Fournir un guide unique pour réaliser et lancer les tutoriels de https://www.behaviortree.dev/docs/tutorial-basics en vous appuyant sur des projets locaux similaires à `first_bt`.

## Prérequis système

- BehaviorTree.CPP v4 installé dans `/usr/local/share/behaviortree_cpp` avec la lib dans `/usr/local/lib` (détecté en 4.8.2).
- Dépendances ROS2 : `ament_index_cpp` (présent dans `/opt/ros/jazzy/lib`) et `tinyxml2`, `sqlite3`, `zmq` déjà fournies par le système.
- Outils : `cmake` >= 3.5, `g++` avec C++17.
- Pensez à exposer les libs dynamiques avant l’exécution :
  `export LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/jazzy/lib:$LD_LIBRARY_PATH`

## Arborescence proposée

```tree
BT_tutorials/
  README.md
  first_bt/              # exemple de base (Sequence simple)
  <nouveau_tutoriel>/    # un dossier par tutoriel
    CMakeLists.txt
    main.cpp
    my_tree.xml (ou autre nom)
    dummy_nodes.h / .cpp (vos noeuds custom)
    build/               # généré par CMake
```

## Lancer l’exemple existant `first_bt`

```bash
cd ~/BT_tutorials/first_bt
cmake -S . -B build
cmake --build build
./first_tree
```
Pourquoi deux commandes ?
- `cmake -S . -B build` configure le projet et génère les fichiers de build dans `build` (out-of-source build, cache CMake propre).
- `cmake --build build` lance la compilation en réutilisant cette configuration (appelle automatiquement le bon outil, make/ninja/MSBuild, selon le générateur).

Sortie attendue (séquence du tutoriel de base) :

```bash
[ Battery: OK ]
GripperInterface::open
ApproachObject: approach_object
GripperInterface::close
```

## Modèle de CMakeLists.txt

Utilisez ce squelette pour tout nouveau tutoriel :

```cmake
cmake_minimum_required(VERSION 3.5)
project(<nom_projet>)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(behaviortree_cpp_DIR "/usr/local/share/behaviortree_cpp/cmake")
find_package(behaviortree_cpp REQUIRED)
set(ament_index_cpp_DIR "/opt/ros/jazzy/share/ament_index_cpp/cmake")
find_package(ament_index_cpp REQUIRED)

add_executable(<binaire> main.cpp)
target_include_directories(<binaire> PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(<binaire>
    behaviortree_cpp::behaviortree_cpp
    ament_index_cpp::ament_index_cpp
)
```

Astuce : dupliquez `first_bt/CMakeLists.txt` puis remplacez le nom du projet et du binaire.

## Modèle de `main.cpp`

```cpp
#include "behaviortree_cpp/bt_factory.h"
#include <filesystem>
#include <iostream>
// ajoutez vos headers de noeuds ici (ex: dummy_nodes.h)

int main()
{
    BT::BehaviorTreeFactory factory;

    // Enregistrer vos types de noeuds ici (ex: factory.registerNodeType<MyAction>("MyAction");)

    const auto tree_file = std::filesystem::path(__FILE__).parent_path() / "my_tree.xml";
    if (!std::filesystem::exists(tree_file))
    {
        std::cerr << "Fichier XML introuvable: " << tree_file << std::endl;
        return 1;
    }

    auto tree = factory.createTreeFromFile(tree_file.string());
    tree.tickWhileRunning();
    return 0;
}
```

## Modèle de fichier XML minimal

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="root_sequence">
      <MyCondition   name="check_something"/>
      <MyAction      name="do_something"/>
    </Sequence>
  </BehaviorTree>
</root>
```

Adaptez les noms de noeuds aux enregistrements faits dans `main.cpp` (ou dans vos fichiers de noeuds).

## Procédure pour créer un nouveau tutoriel

1) Copier l’exemple : `cp -r first_bt <nouveau_tutoriel>` puis renommez `first_tree` dans CMake et le XML selon vos besoins.
2) Remplacer/étendre `dummy_nodes.h` (ou créer `dummy_nodes.cpp`) avec les actions/conditions du tutoriel.
3) Mettre à jour `my_tree.xml` pour décrire le flux du tutoriel.
4) Mettre à jour `main.cpp` pour enregistrer les nouveaux noeuds et charger le bon XML.
5) Construire et lancer :
   ```bash
   cd ~/BT_tutorials/<nouveau_tutoriel>
   cmake -S . -B build
   cmake --build build
   ./<binaire>
   ```

## Dépannage rapide

- `libament_index_cpp.so: not found` : vérifier `LD_LIBRARY_PATH=/opt/ros/jazzy/lib` avant l’exécution.
- `libbehaviortree_cpp.so: not found` : ajouter `/usr/local/lib` au `LD_LIBRARY_PATH`.
- `Fichier XML introuvable` : vérifier que le XML est dans le même dossier que `main.cpp` (ou adaptez le chemin).
- Reconfiguration nécessaire après modification de CMake : relancer `cmake -S . -B build` puis `cmake --build build`.
- Nettoyage minimal : supprimer ou renommer le dossier `build` si vous changez fortement la configuration.

## Rappel

Suivez l’ordre des tutoriels sur https://www.behaviortree.dev/docs/tutorial-basics et utilisez un dossier par étape. Chaque dossier reste autonome pour faciliter les essais, la compilation et l’exécution.
