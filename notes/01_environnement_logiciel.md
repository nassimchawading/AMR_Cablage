# 1. Environnement logiciel

## Répartition des rôles
- **CoppeliaSim** simule le monde : géométrie de l'atelier, moteur physique (Bullet), robot,
  capteurs (lidar, ultrasons, caméras).
- **Python** est le programme de commande du robot, exécuté **en dehors** du simulateur.
  Il lit les capteurs et envoie les consignes (vitesses des roues, positions du bras).

Ce découpage correspond à un robot réel : le simulateur remplace le matériel, et le même code
Python pourrait en principe piloter un vrai robot via une autre interface.

## Communication : API distante ZMQ
Le paquet `coppeliasim-zmqremoteapi-client` ouvre une connexion réseau locale (port 23000) vers
CoppeliaSim. Toutes les fonctions de l'API de CoppeliaSim (`sim.getObjectPosition`,
`sim.setJointTargetVelocity`, `sim.readProximitySensor`, …) deviennent appelables depuis Python.

## Mode pas-à-pas (synchrone)
La simulation est lancée en **mode pas-à-pas** (`sim.setStepping(True)`) : le temps simulé
n'avance que lorsque Python appelle `sim.step()` (un pas = 50 ms). Chaque cycle de commande suit
donc le schéma **lire les capteurs → décider → commander → avancer d'un pas**. Le programme Python
ne peut ni « rater » un pas de simulation ni prendre du retard sur le monde simulé, ce qui rend les
essais **reproductibles**.

## Architecture logicielle : modulaire, inspirée de ROS, sans middleware
Le projet n'utilise pas ROS (Robot Operating System). L'architecture en reprend toutefois le
principe : les fonctions du robot sont **séparées en modules indépendants** (perception,
planification, contrôle, manipulation), chacun dans son propre fichier Python, qui communiquent
par simples appels de fonctions au lieu de messages ROS (*topics*). C'est une **architecture
modulaire inspirée de ROS, sans middleware**.

| Rôle | Module | Équivalent dans l'écosystème ROS |
|---|---|---|
| Carte d'occupation | `occupancy.py` | `map_server` |
| Planification globale (A*) | `planner.py` | planificateur de `nav2` |
| Suivi de trajet et évitement | `navigator.py` | contrôleur de `nav2` |
| Localisation (UWB simulé + gyroscope) | `youbot.py` | `robot_localization` |
| Détection ArUco et accostage | `markers.py`, `docking.py` | `aruco_ros`, accostage |
| Cinématique inverse du bras | `arm.py` | `MoveIt` |
| Cellule de contrôle qualité | `qc_cell.py`, `factory.py` | nœud dédié |

Raisons de ce choix :
- **Windows** : ROS 2 est conçu avant tout pour Linux ; sous Windows son installation est lourde.
- **Compréhension** : les algorithmes (A*, évitement, cinématique inverse) sont écrits et
  expliqués ici, au lieu de configurer des paquets existants.
- **Simplicité** : un seul langage, une seule connexion au simulateur, trois paquets Python.

Évolution possible : CoppeliaSim dispose d'une interface ROS 2 (`simROS2`). Chaque module
pourrait devenir un nœud ROS 2 sans changer ses algorithmes.

## Installation
- Environnement virtuel Python (`.venv`) propre au projet : aucune installation globale.
- Toutes les installations sont consignées dans `INSTALL_LOG.md` (version, emplacement, commande
  de désinstallation).

## Test de validation
`tests\test_connection.py` : connexion à CoppeliaSim, démarrage de la simulation en mode
pas-à-pas, lecture du temps simulé de 0 à 3 s par pas de 0,05 s, arrêt. **Résultat : réussi.**
