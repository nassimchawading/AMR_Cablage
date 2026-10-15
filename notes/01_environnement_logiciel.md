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

## Installation
- Environnement virtuel Python (`.venv`) propre au projet : aucune installation globale.
- Toutes les installations sont consignées dans `INSTALL_LOG.md` (version, emplacement, commande
  de désinstallation).

## Test de validation
`tests\test_connection.py` : connexion à CoppeliaSim, démarrage de la simulation en mode
pas-à-pas, lecture du temps simulé de 0 à 3 s par pas de 0,05 s, arrêt. **Résultat : réussi.**
