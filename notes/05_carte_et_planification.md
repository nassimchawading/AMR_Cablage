# 5. Carte d'occupation et planification de trajectoire (A*)

## Carte d'occupation (`src\occupancy.py`)
Le sol de l'atelier est découpé en **cellules carrées de 5 cm** : 420 × 260 cellules pour un hall de
20 m × 12 m (murs compris). Chaque cellule est **libre** ou **occupée**.

La carte est **générée automatiquement à partir de la description de l'atelier** (`src\layout.py`),
c'est-à-dire des mêmes données que celles qui construisent la scène 3D : la carte est donc toujours
cohérente avec la scène.

Règles de construction :
- un objet n'occupe une cellule que s'il se trouve **entre le sol et la hauteur du robot (0,5 m)** :
  un écran sur un pied ne gêne pas le robot, mais le pied, oui ;
- les **personnes ne figurent pas sur la carte** : elles se déplacent, c'est le lidar qui les
  détecte pendant la navigation ;
- pour les modèles de la bibliothèque, l'emprise au sol est calculée à partir des dimensions et de la
  position de l'origine **mesurées** de chaque modèle.

Trois couches :
| Couche | Signification |
|---|---|
| occupée | les objets eux-mêmes |
| interdite | les objets **grossis du rayon du robot (0,40 m)** : le **centre** du robot ne doit pas y entrer. Le robot peut ainsi être traité comme un point. |
| coût de confort | coût supplémentaire qui décroît sur 0,45 m au-delà de la zone interdite : les trajets gardent une distance confortable aux obstacles (dans l'allée, ils restent près du milieu) |

Les 8 positions de chargement/déchargement (postes + dock) sont toutes en zone autorisée.

## Algorithme A* (`src\planner.py`)
A* explore la grille à partir de la cellule de départ en développant toujours en premier la cellule
de plus petit **f = g + h**, où :
- **g** est le coût du meilleur chemin connu du départ jusqu'à cette cellule ;
- **h** est une estimation de la distance restante jusqu'au but (distance « octile », adaptée aux
  déplacements en 8 directions) ; comme h ne surestime jamais le coût réel, le premier chemin qui
  atteint le but est **optimal**.

Détails :
- déplacements dans les **8 directions** (coût 1 en ligne droite, √2 en diagonale), sans « couper
  les coins » d'un obstacle ;
- coût d'un pas = longueur × (1 + 4 × coût de confort de la cellule) : compromis entre trajet court et
  distance de sécurité ;
- **lissage** : le chemin brut (un escalier de cellules) est simplifié en ne gardant que les points
  nécessaires pour que chaque segment droit reste en zone sûre (test de visibilité).

## Résultats (`tests\test_planner.py`)
- Carte construite en 0,01 s ; 19 % de la surface occupée, 61 % interdite au centre du robot.
- **28 trajets** calculés entre toutes les paires de postes : **aucun échec**, 15 ms en moyenne
  (41 ms au maximum).
- Exemples depuis le dock : Sertissage 15,8 m (316 cellules → 3 points de passage), Coupe 9,0 m,
  Test 9,3 m, Rebuts 10,4 m.
- Image : `media\carte_chemins.png` (noir : obstacles, gris : zone interdite, dégradé : coût de
  confort, couleurs : trajets depuis le dock).

Le robot youBot équipé (lidar, ultrasons, caméra) est désormais placé à son dock dans l'atelier,
prêt pour la partie suivante : le suivi de ces trajets et l'évitement des opérateurs.
