# 6. Navigation parmi les opérateurs : suivi de trajet et évitement

## Opérateurs mobiles (`src\layout.py`, `src\build_scene.py`)
Trois opérateurs (modèle « Bill on path » de la bibliothèque) marchent en continu, chacun conçu pour
tester un comportement du robot :

| Opérateur | Trajet | Comportement testé |
|---|---|---|
| `operateur_allee` | toute l'allée principale (x 2,6 → 18,6 m), 0,8 m/s | croisement de face, dépassement |
| `operateur_traversee` | traverse l'allée à x = 15 m | arrêt et attente |
| `operateur_arret` | entre dans l'allée devant la Coupe et y reste 12 s | attente puis contournement |

Modifications apportées au modèle de la bibliothèque :
- trajet remplacé par le nôtre, vitesse et pauses réglées, « points de pause » d'origine supprimés ;
- les opérateurs sont rendus **détectables** par le lidar et les ultrasons (invisibles par défaut) ;
- ils sont **non « respondables »** : ils se déplacent par petits « téléportations » ; en contact
  physique avec le robot, le moteur physique produisait des forces énormes (blocage de CoppeliaSim) ;
- **ils cèdent le passage au robot** : un opérateur s'arrête tant que le centre du robot est devant
  lui, à moins de 1,3 m et à moins de 0,7 m sur le côté (au plus 20 s, ensuite il repart). C'est le
  comportement attendu d'opérateurs formés à travailler à côté d'AMR. La norme ISO 3691-4 rend le
  robot responsable de ne pas heurter les personnes **lorsqu'il se déplace**.

Les opérateurs ne figurent **pas** sur la carte : le robot doit les découvrir avec ses capteurs.

## Perception des obstacles imprévus (`src\navigator.py`)
À chaque pas de simulation (50 ms) :
1. les points du lidar (240° vers l'avant, 5 m) et les détections des 8 ultrasons (tout autour,
   ~1 m) sont convertis en coordonnées « monde » grâce à la position estimée du robot ;
2. un point situé à moins de 15 cm d'un objet de la carte est **expliqué par la carte** et ignoré ;
   les autres sont des **obstacles imprévus** (en pratique : des personnes) ;
3. ces points sont **mémorisés 1,5 s** (cellules de 10 cm) : une personne qui sort du champ du lidar
   (sur le côté, derrière) n'est pas oubliée immédiatement.

## Machine à états
| État | Rôle |
|---|---|
| FOLLOW | suit le trajet A* : vise un point « carotte » 0,6 m plus loin, orienté dans le sens de la marche (le lidar regarde où va le robot). Obstacle dans le couloir de 0,5 m de demi-largeur devant : ralentit dès 1,8 m, s'arrête à 0,75 m. Personne à moins de 1,2 m : vitesse limitée à 0,2 m/s. Personne à moins de 0,55 m : arrêt. |
| WAIT | arrêté. Voie libre → reprend. Bloqué depuis 3 s → **re-planifie** avec A* en ajoutant les obstacles vus (rayon 0,35 m) à la carte. Personne qui se rapproche → ESCAPE. |
| ESCAPE | s'écarte de la personne en **crabe** (roues mécanum, sans tourner), dans une direction libre sur la carte. |
| CLEAR | attend que plus personne ne soit à moins de 1,2 m pendant 1 s, puis re-planifie. Si la personne ne s'éloigne pas, contourne-la. |
| FINAL | approche précise de la pose finale (position + orientation). |

Arrêt d'urgence : un ultrason orienté dans le sens du mouvement mesure moins de 0,22 m.

## Problèmes rencontrés et corrections
- **Personne oubliée** (v2) : un opérateur sortait du champ du lidar et était oublié → 3 contacts.
  Correction : ajout des ultrasons à la perception + mémoire de 1,5 s.
- **Blocage de CoppeliaSim** (v3) : opérateurs « respondables » téléportés dans le robot →
  rendus non respondables.
- **Obstacles fantômes au dock** : les lettres peintes au sol (« DOCK », « EXPEDITION ») étaient
  détectées par les ultrasons ; le robot s'arrêtait et « fuyait » un obstacle inexistant.
  Correction : marquages au sol rendus indétectables.
- **Fausse « personne qui se rapproche »** : juste après un arrêt, l'historique des distances
  contenait encore l'approche du robot lui-même. Correction : historique remis à zéro à chaque arrêt.
- **Lecture du lidar parfois vide** (l'API renvoie -1 pendant un pas) : le pas est ignoré, la
  mémoire des obstacles prend le relais.

## Résultats (`tests\test_navigation.py`)
Mission dock → Sertissage → Rebuts → dock, avec les 3 opérateurs en mouvement :

| Trajet | Atteint | Temps (simulé) | Distance | Arrêts | Écarts | Re-planifications | Distance min. à un opérateur | Contacts |
|---|---|---|---|---|---|---|---|---|
| dock → Sertissage | oui | 131 s | 21,7 m | 6 | 1 | 5 | 0,73 m | 0 |
| Sertissage → Rebuts | oui | 47 s | 6,1 m | 2 | 0 | 2 | 0,73 m | 0 |
| Rebuts → dock | oui | 51 s | 11,1 m | 1 | 1 | 1 | 0,73 m | 0 |

**Test réussi : 0 contact, 0 arrêt d'urgence.** La distance minimale est mesurée entre le centre du
robot et celui de l'opérateur (positions réelles, utilisées uniquement pour l'évaluation : le robot
n'y a pas accès). Image : `media\navigation_test.png` (bleu : trajet réel du robot ; orange :
trajets des opérateurs) ; journal complet : `media\navigation_test_log.txt`.

Limites :
- le premier trajet est **lent** (131 s pour une route de 15,8 m) : le robot suit l'opérateur de
  l'allée à 0,2 m/s, puis doit le contourner plusieurs fois quand celui-ci fait demi-tour ;
- une personne à moins de 15 cm d'un objet de la carte est considérée comme « connue » (l'arrêt
  d'urgence par ultrasons reste actif) ;
- la simulation tourne à ~0,4 × le temps réel.
- Amélioration possible : prédire le mouvement des personnes (vitesse estimée) pour anticiper.
