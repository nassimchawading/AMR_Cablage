# 3. Le robot : KUKA youBot

## Choix du robot
Le **KUKA youBot** (modèle de la bibliothèque CoppeliaSim) a été retenu :
- base à **4 roues mécanum** : déplacement dans **toutes les directions** (avant, côté, rotation) ;
  utile pour esquiver un opérateur par un pas de côté sans tourner ;
- **bras à 5 axes + pince** : le robot pourra saisir réellement les bacs aux postes ;
- dimensions : 0,57 m × 0,39 m, hauteur 0,44 m bras replié — adaptées à une allée de 2,5 m.

Le modèle de la bibliothèque contient un script de démonstration (le robot glisse puis tourne sans
fin) : il a été supprimé pour que seul notre programme Python commande le robot. Les petits scripts
des roues, qui simulent l'effet des galets inclinés des roues mécanum, sont conservés.

## Cinématique des roues mécanum
Chaque roue mécanum porte des galets inclinés à 45°. En combinant les vitesses des quatre roues, on
obtient n'importe quel mouvement plan. Pour une vitesse souhaitée du robot
(v<sub>x</sub> vers l'avant, v<sub>y</sub> vers la gauche, ω en rotation), avec R le rayon des roues
(0,05 m), L<sub>x</sub> = 0,228 m et L<sub>y</sub> = 0,158 m les demi-distances entre roues :

| Roue | Vitesse angulaire |
|---|---|
| avant gauche | ω<sub>ag</sub> = (v<sub>x</sub> − v<sub>y</sub> − (L<sub>x</sub>+L<sub>y</sub>) ω) / R |
| arrière gauche | ω<sub>arg</sub> = (v<sub>x</sub> + v<sub>y</sub> − (L<sub>x</sub>+L<sub>y</sub>) ω) / R |
| arrière droite | ω<sub>ard</sub> = (v<sub>x</sub> − v<sub>y</sub> + (L<sub>x</sub>+L<sub>y</sub>) ω) / R |
| avant droite | ω<sub>ad</sub> = (v<sub>x</sub> + v<sub>y</sub> + (L<sub>x</sub>+L<sub>y</sub>) ω) / R |

Implémentation : `src\youbot.py`, fonction `move(forward, left, turn)`.

**Validation** (`tests\test_drive.py`, 2 s pour chaque mouvement) :

| Consigne | Mesuré |
|---|---|
| 0,4 m vers l'avant | 0,392 m |
| 0,4 m vers la gauche (latéral) | 0,388 m |
| rotation de 1,0 rad | 1,006 rad |

L'écart de 2 à 3 % correspond au glissement des roues. Le test a aussi montré que l'axe « avant »
du robot correspond à l'axe +y de son repère de référence (`youBot_ref`), ce qui a été pris en compte.

## Localisation : positionnement intérieur simulé (UWB + gyroscope)
Plutôt que l'odométrie (comptage des tours de roues, qui dérive à cause du glissement), on suppose
que l'atelier est équipé d'un **système de positionnement intérieur par balises UWB** (Ultra-Wide
Band), courant dans l'industrie, et que le robot possède un **gyroscope** pour son orientation.

En simulation : position réelle du robot fournie par CoppeliaSim **+ bruit gaussien**
(écart-type 2 cm sur la position, 0,5° sur le cap), pour reproduire l'imprécision d'un système réel.
Une amélioration possible est la localisation par lidar sur la carte connue (filtre particulaire,
méthode AMCL) ; le reste du programme n'en dépend pas.

## Commande en position
Fonction `go_to_step(cible)` : à chaque pas, l'erreur de position est exprimée dans le repère du
robot et convertie en vitesse proportionnelle à la distance restante (limitée à 0,3 m/s),
plus une correction de cap. Grâce aux roues mécanum, le robot va **en ligne droite** vers la cible
quelle que soit son orientation.

**Validation** (`tests\test_square.py`) : carré de 1 m parcouru **sans tourner** (avant, latéral
gauche, arrière, latéral droit), uniquement à partir de la position UWB bruitée.
Chaque côté est parcouru en environ 4 s ; erreur réelle aux coins : **4 à 5 cm**, limitée par le
bruit du positionnement. La précision finale aux postes sera assurée par les marqueurs ArUco.
Tracé : `media\test_square.png` (bleu : trajet réel, orange : mesures UWB).
