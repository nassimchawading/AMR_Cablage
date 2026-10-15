# 2. L'atelier de câblage simulé

## Implantation générale
Hall de **20 m × 12 m**, organisé autour d'une **allée principale** de 2,5 m (couloir vert bordé
de lignes jaunes) qui traverse l'atelier d'ouest en est. Le flux de production suit un **U** :

```
 Nord   MAGASIN  →  COUPE / DÉNUDAGE  →  SERTISSAGE
                                              ↓
 ═══════════════════ allée principale ═══════════════════  (DOCK de recharge à l'ouest)
                                              ↓
 Sud    EXPÉDITION  ←  TEST ÉLECTRIQUE  ←  ASSEMBLAGE
        (quai)        (cellule contrôle qualité)
```

| Poste | Rôle dans la fabrication d'un faisceau | Équipement simulé |
|---|---|---|
| Magasin | stockage des bobines de câble (porte de réception à l'ouest) | rayonnages avec bobines, tourets au sol, palette |
| Coupe / dénudage | coupe des fils à longueur et dénudage des extrémités | 2 machines de coupe avec dévidoir, établi, bac à fils |
| Sertissage | sertissage des contacts sur les fils | établi avec 3 presses de sertissage, bacs de composants |
| Assemblage | montage du faisceau sur planche | 2 planches de câblage inclinées, armoire |
| Test électrique | test de continuité + **contrôle qualité** | bancs de test, convoyeur, caméra, bras UR5 |
| Expédition | emballage et départ des faisceaux (porte de quai au sud) | palettes de cartons, table d'emballage |
| Dock | recharge du robot | borne de recharge |

Chaque poste possède une **table de dépose** basse (0,45 m, à portée du bras du robot) placée en
bordure d'allée, avec un **marqueur ArUco** (identifiant 0 à 7) sur sa face avant : la caméra du
robot l'utilise pour identifier le poste et s'aligner précisément.

## Cellule de contrôle qualité (zone Test électrique)
1. Les opérateurs testent les faisceaux et les déposent, en bac, sur un **convoyeur d'inspection**.
2. Une **caméra** fixée sur un portique au-dessus du convoyeur observe chaque bac : une étiquette
   **verte** signifie conforme, **rouge** non conforme.
3. Un **bras UR5 à ventouse** trie les bacs : conformes → table « OK » (le robot les emporte vers
   l'Expédition), non conformes → table « REBUTS » (le robot les ramène à l'Assemblage pour retouche).

Cette cellule illustre l'idée d'« atelier intelligent » : l'atelier **génère lui-même** des ordres de
transport pour le robot.

## Construction de la scène : une seule source de vérité
L'atelier n'est pas dessiné à la main : il est **décrit sous forme de données** dans
`src\layout.py` (position, taille, couleur de chaque élément), puis construit automatiquement dans
CoppeliaSim par `src\build_scene.py`.

Avantages :
- la **carte de navigation** du robot (partie suivante) sera générée à partir des **mêmes données** :
  la scène 3D et la carte ne peuvent pas diverger ;
- toute modification (déplacer un poste, élargir une allée) se fait en changeant un nombre puis en
  reconstruisant la scène.

## Utilisation des modèles de la bibliothèque CoppeliaSim
Les éléments génériques proviennent de la bibliothèque de CoppeliaSim : sol en béton, murs de
2,40 m, fenêtres, portes automatiques, poteaux, grillages de séparation, rayonnages, armoires,
tables, convoyeur à bande, bras UR5, ventouse, caméra, marqueurs ArUco, opérateurs (« Bill »).
Les éléments propres au câblage, absents de la bibliothèque (machines de coupe, presses de
sertissage, planches de câblage, bobines), sont modélisés par des formes simples.

Avant utilisation, chaque modèle a été **mesuré** automatiquement (`src\inspect_models.py` :
dimensions, position de l'origine, scripts embarqués) afin d'être placé exactement. Plusieurs modèles
ont dû être adaptés par programme :
- convoyeur et marqueurs ArUco : paramètres stockés dans le modèle (longueur, vitesse, identifiant)
  modifiés puis script du modèle relancé ;
- tables : dimensions écrites dans le script du modèle, modifiées par programme ;
- bras UR5 : suppression du script de démonstration, montage de la ventouse sur la bride outil
  (calculée à partir du dernier axe du bras) ;
- caméra de contrôle : suppression du filtre de détection de taches intégré, afin d'obtenir l'image
  brute qui sera analysée en Python.

## Vérifications
- Rendus de contrôle générés automatiquement (`media\atelier_*.png`, `media\zoom_*.png`).
- Simulation de 10 s : le convoyeur transporte réellement les bacs jusqu'à la butée, le bras reste
  immobile, la caméra de contrôle voit le bac et son étiquette (`media\camera_controle_qualite.png`).
