# 0. Présentation du projet

## Contexte
Projet du module **« Atelier intelligent »**. Il s'agit de simuler un **robot mobile autonome (AMR,
Autonomous Mobile Robot)** qui travaille dans une **usine de câblage** (fabrication de faisceaux
électriques). Le robot transporte la matière (bobines de câble, faisceaux, bacs) entre les postes
de l'atelier, planifie lui-même ses trajets à l'aide de ses **capteurs**, évite les obstacles — y
compris les **opérateurs qui se déplacent** — et transmet l'image de sa **caméra** en temps réel.

Aucune contrainte n'a été imposée par l'enseignant : les choix techniques ci-dessous sont les nôtres.

## Objectifs
1. Un atelier simulé réaliste, reconnaissable comme une usine de câblage.
2. Une **vraie planification de trajectoire** (pas un trajet codé en dur) à partir d'une carte.
3. Une **perception par capteurs** : lidar, capteurs à ultrasons, caméra.
4. L'**évitement des opérateurs mobiles** (ralentir, s'arrêter, contourner, replanifier).
5. Une **gestion de missions** (ordres de transport) et une petite interface de supervision.

## Choix de conception (décidés au démarrage)
| Sujet | Choix | Justification |
|---|---|---|
| Carte | Carte **fixe et connue** de l'atelier ; le lidar ajoute les obstacles imprévus | Situation réelle des AMR industriels ; pas besoin de SLAM |
| Capteurs | Lidar 2D + couronne d'ultrasons + caméra | Lidar = navigation/évitement ; ultrasons = arrêt de sécurité de proximité ; caméra = flux vidéo + marqueurs ArUco |
| Chemin bloqué | 1) ralentir / s'arrêter et attendre, 2) replanifier après quelques secondes | Comportement prudent et réaliste en présence d'humains |
| Localisation | Système de positionnement intérieur simulé (balises **UWB** + gyroscope), sans odométrie | Simple et fiable ; laisse le temps aux parties planification et évitement (amélioration possible : localisation par lidar) |
| Robot | **KUKA youBot** (roues mécanum + bras manipulateur) | Peut se déplacer latéralement (utile pour esquiver) et saisir réellement les bacs |

## Outils
- **CoppeliaSim Edu 4.10** : le « monde » (atelier, physique, robot, capteurs).
- **Python 3.15** : le « cerveau » (planification, commande, traitement d'image), relié à CoppeliaSim
  par l'API distante **ZMQ**.
- Bibliothèques Python : `coppeliasim-zmqremoteapi-client`, `numpy`, `opencv-python`.

## Architecture du « cerveau » (vue d'ensemble)
```
 capteurs (lidar, ultrasons, caméra, UWB)
          │
          ▼
 ┌─ gestionnaire de missions (ordres, machine à états, batterie) ◄── interface / cellule contrôle qualité
 │        │ « aller au poste X »
 │        ▼
 │   planificateur A* sur la carte ──► trajet
 │        │
 │        ▼
 │   suivi de trajet + évitement d'obstacles ──► commande des roues
 │        │ arrivé
 │        ▼
 └── accostage par ArUco + saisie/dépose avec le bras
```

## Démarche
Le projet est construit **partie par partie**, chaque partie étant **testée** avant de passer à la
suivante (voir les fichiers suivants de ce dossier et les scripts du dossier `tests\`).
