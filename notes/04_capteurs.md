# 4. Les capteurs du robot

Le youBot de la bibliothèque n'a aucun capteur : ils ont été ajoutés par programme
(`src\equip_youbot.py`), à partir de modèles de la bibliothèque CoppeliaSim.

| Capteur | Modèle | Position | Rôle |
|---|---|---|---|
| Lidar 2D | Hokuyo URG-04LX (version « fast ») : 240°, 684 mesures, portée 5 m | avant, à 13 cm du sol | détection des obstacles imprévus, évitement |
| 8 capteurs à ultrasons | capteurs de proximité du Pioneer P3-DX, recopiés | tout autour de la base | arrêt de sécurité à courte portée, dans toutes les directions (le robot peut reculer et se déplacer latéralement) |
| Caméra | capteur de vision intégré, 640 × 480, champ 60° | avant, à 22 cm du sol | flux vidéo en direct, lecture des marqueurs ArUco |

## Traitement des données (`src\sensors.py`)
- **Lidar** : le modèle Hokuyo « fast » utilise deux capteurs de profondeur de 120°. Python lit
  leurs mesures, les convertit (avec numpy) en points dans le repère du robot (avant / gauche),
  supprime les rayons sans obstacle et les points situés sur le robot lui-même.
- **Ultrasons** : distance mesurée par chaque capteur, ou « rien » si aucun obstacle dans la portée.
- **Caméra** : image convertie en tableau numpy au format d'OpenCV (BGR), prête pour l'affichage
  et le traitement d'image.

## Problème rencontré
Le capteur à ultrasons avant, placé d'abord à 15 cm de haut, ne détectait rien : il se trouvait
juste derrière le boîtier du lidar, qui masquait son champ. Il a été remonté à 25 cm.

## Couverture des capteurs (vue de dessus)
Pour voir ce que le robot perçoit vraiment, la vue de dessus (`draw_scan` dans `src\sensors.py`,
utilisée par la fenêtre de démonstration et par `tests\test_sensors.py --live`) dessine les champs
des capteurs. Les valeurs ne sont pas recopiées à la main : elles sont **lues dans la scène** au
démarrage (position et orientation de chaque capteur par rapport au robot, volume de détection des
capteurs de proximité, angle et portée des capteurs de profondeur du lidar).

| Capteur | Champ lu dans la scène |
|---|---|
| Lidar | 2 capteurs de profondeur de 120° = 240° vers l'avant, portée 5 m |
| Chaque ultrason (× 8) | cône de 30° d'ouverture, portée 1,0 m, monté au bord de la base |

- zone bleue : champ du lidar ;
- cônes gris : ultrasons sans écho ; cône orange et arc épais : écho, à la distance mesurée
  (un ultrason donne seulement une distance, l'obstacle est quelque part sur cet arc) ;
- zones restées blanches : aucun capteur ne voit (zones aveugles).

La vue montre clairement deux limites : l'arrière du robot n'est couvert que par trois cônes
d'ultrasons de 30°, et il reste des secteurs vides entre les cônes, surtout près du robot.

## Pas de lidar arrière (choix assumé)
Le robot n'a qu'un lidar, à l'avant. Ce choix a été étudié puis conservé :

- **Le capteur est fait ainsi** : le Hokuyo URG-04LX couvre 240° par construction ; il ne peut pas
  voir derrière lui.
- **Le robot se masque lui-même** : monté à 13 cm du sol à l'avant, le lidar aurait de toute façon
  le corps du youBot dans son champ vers l'arrière.
- **Ce que font les AMR industriels** : soit deux lidars de sécurité de 270° placés sur deux coins
  opposés, dont les mesures sont fusionnées en une vue à 360° ; soit un seul lidar en hauteur sur un
  mât. Le mât ne convient pas ici : le bras et le bac transporté seraient dans son champ.
- **Pourquoi ce n'est pas fait dans ce projet** : coût de simulation (chaque Hokuyo « fast » =
  deux capteurs de vision rendus à chaque pas), risque de collision entre un capteur au coin arrière
  et la plateforme / le bac pendant les manipulations du bras, et effort d'intégration (fusion des
  deux nuages de points, nouveaux tests).
- **Comment l'arrière est couvert** : par l'anneau d'ultrasons (trois cônes vers l'arrière, portée
  d'environ 1 m), dont les échos non expliqués par la carte sont traités comme des obstacles, et par
  l'arrêt d'urgence : si un ultrason orienté dans le sens du déplacement mesure moins de 22 cm, le
  robot s'arrête.
- **Limite acceptée** : une personne qui arrive par l'arrière est détectée tard (à 1 m au plus, et
  seulement si elle entre dans un cône) et ne peut pas être suivie : un ultrason donne une distance,
  pas une position précise ni une vitesse.
- **Évolution possible** : un second lidar sur le coin arrière opposé, fusionné avec le premier,
  comme sur les AMR du commerce.

## Validation (`tests\test_sensors.py`)
Scène de test avec trois obstacles connus : une caisse à 1,2 m devant, un poteau à 0,7 m à gauche,
un mur à 1 m derrière.

| Mesure | Obtenu | Attendu |
|---|---|---|
| Lidar : face avant de la caisse | 0,950 m | 0,95 m |
| Lidar : surface du poteau | 0,641 m | 0,64 m |
| Ultrason avant / gauche / arrière | 0,65 / 0,42 / 0,65 m | 0,65 / 0,42 / 0,65 m |
| Caméra | image 640 × 480 de la caisse | — |

Images : `media\camera_robot.png` (vue caméra), `media\lidar_scan.png` (vue de dessus :
points lidar en rouge, champs des capteurs comme décrit plus haut). Un mode d'affichage en direct
(`tests\test_sensors.py --live`) montre le flux caméra et le balayage lidar en temps réel.
