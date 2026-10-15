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
points lidar en rouge, ultrasons en orange). Un mode d'affichage en direct
(`tests\test_sensors.py --live`) montre le flux caméra et le balayage lidar en temps réel.
