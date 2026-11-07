# 10. Interface : suivi en direct et commande des ordres

L'atelier se suit et se pilote pendant qu'il tourne, sans outil externe : tout est fait en Python avec
OpenCV (fenêtres d'images) et Tkinter (fenêtre de commande, fournie avec Python).

## Fenêtres de suivi (`src\live_view.py`)
Mises à jour toutes les 3 étapes de simulation (0,15 s simulées) :
- **caméra** du robot, avec les marqueurs ArUco détectés (code, distance, angle) ;
- **lidar + sonars** : vue de dessus autour du robot, champ du lidar (240°), cônes réels des 8 sonars
  (orange quand ils détectent), points du lidar, objets suivis (orange = en mouvement, avec leur
  position prédite) ;
- **carte** de l'atelier : route A* prévue (bleu), trajet parcouru (vert), obstacles imprévus vus
  (rouge), objets suivis et prédictions, position vraie des opérateurs (cercles), robot ; avec une
  **légende** ;
- **ordres** : temps du poste, ce que fait le robot (ordre en cours, emplacements, état, « URGENT »),
  barre de batterie (vert / orange / rouge), file d'attente avec la classe de priorité et la mention
  « pas prêt », derniers ordres (terminé / refusé / échoué / annulé), compteurs de la cellule CQ et des
  opérateurs.

**Mode mosaïque** (`src\shift.py --mosaic`) : les quatre vues dans **une seule fenêtre 2 × 2**
(960 × 720), plus lisible pour une démonstration ; la légende de la carte est placée sous la carte.
**Enregistrement** (`--record poste.mp4`) : la mosaïque est enregistrée en vidéo dans `media\`, à la
cadence du temps simulé (6,7 images par seconde simulée) : la vidéo est fluide même si la simulation
tourne plus lentement que le temps réel. Essai : 1,9 min simulée → 760 images
(`media\essai_mosaique.mp4`).

## Fenêtre de commande (`src\order_app.py`)
Voir la note 08 (étape 8.6) : passer un ordre (urgent ou non), annuler un ordre en attente, mettre en
pause les ordres scriptés, envoyer le robot en charge, voir la file et les derniers ordres.

## Limites et suite
- Les fenêtres ralentissent la simulation (environ 5 fois plus lente que le temps réel avec quatre
  fenêtres) : `--no-live` pour les mesures longues de la partie 11.
- Prévu : une caméra « de poursuite » (vue 3D qui suit le robot) en cinquième vue, et le montage de la
  vidéo de démonstration (partie 11).
