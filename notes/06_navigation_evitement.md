# 6. Navigation parmi les opérateurs : suivi de trajet et évitement

## Opérateurs mobiles (`src\layout.py`, `src\build_scene.py`)
Trois opérateurs (modèle « Bill on path » de la bibliothèque) marchent en continu, chacun conçu pour
tester un comportement du robot :

| Opérateur | Trajet | Comportement testé |
|---|---|---|
| `operateur_traversee_ouest` | traverse l'allée entre Magasin et Coupe (x = 5,75 m, y 8,0 ↔ 3,6 m), 0,7 m/s, pauses de 6 s | prédiction de sa trajectoire : ralentir / s'arrêter tôt, passer derrière lui ou pendant sa pause |
| `operateur_traversee` | traverse l'allée entre Sertissage et Assemblage (x = 15 m, y 8,6 ↔ 3,4 m), 0,8 m/s, pauses de 6 s | idem |
| `operateur_arret` | entre dans l'allée devant la Coupe et y reste 12 s | objet **fixe** : re-planifier tout de suite |

**Pourquoi cette disposition** (modifiée pour l'étape de prédiction) : la première version avait un
opérateur qui parcourait toute l'allée principale, c'est-à-dire **le même chemin que le robot**. Le
robot ne pouvait que le suivre à 0,2 m/s ou le croiser de face : il y avait peu à prédire, et c'est
lui qui rendait certains trajets très lents (retour Coupe → dock : 125 s, 12 arrêts). Il a été
remplacé par un second opérateur qui **traverse** l'allée, sur le trajet Magasin → Coupe. Le
couloir de passage a été vérifié sur la carte d'occupation : à x = 5,75 m, une bande de 0,6 m de
large est libre de y = 0,05 à 8,30 m, à l'ouest de la ligne de grillages et de poteaux (x = 6,5 m).
Les deux opérateurs qui traversent n'attendent pas le robot à distance : c'est au robot de prévoir
leur trajectoire et de s'adapter. Ils ne s'arrêtent que si le robot est **juste devant eux** (centre
du robot à moins de 0,9 m devant eux, soit ~0,5 m entre les corps), comme le ferait une personne.

Réglages successifs (et pourquoi) :
- avec des opérateurs qui ne réagissaient **jamais**, le robot (0,45 m/s, plus lent qu'eux) était
  parfois rattrapé quelle que soit sa prédiction (contacts dans 3 essais sur 4) ;
- avec des pauses de 1,5 s, le robot n'avait jamais le temps de traverser le passage (~3 s) entre
  deux traversées de l'opérateur → **pauses de 6 s** à chaque extrémité : l'opérateur marche encore
  environ la moitié du temps ;
- un couloir de passage à x = 4,9 m (mieux visible depuis l'est) a été essayé puis abandonné : à
  ~1,35 m de la table du Magasin, l'opérateur perturbait l'accostage (prise du bac manquée deux fois).

Modifications apportées au modèle de la bibliothèque :
- trajet remplacé par le nôtre, vitesse et pauses réglées, « points de pause » d'origine supprimés ;
- les opérateurs sont rendus **détectables** par le lidar et les ultrasons (invisibles par défaut) ;
- ils sont **non « respondables »** : ils se déplacent par petits « téléportations » ; en contact
  physique avec le robot, le moteur physique produisait des forces énormes (blocage de CoppeliaSim).

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
Mission dock → Sertissage → Rebuts → dock, avec les 3 opérateurs en mouvement (première disposition
des opérateurs, avec l'opérateur de l'allée principale) :

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
- Amélioration possible : prédire le mouvement des personnes (vitesse estimée) pour anticiper
  (étape suivante, voir ci-dessous).

## Suivi des obstacles mobiles et prédiction (`src\tracker.py`)
La perception ci-dessus ne voit que des **points** : elle sait qu'il y a « quelque chose » mais pas
s'il s'agit d'une caisse posée ou d'une personne qui marche, ni dans quelle direction elle va. Le
suivi transforme ces points en **objets** avec une position et une vitesse.

1. **Regroupement (clustering)** : les points lidar non expliqués par la carte qui sont à moins de
   0,6 m les uns des autres forment un même objet (regroupement « single-linkage »). Le centre de
   l'objet est la moyenne de ses points.
2. **Filtre de Kalman** (OpenCV `cv2.KalmanFilter`), un par objet, modèle à **vitesse constante** :
   état (x, y, vx, vy), mesure (x, y) = le centre du groupe. À chaque pas (50 ms), chaque objet est
   prédit, puis associé au groupe le plus proche de sa prédiction (plus proche voisin, à moins de
   0,8 m). Un groupe non associé crée un nouvel objet ; un objet non revu pendant 1,5 s est
   supprimé (entre-temps sa position continue d'être prédite : une personne cachée un instant
   n'est pas oubliée).
3. **État** : un objet est confirmé après 4 détections ; il est **fixe** si sa vitesse reste sous
   0,15 m/s pendant 2 s, sinon **en mouvement**. Pour un objet en mouvement, la position prévue
   dans t secondes est position + vitesse × t.

Affichage : sur la carte de la fenêtre de démonstration, un objet en mouvement est dessiné en orange
avec une flèche vers sa position prévue 1,5 s plus tard ; un objet fixe est gris.

### Problème rencontré : le lidar voit les jambes
Le lidar est à 13 cm du sol : il voit les **jambes** des opérateurs, pas leur corps. En pleine
foulée les deux pieds sont à ~0,5 m l'un de l'autre. Avec un seuil de regroupement de 0,35 m,
chaque jambe devenait un objet : l'une presque immobile (pied au sol), l'autre à deux fois la
vitesse de marche. Résultat : erreur de vitesse de 0,4 m/s et 11,5 s d'objets « en mouvement »
qui ne correspondaient à personne. Corrections : seuil porté à 0,6 m (les deux jambes forment un
seul objet), fusion de deux objets qui se retrouvent à moins de 0,5 m, vitesse lissée davantage.

### Validation (`tests\test_tracking.py`)
Le robot reste immobile 60 s (simulées) et regarde les opérateurs marcher. Les objets suivis sont
comparés aux **vraies** positions et vitesses des opérateurs, lues dans le simulateur (mesure
possible uniquement en simulation). Un opérateur ne compte comme « visible » que s'il est dans le
champ du lidar **et** réellement touché par au moins 2 rayons (pas caché derrière une machine).

| Scène | Opérateur | Vu par le lidar | Suivi | Erreur de position | Erreur de vitesse (médiane) |
|---|---|---|---|---|---|
| Atelier | opérateur de l'allée | 13 s | 98 % | 15 cm | 0,13 m/s |
| Atelier | opérateur qui traverse | 43 s | 99 % | 13 cm | 0,06 m/s |
| Arène de démonstration | opérateur qui traverse | 60 s | 100 % | 8 cm | 0,02 m/s |

Objets « en mouvement » ne correspondant à aucun opérateur : 1,2 s (atelier), 0 s (arène). Dans
l'arène, le carton absent de la carte est bien classé **fixe**. L'erreur de position est surtout
un décalage constant : le lidar voit l'avant des jambes, la position du simulateur est le centre
du corps. Images : `media\suivi_atelier_cablage.png`, `media\suivi_test_youbot.png`.

### Utilisation dans la navigation
Le navigateur utilise maintenant les objets suivis (option `predict`, activée par défaut ;
`--no-predict` dans `src\demo_youbot.py` et `tests\test_errand.py` redonne l'ancien comportement,
pour comparer) :

- **Objet fixe sur la route** (ex. un carton) : le robot s'arrête et re-planifie après 0,5 s au lieu
  d'attendre 3 s. Un objet fixe ne partira pas tout seul, attendre ne sert à rien.
- **Personne en mouvement** : ses positions prévues sur les 2 prochaines secondes (tous les 0,25 s)
  sont ajoutées aux obstacles. Le robot ralentit ou s'arrête donc **avant** que la personne
  n'entre dans son couloir, et non quand elle y est déjà.
- **Traînée de mémoire** : une personne qui marche laisse derrière elle des points mémorisés
  pendant 1,5 s. Le suivi sait où elle est maintenant : ces anciens points sont supprimés (ils
  bloquaient le robot derrière elle).
- **Personne qui s'éloigne sur la route** (vitesse le long de la route > 0,25 m/s) : ce n'est plus
  une raison de s'arrêter. Le robot la suit à distance (écart visé 1,4 m, vitesse adaptée), comme
  un régulateur de distance de voiture.
- La distance à « ce qui est vraiment là » (arrêt à moins de 0,55 m, vitesse limitée à 0,2 m/s à
  moins de 1,2 m, arrêt d'urgence par ultrasons) n'est pas modifiée : les prédictions ajoutent de la
  prudence, elles n'en retirent pas.

Résultats (une exécution chacun) :

| Essai | Résultat | Remarque |
|---|---|---|
| Arène de démonstration (`src\demo_youbot.py`) | réussi, 9/9 étapes, 0 contact, 186 s (189 s avant) | arrêt devant le carton à 14,6 s, re-planification à 15,2 s (au lieu de 3 s d'attente) |
| Atelier (`tests\test_errand.py`) | réussi, 0 contact, 172 s (172-190 s avant) | trajet vers Coupe : 4 arrêts courts (1 à 1,4 s) au lieu de 2, le robot s'arrête plus tôt |

(Ces deux essais datent de la première disposition des opérateurs, qui s'arrêtaient pour le robot.)

### Opérateurs qui ne s'arrêtent pas : comparaison avant / après
Avec la nouvelle disposition (deux opérateurs qui traversent l'allée **sans s'arrêter** pour le
robot), la même mission (dock → Magasin, chargement → Coupe, déchargement → dock) a été lancée sans
les prédictions (`--no-predict`, comportement d'avant), puis avec. Une exécution par version.

| Mission dans l'atelier | Sans prédiction | Prédiction, 1re version | Prédiction, version finale |
|---|---|---|---|
| Durée totale de la mission | 185,6 s | 176,7 s | **167,7 s** |
| Magasin → Coupe | 55,0 s ; 4 arrêts ; 1 re-planif. ; dist. min. 0,35 m | 48,1 s ; 2 arrêts ; 2 re-planif. ; 0,58 m | **40,2 s** ; 2 arrêts ; 0 re-planif. ; 0,72 m |
| Coupe → dock | 58,7 s ; 4 arrêts ; 3 re-planif. ; dist. min. **0,07 m** | 56,3 s ; 2 arrêts ; 2 re-planif. ; 0,33 m | 55,4 s ; 3 arrêts ; 3 re-planif. ; 0,46 m |
| Pas de simulation avec contact (robot en mouvement / arrêté) | 25 / 39 | 9 / 12 | **0 / 0** |

Sans prédiction, le robot ne réagit qu'à ce que la personne **est** : il s'arrête quand elle est
déjà dans son couloir, souvent au milieu de son passage, et elle lui marche dessus (39 pas de
contact robot arrêté ; distance minimale 7 cm entre les centres, donc chevauchement des corps).

La première version de la prédiction divisait les contacts par trois sans les supprimer. Le journal
du trajet de retour montrait trois défauts, corrigés dans la version finale :
1. **Horizon trop court** : 2 s de prédiction, soit 1,4 m pour un opérateur à 0,7 m/s, alors que le
   robot met ~2,5 s à traverser son passage → horizon porté à **3 s**.
2. **Arrêt trop près du passage** : le robot s'arrêtait à 0,75 m du point de croisement prévu,
   l'opérateur le frôlait ou le touchait → pour un croisement **prévu**, le robot s'arrête **0,5 m
   plus tôt** que pour un obstacle réel : il attend en dehors du passage.
3. **Fuite inutile** : une personne qui passe **devant** le robot arrêté s'en rapproche un instant ;
   la règle « quelqu'un se rapproche → s'écarter » faisait reculer le robot plusieurs fois de suite.
   Si la trajectoire prévue de la personne reste à plus de 0,8 m du robot, elle va passer : pas de
   manœuvre d'écart, et pas de limitation à 0,2 m/s à cause d'elle.
De plus, si le robot est **déjà** dans le passage quand la personne repart (fin de sa pause), il ne
s'arrête pas sur place : il continue pour en sortir.

Une seule exécution par version : les chiffres de temps varient d'un essai à l'autre (les
opérateurs ne sont jamais au même endroit au même moment), mais l'écart sur les contacts et la
distance minimale est net. À compléter par plusieurs exécutions par version pour des moyennes.

Mais dans d'autres essais avec ces opérateurs qui ne réagissent jamais, la version finale a encore
eu des contacts (3 essais sur 4) : opérateur caché derrière une machine jusqu'au dernier moment,
robot pris au milieu d'un passage, manœuvre d'écart vers la trajectoire de la personne. Le robot
étant plus lent que les opérateurs, la disposition a été rendue plus réaliste (voir « Opérateurs
mobiles » en haut de cette note : arrêt quand le robot est juste devant eux, pauses de 6 s).

### Disposition finale : comparaison avant / après
Même mission, opérateurs qui traversent sans attendre le robot mais s'arrêtent s'il est juste devant
eux, pauses de 6 s. Une exécution par version.

| Mission dans l'atelier | Sans prédiction | Prédiction (1er réglage) | Prédiction (réglage final) |
|---|---|---|---|
| Durée totale | 167,6 s | 208,6 s | **162,4 s** |
| Magasin → Coupe | 43,5 s ; 2 arrêts ; 1 re-planif. ; dist. min. 0,74 m | 71,3 s ; 4 arrêts ; 3 re-planif. ; 0,84 m | **42,0 s** ; 3 arrêts ; 1 re-planif. ; 0,86 m |
| Coupe → dock | 51,8 s ; 1 arrêt ; 1 re-planif. ; 0,77 m | 65,0 s ; 6 arrêts ; 2 re-planif. ; 0,82 m | **48,6 s** ; 3 arrêts ; 1 re-planif. ; 0,82 m |
| Contacts | 0 | 0 | 0 |

Avec des opérateurs qui s'écartent au dernier moment, le comportement réactif d'origine suffit
déjà : aucun contact. Le premier réglage de la prédiction rendait le robot plus prudent (≈ 10 cm de
marge en plus) mais **40 s plus lent**, pour deux raisons, corrigées dans le réglage final :
1. **Arrêt pour tout croisement prévu**, même quand le robot serait passé bien avant l'arrivée de la
   personne. Désormais un croisement prévu ne compte que si la personne y arrive **au moment où le
   robot y passe** : le robot arrive au point de croisement en d / v_max et l'a dépassé 0,8 m plus
   loin ; si la personne n'y est prévue que plus d'1 s après, ce n'est pas un conflit, le robot
   continue (sans s'arrêter ni ralentir à 0,2 m/s à cause d'elle).
2. **Re-planification immédiate autour d'un opérateur à l'arrêt** (en pause au bout de sa traversée,
   ou l'opérateur qui reste 12 s dans l'allée) : la règle « objet fixe → re-planifier tout de suite »
   était faite pour un carton, elle provoquait des détours (10,2 m au lieu de 6,9 m vers la Coupe).
   Le suivi retient maintenant si un objet a **déjà bougé** (vitesse > 0,3 m/s) : seul un objet qui
   n'a jamais bougé déclenche la re-planification immédiate ; une personne arrêtée va repartir, le
   robot attend (re-planification après 3 s si elle ne bouge pas).

Un second essai du réglage final a duré 175,4 s (Coupe 41,3 s ; retour 62,0 s, dont un détour
autour de l'opérateur arrêté devant la Coupe ; distance minimale 0,78-0,82 m ; 0 contact) : les
durées varient d'un essai à l'autre selon l'endroit où se trouvent les opérateurs.

Avec le réglage final, la prédiction met donc **à peu près le même temps** que le comportement
réactif (162-175 s contre 168 s) et reste **plus loin des opérateurs** (distance minimale
0,78-0,86 m contre 0,74-0,77 m). Conclusion pour le rapport : face à des personnes qui ne font pas
attention, la prédiction apporte surtout de la sécurité (contacts divisés par trois, distance
minimale 0,07 → 0,33-0,46 m) ; face à des personnes attentives, bien réglée, elle ne coûte pas de
temps et garde plus de marge. Peu d'essais par version : des moyennes sur plusieurs exécutions
rendraient la comparaison plus solide.

## Obstacle fixe absent de la carte : le carton près du Sertissage
Un carton est posé dans l'allée sur le chemin du Sertissage, sans figurer sur la carte (position et
raisons du choix : voir la note 02). Essai (trajet vers le Sertissage, départ à x = 10,5 m) :

- le suivi classe le carton **fixe** 2,7 s après l'avoir vu ;
- le robot s'arrête à 0,74 m du carton (x = 11,9 m, hors du passage de l'opérateur à x = 15 m) et
  **re-planifie 0,5 s plus tard** (« objet fixe ») au lieu d'attendre 3 s ; détour de 6,8 m par la
  moitié sud de l'allée, Sertissage atteint en 53,8 s, 0 contact (image :
  `media\carton_sertissage_carte.png`).
- Limite visible sur l'image : quand le robot tourne autour du carton, le centre de ses points se
  déplace et le carton peut apparaître un instant « en mouvement » à ~0,2 m/s. Cela ne change pas
  la décision (il n'a jamais parcouru 0,6 m, il reste un objet qui « n'a jamais bougé »).

Deux corrections ont été nécessaires pour que le carton soit bien reconnu comme « objet qui ne
bougera pas » :
1. Vu de loin (~5 m) puis sous des angles différents pendant que le robot roule, le carton montre
   d'autres faces : le centre de ses points se déplace un peu, ce qui ressemble à une vitesse. Un
   objet n'est donc plus considéré comme « ayant bougé » sur sa vitesse, mais sur la **distance
   réellement parcourue** depuis sa première détection (plus de 0,6 m : une personne qui marche
   parcourt des mètres, le centre visible d'un carton se décale au plus de ~0,3 m).
2. Pour la même raison, son état oscille entre « fixe » et « nouveau » quand le robot tourne autour :
   la re-planification rapide vaut pour tout objet **qui n'a jamais bougé et qui est suivi depuis au
   moins 2 s**, quel que soit cet état.

Une personne qui s'est arrêtée (par exemple l'opérateur qui reste 12 s dans l'allée devant la Coupe)
a, elle, déjà bougé : le robot l'attend (re-planification après 3 s seulement), car elle va repartir.
