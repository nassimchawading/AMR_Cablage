# 8. Gestion des missions : ordres de transport

Le robot ne fait plus une mission écrite à l'avance : il reçoit des **ordres de transport** et les
exécute l'un après l'autre (`src\missions.py`). L'atelier en crée lui-même : c'est l'idée
d'« atelier intelligent ».

## Ordres et file d'attente
Un ordre contient : ce qui est transporté (« bobines », « faisceau OK »…), le poste de prise, le
poste de dépose, son origine (cellule de contrôle qualité, script, manuel), les heures de création,
de début, de chargement et de fin, son état et son résultat.

Origine des ordres (étape 8.1) :
- **automatique, depuis la cellule de contrôle qualité** : quand le bras UR5 pose un bac sur la table
  OK, l'ordre « table OK → Expédition » est créé ; sur la table REBUTS, « REBUTS → Assemblage »
  (reprise) ;
- **par script** (ex. « bobines : Magasin → Coupe ») ; plus tard aussi **manuellement** depuis une
  petite application.

**Priorités** (classe la plus basse d'abord, puis ordre d'arrivée ; le robot ne porte qu'un bac à
la fois) :
1. batterie faible (étape 8.4) ;
2. ordre qui **débloque la cellule de contrôle qualité** : sa table est pleine et le bac suivant
   attend en bout de convoyeur (le tapis est arrêté) ;
3. tous les autres ordres, y compris les ordres normaux de la cellule : le travail de la cellule
   ne doit pas monopoliser le robot.

## Machine à états d'un ordre
EN ATTENTE → ALLER AU POSTE DE PRISE → CHARGER → ALLER AU POSTE DE DÉPOSE → DÉCHARGER → TERMINÉ
(ou ÉCHEC). Chaque étape réutilise les briques déjà construites : navigation (A* + évitement +
prédiction), accostage (ArUco + lidar) et bras. Quand le robot prend un bac sur une table de la
cellule, il le signale à la cellule, qui peut de nouveau utiliser cette place. Si le robot reste
bloqué avec un bac à bord (poste de dépose non atteint, dépose ratée), le gestionnaire s'arrête en
défaut plutôt que de continuer avec un bac sur la plateforme.

## Validation (`tests\test_missions.py`)
Au départ, un seul ordre (script) : bobines Magasin → Coupe. Les autres ordres sont créés par la
cellule pendant que le robot travaille :

| Ordre | Origine | Classe | Attente | Exécution |
|---|---|---|---|---|
| #1 bobines : Magasin → Coupe | script | normale | 0 s | 149 s |
| #2 faisceau OK : table OK → Expédition | cellule | **déblocage** (table OK pleine, bac suivant en attente) | 111 s | 123 s |
| #3 faisceau à reprendre : REBUTS → Assemblage | cellule | normale | 214 s | 144 s |
| #4 faisceau OK : table OK → Expédition | cellule | — | créé à la fin, reste en attente | — |

Enchaînement observé : la cellule pose un bac sur la table OK (ordre #2), puis sur la table REBUTS
(ordre #3) ; le bac OK suivant ne trouve pas de place et attend en bout de convoyeur, ce qui place
l'ordre #2 dans la classe « déblocage ». Dès que le robot a pris le bac, la cellule pose le bac en
attente sur la table OK, ce qui crée l'ordre #4. Positions réelles vérifiées : bac de bobines sur
la table de la Coupe, premier bac OK à l'Expédition, bac défectueux à l'Assemblage, second bac OK
sur la table OK. **3 ordres terminés, 0 contact, 417 s simulées.**

## Flux de bacs et opérateurs (étape 8.2, `src\tote_flow.py`)
Pour qu'un poste de travail complet puisse tourner, l'atelier doit vivre tout seul autour du robot.
Trois rôles d'opérateurs sont simulés (les bacs sont déplacés directement, sans personnage qui marche) :
- **alimentation de la cellule CQ** : toutes les N secondes, un opérateur pose un bac de faisceau au
  début du convoyeur. Le résultat du contrôle suit une **séquence fixe répétée** (OK, OK, DÉFAUT…) :
  deux essais donnent les mêmes ordres, ce qui rend les mesures de la partie 11 comparables. La
  caméra décide toujours d'après la couleur de l'étiquette ; le flux ne fait que la peindre ;
- **retrait des bacs livrés** : un bac posé sur la table d'un poste consommateur (Coupe, Sertissage,
  Assemblage, Expédition) est pris par l'opérateur du poste après 75 s ; la table est de nouveau libre ;
- **réapprovisionnement du Magasin** : quand la table du Magasin est vide depuis 30 s, le magasinier
  y repose un bac de bobines.

Les bacs « retirés » vont dans une **réserve cachée** (sous le sol, hors du hall : statiques, sans
collision, invisibles pour les capteurs) où le flux et le magasinier les reprennent : la scène
contient un nombre fixe de bacs (3 sur le convoyeur au départ + 4 de réserve + le bac du Magasin).

**Choix de l'intervalle N** (remarque de l'utilisateur : les bacs doivent arriver assez lentement pour
que le robot ait le temps de montrer ses autres missions). Un ordre de la cellule occupe le robot
~127 s en moyenne (mesuré). Pour que le travail de la cellule prenne au plus ~1/3 du temps du robot,
N = **400 s** dans le programme de poste. Les ordres de la cellule restent dans la classe normale ;
ils ne passent devant que lorsqu'ils débloquent vraiment la cellule.

Règles ajoutées au gestionnaire de missions :
- un ordre ne démarre que si sa table de prise porte un bac (ex. un ordre de bobines attend le
  réapprovisionnement du Magasin) ;
- le robot ne pose jamais un bac sur une table encore occupée : il attend devant que l'opérateur
  ait retiré le bac précédent ;
- une prise ratée (robot toujours vide, bac resté sur la table) remet l'ordre dans la file
  (2 tentatives au plus) au lieu de l'abandonner : sinon la table de la cellule restait occupée et
  la cellule se bloquait.

Défaut trouvé pendant l'essai : le bras du youBot dressait **une seule fois** la liste des objets
« libres » qu'il peut saisir. Les bacs sortis de la réserve après cette première prise n'y étaient
pas : les doigts se fermaient bien sur la poignée, mais le bac n'était pas reconnu (« prise ratée »).
La liste est maintenant refaite à chaque prise.

**Validation** (`tests\test_tote_flow.py`, intervalle raccourci à 150 s pour que l'essai dure ~20
min) : convoyeur vide au départ, 3 bacs (OK, OK, DÉFAUT), deux ordres de bobines Magasin → Coupe dont
le second attend le réapprovisionnement.

| Ordre | Origine | Attente | Exécution |
|---|---|---|---|
| #1 bobines : Magasin → Coupe | script | 0 s | 150 s |
| #2 faisceau OK : table OK → Expédition | cellule | 91 s | 117 s |
| #3 faisceau OK : table OK → Expédition | cellule | 49 s | 159 s |
| #4 bobines : Magasin → Coupe (après réapprovisionnement) | script | 159 s | 109 s |
| #5 faisceau à reprendre : REBUTS → Assemblage | cellule | 177 s | 107 s |

Décisions de la caméra = la séquence (OK, OK, DÉFAUT). 4 bacs retirés par les opérateurs,
2 réapprovisionnements du Magasin, aucun bac tombé ni renversé. **5 ordres terminés, 0 contact,
642 s simulées.** Avec N = 150 s, la cellule occupe 60 % du temps du robot (382 s sur 642 s) ; avec
N = 400 s, la même durée moyenne par ordre donne ~32 %.

## Emplacements codés et vérification du code (étape 8.3)
Dans un atelier de câblage, on choisit un bac par l'**emplacement** où il est rangé (étiquette
« poste + emplacement » sur l'étagère ou la table), pas par une marque sur le bac lui-même. Chaque
place pour un bac porte donc son propre **code ArUco** sur la face de la table, côté allée :
- **Magasin** : 3 emplacements de prise, codes 11, 12, 13 (bobines rouges, noires, bleues) ;
- **Assemblage** : 2 emplacements de dépose, 41 et 42 ; **Expédition** : 46 et 47 ;
- les autres postes gardent un seul emplacement (le marqueur du poste).
Ces trois tables sont élargies à 1,20 m pour que le lidar voie encore la face de la table de part et
d'autre du robot aux emplacements extérieurs (ajustement de droite de l'accostage). Le dictionnaire
ArUco 4X4_50 ne contient que les identifiants 0 à 49 : les codes 61/62 essayés d'abord n'étaient pas
lisibles (le robot n'a rien lu et a refusé les deux places, sans poser le bac).

L'**inventaire du Magasin** (emplacement → contenu) est connu du gestionnaire : un ordre « bobines
rouges » devient « prendre à l'emplacement 11 ». Pour la dépose, le gestionnaire choisit la
**première place libre** du poste ; si toutes sont occupées, le robot attend devant le poste.

**Vérification** (point important pour le rapport) : arrivé devant l'emplacement, **avant d'accoster**,
le robot lit le code le plus centré dans l'image de sa caméra. Il doit être celui attendu :
- à la prise : sinon l'ordre est **REFUSÉ** (rien n'est pris) et signalé ;
- à la dépose : la place est refusée et signalée, une autre place libre est utilisée ; s'il n'y en a
  aucune, le robot s'arrête en défaut avec le bac à bord, sans le poser au hasard.
Ensuite le robot accoste sur le marqueur de **cet emplacement** (caméra puis lidar) : le problème
du champ de la caméra (60°, une seule place visible de près) ne se pose plus.

**Validation** (`tests\test_slots.py`) : la carte du gestionnaire est volontairement **fausse** pour
l'emplacement 13 (il le croit à la place du 12, comme après un réaménagement non reporté).

| Ordre | Emplacements | Résultat |
|---|---|---|
| #1 bobines rouges : Magasin → Expédition | 11 → 46 (première place libre) | livré en 80 s |
| #2 bobines bleues : Magasin → Coupe | attendu 13, **code lu 12** | **refusé** en 11 s, rien pris |
| #3 bobines noires : Magasin → Expédition | 12 → 47 (46 encore occupée) | livré en 70 s |
| #4 bobines rouges : Magasin → Coupe | 11 vide : attente du réapprovisionnement (322 s), puis 11 → Coupe | livré en 118 s |

Positions réelles : bac rouge sur la table de la Coupe, bac bleu toujours sur l'emplacement 13, bac
noir retiré de la place 47 par l'opérateur et reposé sur l'emplacement 12. **0 contact, 440 s
simulées.** (Le test a affiché « FAILED » à cause d'une attente erronée du test lui-même : il
supposait le bac noir encore sur la place 47 ; la vérification a été corrigée.)

## Batterie et recharge automatique (étape 8.4, `src\battery.py`)
Le modèle du youBot n'a pas de batterie : son état de charge (en %) est simulé à partir de l'activité
réelle du robot, mesurée dans la simulation toutes les 0,5 s :

consommation = 0,02 %/s × temps (électronique, capteurs, calculateur)
             + 0,15 %/m × distance parcourue (roues)
             + 0,03 %/rad × mouvement des articulations du bras (somme sur les 5 articulations)

Ces valeurs sont choisies pour qu'un poste de travail montre la recharge : environ 75 % consommés en
~20 min de travail (une dizaine d'ordres). La recharge est **accélérée** (0,6 %/s : ~2 min de 25 % à
95 %, au lieu d'environ une heure en vrai), pour ne pas allonger les essais.

Règles (classe de priorité 1) :
- sous **25 %**, le robot **termine l'ordre en cours** (il ne laisse pas un bac à bord ni une table à
  moitié servie), puis va à la borne de recharge **avant** tout autre ordre ;
- il accoste sur la borne (marqueur 0, caméra puis lidar, comme aux tables), se recharge jusqu'à
  **95 %**, recule et reprend la file ; les ordres créés pendant la recharge (cellule CQ) attendent ;
- une recharge peut aussi être demandée à la main (application de l'étape 8.6) ;
- batterie vide (0 %) : le robot s'arrête en défaut (ne s'est jamais produit : le seuil de 25 %
  laisse plusieurs ordres de marge).

**Validation** (`tests\test_battery.py`) : départ à 27 %, deux ordres de bobines.

| Étape | Temps | Batterie |
|---|---|---|
| #1 bobines rouges : Magasin → Coupe | 0 → 140 s | 27,0 → 20,9 % (6,1 % utilisés) |
| trajet vers la borne + accostage | 140 → 187 s | 20,9 → 18,6 % |
| recharge (accostée sur la borne) | 187 → 318 s | 18,6 → 95 % |
| #2 bobines noires : Magasin → Expédition | 318 → 400 s | 94,9 → 91,2 % (3,7 % utilisés) |

Répartition sur l'essai : temps 5,4 %, roues 4,5 % (30 m), bras 2,4 %. L'ordre #1 a bien été
terminé avant la recharge, et l'ordre #2 n'a commencé qu'après. 0 contact. (Le test a affiché
« FAILED » sur un détail de mesure : le niveau « après recharge » était relevé après le recul du
robot, 94,9 % au lieu de 95 % ; il est maintenant relevé à la fin de la charge.)

## Programme « poste de travail » (étape 8.5, `src\shift.py`)
Un seul programme fait tourner tout l'atelier pendant une durée donnée (30 min simulées par défaut) :
cellule de contrôle qualité, flux de bacs et opérateurs, ordres de bobines « scriptés » (un toutes les
240 s : bobines rouges → Coupe, noires → Coupe, bleues → Sertissage, en boucle), batterie et recharge,
gestionnaire de missions et robot. À la fin du poste, le robot termine l'ordre en cours, puis le
programme écrit un **bilan** (`results\shift_<date>_<heure>.txt` et `.json`) : ordres créés / terminés /
refusés / échoués / en attente, temps d'attente et d'exécution par origine, taux d'occupation du robot,
cellule CQ, opérateurs, batterie et recharges, navigation (distance, arrêts, re-planifications,
distance minimale à un opérateur, contacts). Ce sont les chiffres de la partie 11 (évaluation).
Une fenêtre « ordres » s'ajoute aux fenêtres en direct : ce que fait le robot, barre de batterie,
file d'attente avec la classe de priorité, derniers ordres, compteurs de la cellule et des opérateurs.

**Premier essai** (12 min simulées, batterie au départ 45 %, arrêté à 12,8 min pendant l'ordre #5) :

| Indicateur | Valeur |
|---|---|
| Ordres | 6 créés, 3 terminés (108 à 119 s), 1 échoué, 1 en cours, 1 en attente |
| Occupation du robot | 70,8 % (ordres + recharge) |
| Recharge | 1, de 25 à 95 % en 122 s |
| Batterie consommée | temps 12,9 %, roues 8,6 % (41 m), bras 4,8 % |
| Navigation | 11 arrêts, 4 re-planifications, distance minimale à un opérateur 0,78 m, **0 contact** |

L'ordre échoué (#4) a montré un défaut de la cellule : le bras UR5 pose les bacs **1 à 1,5 cm à côté**
de leur place, le long de l'allée. C'est la direction où la pince du youBot ne tolère qu'environ
± 2 cm. Le youBot a manqué ce bac deux fois, et sa première tentative l'a poussé de 4 cm. Mesures sur
6 bacs (cellule seule) : la barrière lumineuse donne la bonne position du bac (au mm près) ; attendre
que le bras réel ait rejoint sa cible avant de lâcher le bac, ou avant d'aspirer, ne change rien. Le
décalage vient du bac qui pend légèrement de travers sous la ventouse. Correction prévue : un
« contrôle de la prise » (mesure du décalage du bac sous la ventouse juste avant de le poser, comme
une caméra regardant la pince dans une vraie cellule), écrite mais **pas encore testée**.
En attendant, si un ordre de la cellule échoue définitivement, un **opérateur est appelé** : il retire
le bac à la main (signalé dans le bilan) et la cellule peut de nouveau utiliser la table ; sinon la
table restait occupée et la cellule se bloquait.

Durée : avec les fenêtres en direct, la simulation tourne environ 5 fois moins vite que le temps réel
(12,8 min simulées = 60 min réelles).

## Application d'ordres manuels (étape 8.6, `src\order_app.py`)
Une petite fenêtre (Tkinter, fourni avec Python : rien à installer) tourne dans le même programme que
le poste de travail et se met à jour entre deux pas de simulation. L'opérateur peut :
- **passer un ordre** : quoi prendre au Magasin (une couleur de bobines = son emplacement codé, ou
  « n'importe quel emplacement »), vers quel poste (Coupe, Sertissage, Assemblage, Expédition), sur
  quelle place (« auto » = première place libre, ou une place codée 41/42, 46/47), **urgent** ou non ;
- **annuler** un ordre encore en attente ;
- **mettre en pause / reprendre** les ordres scriptés (l'opérateur prend la main) ;
- **envoyer le robot en charge** (après l'ordre en cours) ;
- voir ce que fait le robot, sa batterie, la file d'attente (avec la classe de priorité) et les
  derniers ordres.
Les ordres manuels passent par le même gestionnaire que les autres : même choix d'emplacement, même
vérification du code avant l'accostage, mêmes priorités (un ordre urgent est en classe 2, devant les
ordres normaux, derrière la batterie). Un emplacement choisi à la main n'est utilisé que s'il porte un
bac (prise) ou s'il est libre (dépose). Les prises sont limitées au Magasin : la cellule de contrôle
qualité crée déjà ses propres ordres pour ses bacs.

**Essai** (`tests\test_order_app.py`, fenêtre visible, actions faites par le test à 10 s pendant que le
robot fait l'ordre scripté #1) : pause des ordres scriptés ; ordre urgent bobines bleues (emplacement
13) → Assemblage place 42 ; ordre bobines noires → Expédition, annulé dans la file ; ordre bobines
rouges → Coupe ; recharge demandée. Déroulement observé, dans l'ordre attendu :

| Étape | Résultat |
|---|---|
| #1 ordre scripté bobines rouges → Coupe | terminé en 109 s |
| ordres scriptés en pause | aucun nouvel ordre scripté ensuite |
| #3 bobines noires | annulé, retiré de la file |
| recharge demandée | faite juste après #1 (75 → 95 % en 38 s), avant tout autre ordre |
| #2 urgent bobines bleues 13 → 42 | passé devant les ordres normaux (classe 2), terminé en 156 s |
| #4 bobines rouges → Coupe | démarré ensuite (l'emplacement 11 avait été réapprovisionné entre-temps) |

L'essai a été arrêté pendant l'ordre #4 (demande de l'utilisateur) : son résultat final n'est pas mesuré.
Un ordre #5 a aussi été ajouté pendant l'essai depuis la fenêtre, par un clic.

La partie 8 est terminée : ordres de trois origines (cellule CQ, script, opérateur), emplacements codés
avec vérification, flux de bacs et opérateurs, batterie et recharge, programme de poste avec bilan, et
application de commande manuelle.
