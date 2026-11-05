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

## Étapes suivantes
8.3 emplacements codés (marqueur ArUco par emplacement) et vérification du code avant la
prise ou la dépose ; 8.4 batterie et recharge automatique ; 8.5 programme « poste de travail »
(`shift.py`) avec bilan ; 8.6 petite application pour passer des ordres à la main.
