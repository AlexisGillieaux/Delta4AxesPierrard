# Robot Delta 4 axes - Cinématique, Trajectoires, Repères, Commande et Visualisation

Ce projet regroupe le travail de l'équipe sur un robot **Delta à 4 axes** (3 bras qui positionnent le plateau en XYZ + 1 moteur qui le fait tourner autour de Z) :

- cinématique directe et inverse, trajectoires synchronisées à profil trapézoïdal ;
- repères **outil** et **utilisateur** (table de travail, outil excentré), avec calibration ;
- commande des moteurs (Modbus, free drive) et communication Raspberry Pi ↔ Teensy ;
- pince, visualisation 3D, tests d'usine (FAT) ;
- travaux de robotique du Quadrimestre 1 (robot Niryo).

## Structure du projet

```
.
├── Coords/                          # Cinématique et trajectoires
│   ├── DeltaCoord_fixed (1).py      # Cinématique inverse/directe 3 bras, visualisation, résolution
│   ├── trajectoires4Axes.py         # Trajectoires 4 axes avec repères (version actuelle)
│   ├── trajectoires.py              # Ancienne version 3 axes, sans repères
│   ├── trajectoires_backup.py       # Sauvegarde de l'ancienne version
│   ├── compare_trajectories.py      # Comparaison linéaire / trapézoïdal (version 3 axes)
│   └── Pyplot.py                    # Affichage 3D de vecteurs
│
├── M2/                              # Repères, transformations, fonctions robot
│   ├── delta_transform.py           # Repères outil / utilisateur, MGD/MGI avec repères, calibration
│   ├── test_delta_transform.py      # 17 tests + graphiques de delta_transform.py
│   ├── rigid_transform.py           # Classe RigidTransform (fournie par le professeur)
│   ├── 01_Fonctions_Fast.py         # Points, repères, free drive, pince
│   └── 02_Fonction_FAT.py           # Tests FAT (Factory Acceptance Tests)
│
├── Mouvements_Robot/                # Pilotage des moteurs en Modbus RTU (Python et C)
│   ├── Bouger_Moteur_OK.py
│   ├── Code_C_V1_NOK.c ... Code_C_V4_Dangereux.c, Code_C_V2_OK.exe
│   └── Formules_VSCode.txt          # (vide)
│
├── Communication/                   # Raspberry Pi <-> Teensy 4.1 (PlatformIO)
│   ├── src/main.cpp                 # Firmware Teensy : 4 UART, réception d'une matrice
│   ├── src/Teensy.cpp               # Ancien pont USB <-> UART (entièrement commenté)
│   ├── commnuciation_rasp.py        # Envoi d'une trajectoire en trame binaire (Raspberry)
│   ├── rasp_test.py                 # Essais de la liaison série (Raspberry)
│   └── platformio.ini
│
├── Visualisation 3D/                # Simulation du robot dans le navigateur
│   ├── Maxendre_NiceGui.py          # Interface NiceGUI avec scène 3D (version principale)
│   ├── Maxendre_Vpython.py          # Modèle VPython
│   ├── Maxendre_Flask.py            # Essai Flask (Hello World)
│   ├── Main.emojic                  # Essai Emojicode (Hello World)
│   └── Archives_NiceGUI.txt, Archives_Vpython.txt   # Anciennes versions
│
├── OuverturePince.py / .cpp         # Commande de la pince (Python côté Raspberry, C++ côté Teensy)
├── Trasnfostep.cpp / .exe           # Conversion angles -> pas moteur
├── ZZ robotiqueQ1/                  # Travaux du Quadrimestre 1 (robot Niryo)
├── Output/                          # Graphiques générés par les scripts et les tests
├── OLD/                             # Anciennes versions de DeltaCoord
├── .vscode/launch.json              # Configurations de lancement / débogage
├── requirements.txt                 # Dépendances Python
└── README.md
```

## Contributions (d'après l'historique git)

| Membre | Travaux principaux |
|--------|--------------------|
| AlexisGillieaux | Cinématique delta, 4ème axe, trajectoires et diagrammes temps, repères outil / utilisateur (`delta_transform.py`) et leurs tests |
| Maxendre | Visualisation 3D (NiceGUI, VPython), programmes de pilotage Modbus et free drive (`Mouvements_Robot/`), fonctions de points, repères et FAT (`M2/01`, `M2/02`), `Pyplot.py` |
| MpoyiHena | Communication Raspberry Pi ↔ Teensy (`Communication/`) |
| TIENSDONC | Commande de la pince (`OuverturePince`), conversion angles → pas (`Trasnfostep`) |

## Installation

```bash
pip install -r requirements.txt
```

Pour les scripts qui parlent au matériel (`Mouvements_Robot/`, `Communication/`, `OuverturePince.py`), `import serial` demande le paquet **`pyserial`**. Le paquet `serial` listé dans `requirements.txt` est un autre paquet, sans rapport. `Bouger_Moteur_OK.py` demande aussi `crcmod` :

```bash
pip install pyserial crcmod
```

## Conventions

- **Angles** : en degrés. **Positions** : en millimètres.
- **Repère de la base** : x, y horizontaux, z vertical, **négatif sous la base**. Angle moteur θ = 0 quand le bras est horizontal, positif vers le bas ; les moteurs des bras sont limités à ±90°.
- **Pose 4 axes** : `[x, y, z, rz]`, avec `rz` la rotation du plateau autour de Z. Une pose `[x, y, z]` est acceptée : `rz` vaut 0 au départ et reste inchangé à l'arrivée.
- **Angles moteurs** : `[θ1, θ2, θ3, θ4]`. Le 4ème axe suit `θ4 = RAPPORT_AXE4 · rz + OFFSET_AXE4`. Les valeurs par défaut (1.0 et 0.0) sont à adapter au montage réel.
- **Position inatteignable** : les fonctions de cinématique renvoient `NaN`, les fonctions de trajectoire lèvent une `ValueError`.
- **Un robot delta garde sa plateforme parallèle à la base** : la seule orientation possible est `rz`. Les repères outil et utilisateur ne peuvent donc pas être inclinés.

---

## Coords — Cinématique et trajectoires

### Cinématique des 3 bras (`DeltaCoord_fixed (1).py`)

```python
angles = DeltaInverse([[0, 0, -300]])          # array (1, 3) : [θ1, θ2, θ3] en degrés
position = DeltaForward([[78.5, -61.8, 57.4]])  # array (1, 3) : [x, y, z] en mm
```

Le nom du fichier (espace et parenthèses) interdit `import` : `delta_transform.py` le charge par son chemin avec `importlib`. Le fichier contient aussi `visualisation`, `resolution_xyz` et `visualisation_resolution` (résolution en XYZ selon le nombre de pas par tour), ainsi que ses propres tests (`test_aller_retour`, `test_forward_inverse`).

| Paramètre | Valeur | Signification |
|-----------|--------|---------------|
| `R_BASE` | 200 mm | Rayon du cercle des moteurs |
| `W_BASE` | 100 mm | Rayon de la base du triangle formé par les moteurs |
| `R_PLAT` | 40 mm | Rayon de la plateforme |
| `L_BRAS` | 200 mm | Longueur des bras moteurs |
| `L_BARRE` | 430 mm | Longueur des bras parallèles |

### Trajectoires 4 axes (`trajectoires4Axes.py`)

Toutes les trajectoires passent par les transformées de `delta_transform.py`. Les poses sont celles du **TCP** (l'origine du repère outil) exprimées dans le **repère utilisateur**. Sans repère donné (`None`), ce sont des matrices identité : la pose est celle de la bride dans la base.

| Fonction | Rôle |
|----------|------|
| `lineartrajectory(start, end, stepsmm, stepsdeg)` | Discrétise une droite en poses `[x, y, z, rz]` (pas d'angles) |
| `jointmotangles(start, end, deg_per_step, steps_per_second, ...)` | Trajectoire articulaire linéaire (vitesse constante), chemin droit dans l'espace des angles |
| `jointmotangles_trapezoidal(start, end, v_max_deg_s, a_max_deg_s2, dt, ...)` | Trajectoire articulaire synchronisée à profil trapézoïdal |
| `cartesian_trapezoidal_trajectory(start, end, v_max_mm_s, a_max_mm_s2, dt, ...)` | **Droite du TCP en XYZ** avec rotation `rz` synchronisée, profil trapézoïdal |
| `timediagram(trajectory, duration_sec)` / `resume_trajectoire(...)` | Angles et vitesses en fonction du temps, résumé par moteur |

```python
import delta_transform as tf
from trajectoires4Axes import cartesian_trapezoidal_trajectory, timediagram

crayon = tf.tool_frame_from_pose([30, 0, -80])           # TCP à 30 mm de l'axe, 80 mm sous la bride
table = tf.user_frame_from_pose([20, -30, -400, 15])     # table 400 mm sous la base, tournée de 15°

angles, duree = cartesian_trapezoidal_trajectory(
    [-40, -30, 20, 0], [60, 50, 60, 90],
    v_max_mm_s=100.0, a_max_mm_s2=300.0, dt=0.01,
    v_max_deg_s=30.0, a_max_deg_s2=60.0,                  # optionnel : limites des moteurs 1 à 3
    tool_frame=crayon, user_frame=table,
)
timediagram(angles, duree)
```

- Le TCP suit une droite à la vitesse demandée, même quand l'outil est excentré et tourne ; la bride, elle, décrit une courbe.
- Le mouvement le plus lent (TCP ou moteur 4) fixe la durée ; les limites du moteur 4 sont `V_MAX_AXE4_DEG_S` et `A_MAX_AXE4_DEG_S2`.
- `plus_court_chemin=True` fait tourner `rz` par le plus court chemin (180° au plus).
- Une pose hors de l'espace de travail, une vitesse nulle, un `dt` ou un pas qui donnerait plus de `MAX_ECHANTILLONS` (2 000 000) points, ou un repère du mauvais type lèvent une erreur explicite (`ValueError` / `TypeError`).
- Ne vérifie pas les limites ±90° des moteurs des bras (à contrôler sur le diagramme).

```bash
python Coords/trajectoires4Axes.py     # démonstrations + graphiques dans Output/
```

### Anciennes trajectoires (`trajectoires.py`, `compare_trajectories.py`)

Version 3 axes sans repères, conservée pour comparaison. `compare_trajectories.py` produit `Output/comparison_linear_vs_trapezoidal.png`.

```python
from trajectoires import jointmotangles, jointmotangles_trapezoidal, timediagram

trajectory, duration = jointmotangles(start=[0, 0, -300], end=[100, 150, -350],
                                      deg_per_step=0.5, steps_per_second=250)
trajectory, duration = jointmotangles_trapezoidal(start=[0, 0, -300], end=[100, 150, -350],
                                                  v_max_deg_s=30.0, a_max_deg_s2=60.0, dt=0.001)
```

---

## M2 — Repères et fonctions robot

### `delta_transform.py` — Repères outil et utilisateur

Chaîne de transformation (matrices homogènes 4×4, via `RigidTransform`) :

```
T_user_tcp   = T_base_user⁻¹ @ T_base_flange @ T_flange_tool         (angles -> pose du TCP)
T_base_flange = T_base_user   @ T_user_tcp    @ T_flange_tool⁻¹       (pose du TCP -> angles)
```

| Repère | Construction | Contenu |
|--------|--------------|---------|
| Outil `T_flange_tool` | `tool_frame_from_pose([x, y, z])` | Position du TCP dans le repère de la bride (outil excentré ou plus long) |
| Utilisateur `T_base_user` | `user_frame_from_pose([x, y, z, rz])` | Table ou zone de travail : translation + rotation autour de Z |

Un repère incliné est refusé (`ValueError`), y compris un `RigidTransform` construit à la main : la plateforme d'un delta reste horizontale.

**Calibration**

```python
# Repère utilisateur : 3 points à la même hauteur (pente ≤ 1°) : origine, point sur X, point côté Y positif
T_user = tf.calibrate_user_frame(points_3x3)

# Repère outil : poses de la bride [x, y, z, rz] (≥ 3, rz bien différents) avec le TCP posé sur un point fixe.
# La longueur de l'outil (z) n'est pas observable et se donne à part.
T_tool = tf.calibrate_tool_frame(poses_bride_nx4, z_outil=-80.0)
```

**Fonctions principales**

| Fonction | Rôle |
|----------|------|
| `delta_forward_with_frames(angles, tool, user)` | Angles `[θ1..θ4]` → pose du TCP (`RigidTransform`) |
| `delta_inverse_with_frames(pose, tool, user)` | Pose du TCP (`RigidTransform`) → angles ; `NaN` si inatteignable |
| `delta_inverse_from_poses4(poses, tool, user)` | Poses `[x, y, z, rz]` → angles (conserve les rotations de plus de 180°) |
| `delta_forward_to_poses4(angles, tool, user)` | Angles → poses `[x, y, z, rz]` |
| `DeltaInverse4`, `DeltaForward4` | Cinématique 4 axes sans repères (bride dans la base) |

`tool` et `user` valent `None` (identité) s'ils ne sont pas donnés.

```bash
python M2/test_delta_transform.py --sans-affichage     # 17 tests, image dans Output/
```

Les tests vérifient notamment que l'origine du repère outil décrit bien une droite (écart ~1e-13 mm) avec un outil excentré, y compris pendant une rotation.

### `rigid_transform.py` — Classe `RigidTransform`

Poses 3D rigides (rotation + translation), fournie par le professeur.

```python
from rigid_transform import RigidTransform

T = RigidTransform.from_homogeneous(mat)                  # depuis une matrice homogène
T = RigidTransform.from_pose([100, 0, -300, 0, 0, 90])    # depuis [x, y, z, rx, ry, rz] (mm, degrés)
```

### `01_Fonctions_Fast.py` — Points, repères, free drive, pince

Classes `Point` (création, affichage, suppression de points articulaires), `User_Frame`, `Tool_Frame` (TCP, limites, charge), `Free_Drive` et `Pince`.

```python
p = Point()
p.Add_Point(nom="P1", Theta1=0, Theta2=0, Theta3=0, Theta4=0)
```

Le nom du fichier commence par un chiffre : il ne peut pas être importé par `import`, il faut le renommer ou le charger avec `importlib`.

### `02_Fonction_FAT.py` — Tests d'usine (FAT)

- `FAT_Tests` : précision et répétabilité du robot, précision aux coins, vitesse du robot et de la pince, angle et précision de la pince.
- `FAT_Programmation` : vitesse, accélération, repère utilisateur (méthode 2 points), repère outil (méthode 3 points), contour, free drive.
- `FAT_Resultats` : enregistrement des résultats et rapport FAT.

---

## Mouvements_Robot — Pilotage des moteurs

Les variateurs sont pilotés en **Modbus RTU** (CRC16) sur un port série (`COM3`, 9600 bauds).

| Fichier | Rôle |
|---------|------|
| `Bouger_Moteur_OK.py` | Séquence de commandes Modbus (mode position, vitesse, déplacement, arrêt). Demande `pyserial` et `crcmod`. |
| `Code_C_V1_NOK.c` | Mode vitesse avec activation forcée de l'entrée ENABLE (DI5/S-ON) ; port passé en argument (`COMx`). Version non retenue. |
| `Code_C_V2.c`, `Code_C_V2_OK.exe` | Équivalent en C de la séquence Python ; l'exécutable est la version compilée. |
| `Code_C_V3_Apprendre.c` | Mode couple avec compensation de gravité, lecture de l'angle mécanique (apprentissage). |
| `Code_C_V4_Dangereux.c` | **Free drive** : mode couple, compensation de gravité (sinus), Ctrl+C pour quitter. À utiliser avec précaution. |

---

## Communication — Raspberry Pi ↔ Teensy 4.1

Liaison UART à **115200 bauds**. Le projet PlatformIO (`platformio.ini`, carte `teensy41`) est dans `Communication/`.

- `src/main.cpp` : initialise `Serial` (USB), `Serial1` (Raspberry Pi, broches 0/1) et `Serial2` à `Serial4` (3 variateurs, broches 7/8, 15/14 et 16/17, envoi en 9 bits par `driver()`). `menu()` attend le message `START_MATRIX` puis `ecrire_matrice()` reçoit une matrice. **Code en cours de développement.**
- `commnuciation_rasp.py` (Raspberry) : envoie une trajectoire (jusqu'à 255 points `[X, Y, Z, T]`, 4 `float` little-endian) en trame binaire : `0xAA 0xBB`, nombre de points, données, checksum (somme & 0xFF), `0xCC 0xDD`.
- `rasp_test.py` : essais de la liaison série côté Raspberry.
- `src/Teensy.cpp` : ancien pont USB ↔ UART, entièrement commenté.

Les deux côtés n'utilisent pas encore le même protocole (texte `START_MATRIX` côté Teensy, trame binaire côté Raspberry) : à unifier.

Vérifier le brochage de la carte et la compatibilité des niveaux logiques avant le branchement (TX ↔ RX croisés, GND commun).

---

## Pince et conversion en pas

- `OuverturePince.py` (Raspberry) : `OuverturePince(pourcentage, temps_ms)` et `FermeturePince(...)` envoient `"pourcentage,temps_ms\n"` au Teensy sur `/dev/ttyACM0` (115200 bauds, pourcentage borné à 0-100).
- `OuverturePince.cpp` (Teensy) : moteur pas à pas (AccelStepper) et encodeur : broches STEP 2, DIR 3, EN 4, encodeur 5/6 ; course de 0 (fermée) à 4000 pas (ouverte).
- `Trasnfostep.cpp` / `.exe` : `Step` et `Stepper` convertissent des angles en pas moteur (1000 pas par tour).

---

## Visualisation 3D

```bash
python "Visualisation 3D/Maxendre_NiceGui.py"     # http://localhost:8080
```

- `Maxendre_NiceGui.py` : interface web NiceGUI avec scène 3D du robot ; l'état `x, y, z, rz` est partagé entre tous les utilisateurs connectés, avec limites de déplacement et remise à zéro.
- `Maxendre_Vpython.py` : modèle VPython « Robot Delta - Modèle réel » (axes, bras, plateau).
- `Maxendre_Flask.py`, `Main.emojic` : essais « Hello World » de technologies web.
- `Archives_*.txt` : anciennes versions.

---

## ZZ robotiqueQ1 — Quadrimestre 1

Travaux sur le robot **Niryo** : cinématique directe (`niryo-dir.py`) et inverse (`niryo-inv.py`), cinématique inverse générale (`inverse.py`), exercice 2 (`ex2.py`, transformations de Denavit-Hartenberg avec sympy) et le notebook `traj.ipynb` (trajectoires), avec `robot_animation.gif`.

---

## Output — Graphiques

| Fichier | Produit par |
|---------|-------------|
| `test_delta_transform_lignes.png` | `M2/test_delta_transform.py` : ligne droite du TCP avec outil excentré, pivot, écart, angles |
| `trajectoire4axes_lineaire.png`, `trajectoire4axes_trapezoidal.png`, `trajectoire4axes_reperes.png` | `Coords/trajectoires4Axes.py` (`test()`) |
| `comparison_linear_vs_trapezoidal.png`, `trajectoire_linéaire.png`, `trajectoire_trapezoidal.png`, `test_timediagram.png` | Anciennes trajectoires 3 axes |
| `Resolution1.png` | Résolution en XYZ (`DeltaCoord_fixed (1).py`) |

---

## Notes techniques

- **Arrays** : numpy arrays de shape `(n, 3)` pour la cinématique 3 bras, `(n, 4)` pour les poses et angles 4 axes.
- **Synchronisation des moteurs** : `t_total = max(t_i)` sur tous les axes.
- **Un exemple de mauvaise utilisation** : un repère outil incliné (ex. crayon à 45°) ne change ni la position du TCP ni les angles moteurs, et le robot ne saurait pas l'orienter. Seule l'excentration `[x, y, z]` est modélisée.
- **`.vscode/launch.json`** contient une configuration « Essai moteur sur COM3 » qui vise `Mouvements Robot\modbus.exe` (avec un espace) et une tâche de compilation « Compiler le programme moteur (C) » : le dossier s'appelle `Mouvements_Robot`, `modbus.exe` n'existe pas et aucun `tasks.json` ne définit cette tâche. À mettre à jour avant d'utiliser cette configuration.
