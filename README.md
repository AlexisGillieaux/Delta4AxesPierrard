# Robot Delta - Cinématique, Trajectoires et Visualisation

Ce projet implémente les algorithmes de cinématique directe/inverse pour un robot Delta, génère des trajectoires articulaires synchronisées avec profils trapézoïdaux, et regroupe les travaux de robotique du Master 1.

## Structure du projet

```
.
├── M2/                            # Module Master 2 (travaux actifs)
│   ├── delta_transform.py         # Calibration repères outil et utilisateur
│   ├── rigid_transform.py         # Classe RigidTransform (poses 3D rigides)
│   ├── 01_Fonctions_Fast.py       # Gestion des points (classe Point)
│   └── 02_Fonction_FAT.py         # Tests FAT (Factory Acceptance Tests)
│
├── Coords/                        # Cinématique et trajectoires
│   ├── DeltaCoord_fixed (1).py    # Cinématique inverse/directe (version corrigée)
│   ├── trajectoires.py            # Génération de trajectoires (linéaire + trapézoïdal)
│   ├── compare_trajectories.py    # Comparaison des profils de trajectoire
│   └── Pyplot.py                  # Visualisation 3D
│
├── robotiqueQ1/                   # Travaux du Quadrimestre 1
│   ├── niryo-dir.py               # Cinématique directe - Robot Niryo
│   ├── niryo-inv.py               # Cinématique inverse - Robot Niryo
│   ├── inverse.py                 # Cinématique inverse générale
│   ├── ex2.py                     # Exercice 2
│   └── traj.ipynb                 # Notebook - trajectoires
│
├── Output/                        # Graphiques générés
│   ├── comparison_linear_vs_trapezoidal.png
│   ├── trajectoire_linéaire.png
│   ├── trajectoire_trapezoidal.png
│   ├── Resolution1.png
│   └── test_timediagram.png
│
├── OLD/                           # Archives (anciennes versions)
│   ├── DeltaCoord.py
│   ├── DeltaCoord2.py
│   └── DeltaCoord_fixed.py
│
├── requirements.txt               # Dépendances Python
└── README.md
```

## Installation

```bash
pip install -r requirements.txt
```

---

## M2 — Master 2

### rigid_transform.py — Classe `RigidTransform`

Utilitaire pour manipuler des poses 3D rigides (rotation + translation).

```python
from rigid_transform import RigidTransform

# Depuis une matrice homogène
T = RigidTransform.from_homogeneous(mat)

# Depuis une pose [x, y, z, rx, ry, rz] (mm et degrés)
T = RigidTransform.from_pose([100, 0, -300, 0, 0, 90])
```

### delta_transform.py — Calibration des repères

```python
from delta_transform import tool_frame_from_pose, user_frame_from_pose

# Repère outil à partir d'une géométrie connue
T_tool = tool_frame_from_pose([x, y, z, rx, ry, rz])

# Repère utilisateur à partir d'une position mesurée
T_user = user_frame_from_pose([x, y, z, rx, ry, rz])
```

### 01_Fonctions_Fast.py — Gestion des points

Classe `Point` pour créer et stocker des points articulaires.

```python
from Fonctions_Fast import Point

p = Point()
p.Add_Point(nom="P1", Theta1=0, Theta2=0, Theta3=0, Theta4=0)
```

### 02_Fonction_FAT.py — Tests FAT

Classe `FAT_Tests` pour valider la précision du robot sur 10 points prédéfinis.

```python
from Fonction_FAT import FAT_Tests

fat = FAT_Tests()
fat.FAT_Précision_Robot(mode_déplacement="Linéaire")
# Écart acceptable : max 2.5 mm
```

---

## Coords — Cinématique et Trajectoires

### Cinématique Delta (DeltaCoord_fixed.py)

#### Cinématique Inverse
```python
from DeltaCoord_fixed import DeltaInverse

angles = DeltaInverse([[0, 0, -300]])
# Retourne: array (1, 3) avec [theta1, theta2, theta3] en degrés
```

#### Cinématique Directe
```python
from DeltaCoord_fixed import DeltaForward

position = DeltaForward([[78.5, -61.8, 57.4]])
# Retourne: array (1, 3) avec [x, y, z] en mm
```

#### Paramètres du robot Delta

| Paramètre | Valeur | Signification |
|-----------|--------|---------------|
| `rB` | 200 mm | Rayon de la base (moteurs) |
| `wB` | 100 mm | Distance au centre du triangle de base |
| `rP` | 40 mm | Rayon de la plateforme |
| `L` | 200 mm | Longueur des bras moteurs |
| `l` | 430 mm | Longueur des bras parallèles |

### Trajectoires (trajectoires.py)

#### Trajectoire linéaire articulaire
```python
from trajectoires import jointmotangles

trajectory, duration = jointmotangles(
    start=[0, 0, -300],
    end=[100, 150, -350],
    speedmot=0.5,           # degrés par pas
    steps_per_second=250
)
```

#### Trajectoire trapézoïdale (recommandé)
```python
from trajectoires import jointmotangles_trapezoidal

trajectory, duration = jointmotangles_trapezoidal(
    start=[0, 0, -300],
    end=[100, 150, -350],
    v_max_deg_s=30.0,       # vitesse max (degrés/s)
    a_max_deg_s2=60.0,      # accélération max (degrés/s²)
    dt=0.001                # pas de temps (s)
)
```

Tous les moteurs sont synchronisés (terminent au même moment). Profil: accélération → vitesse constante → décélération.

#### Visualisation
```python
from trajectoires import timediagram

fig = timediagram(trajectory, duration)
# Angles vs temps + vitesses angulaires vs temps
```

---

## robotiqueQ1 — Quadrimestre 1

Travaux sur le robot **Niryo** (cinématique directe et inverse) et génération de trajectoires.

```bash
python robotiqueQ1/niryo-dir.py   # Cinématique directe
python robotiqueQ1/niryo-inv.py   # Cinématique inverse
```

Le notebook `traj.ipynb` contient les trajectoires commentées.

---

## Notes techniques

- **Angles** : en degrés (pas radians)
- **Positions XYZ** : en millimètres
- **Arrays** : numpy arrays de shape `(n, 3)`
- Synchronisation moteurs : `t_total = max(t_i)` sur tous les axes
