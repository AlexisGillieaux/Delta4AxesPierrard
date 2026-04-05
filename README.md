# Robot Delta - Cinématique, Trajectoires et Visualisation

Ce projet implémente les algorithmes de cinématique directe/inverse pour un robot Delta et générant des trajectoires articulaires synchronisées avec profils trapézoïdaux.

## Structure du projet

```
.
├── DeltaCoord.py              # Cinématique (implémentation optimisée)
├── DeltaCoord2.py             # Cinématique alternative avec visualisation
├── trajectoires.py            # Génération et visualisation de trajectoires
├── requirements.txt           # Dépendances Python
└── README.md                  # Documentation
```

## Installation

```bash
pip install -r requirements.txt
```

---

## 1. Cinématique (DeltaCoord.py)

### Fonctions disponibles

#### Cinématique Inverse
```python
from DeltaCoord import DeltaInverse

angles = DeltaInverse([[0, 0, -300]])
# Retourne: array de shape (1, 3) avec [theta1, theta2, theta3]
```

#### Cinématique Directe
```python
from DeltaCoord import DeltaForward

position = DeltaForward([[78.5, -61.8, 57.4]])
# Retourne: array de shape (1, 3) avec [x, y, z]
```

### Paramètres du robot Delta

| Paramètre | Valeur | Signification |
|-----------|--------|---------------|
| `rB` | 200 mm | Rayon de la base (moteurs) |
| `wB` | 100 mm | Distance au centre du triangle de base |
| `rP` | 40 mm | Rayon de la plateforme (points d'attache) |
| `L` | 200 mm | Longueur des bras moteurs |
| `l` | 430 mm | Longueur des bras parallèles (connecteurs) |

---

## 2. Génération de Trajectoires (trajectoires.py)

### 2.1 Trajectoires linéaires

```python
from trajectoires import lineartrajectory

# Chemin cartésien linéaire
path = lineartrajectory(
    start=[0, 0, -300],
    end=[100, 150, -350],
    stepsmm=2  # pas de 2mm entre chaque point
)
```

### 2.2 Trajectoires articulaires - Linéaire

```python
from trajectoires import jointmotangles

trajectory, duration = jointmotangles(
    start=[0, 0, -300],
    end=[100, 150, -350],
    speedmot=0.5,           # degrés par pas
    steps_per_second=250    # cadence
)
# Retourne:
#   - trajectory: array (n_steps, 3) avec angles en degrés
#   - duration: durée en secondes
```

**Caractéristiques:**
- Profil de vitesse **rectangulaire** (vitesse instantanée)
- Rapide, simple
- Utile pour spécifications de bas niveau

### 2.3 Trajectoires articulaires - Trapézoïdale (recommandé)

```python
from trajectoires import jointmotangles_trapezoidal

trajectory, duration = jointmotangles_trapezoidal(
    start=[0, 0, -300],
    end=[100, 150, -350],
    v_max_deg_s=30.0,       # vitesse max (degrés/s)
    a_max_deg_s2=60.0,      # accélération max (degrés/s²)
    dt=0.001                # pas de temps (s)
)
# Retourne:
#   - trajectory: array (n_steps, 3) avec angles en degrés
#   - duration: durée en secondes
```

**Caractéristiques:**
- Profil de vitesse **trapézoïdal** (accélération → vitesse constante → décélération)
- Tous les moteurs synchronisés (terminent au même moment)
- Courbes lisses et réalistes (sigmoïdes)
- Idéal pour les mouvements robustiques

### 2.4 Visualisation des trajectoires

```python
from trajectoires import timediagram

fig = timediagram(trajectory, duration)
# Affiche 2 graphiques:
#   1. Angles vs temps (3 moteurs)
#   2. Vitesses angulaires vs temps (dérivées numériques)
```

---

## Exemples complets

### Exemple 1: Cinématique simple

```python
import numpy as np
from DeltaCoord import DeltaInverse, DeltaForward

# Position de départ
pos_start = [0, 0, -300]
angles_start = DeltaInverse(pos_start)
print(f"Angles de départ: {angles_start}")

# Position finale
pos_end = [100, 150, -350]
angles_end = DeltaInverse(pos_end)
print(f"Angles finaux: {angles_end}")

# Vérification (cinématique directe)
pos_check = DeltaForward(angles_end)
print(f"Position calculée: {pos_check}")
```

### Exemple 2: Trajectoire avec profil trapézoïdal

```python
from trajectoires import jointmotangles_trapezoidal, timediagram
import matplotlib.pyplot as plt

# Générer la trajectoire
trajectory, duration = jointmotangles_trapezoidal(
    start=[0, 0, -300],
    end=[100, 150, -350],
    v_max_deg_s=30.0,
    a_max_deg_s2=60.0,
    dt=0.001
)

print(f"Durée du mouvement: {duration:.3f} s")
print(f"Nombre de points: {len(trajectory)}")

# Visualiser
fig = timediagram(trajectory, duration)
plt.show()  # ou fig.savefig('trajectory.png')
```

### Exemple 3: Comparaison linéaire vs trapézoïdal

```python
from trajectoires import jointmotangles, jointmotangles_trapezoidal
import matplotlib.pyplot as plt

# Profil linéaire
traj_lin, dur_lin = jointmotangles(
    [0, 0, -300], [100, 150, -350],
    speedmot=0.5, steps_per_second=250
)

# Profil trapézoïdal
traj_trap, dur_trap = jointmotangles_trapezoidal(
    [0, 0, -300], [100, 150, -350],
    v_max_deg_s=30.0, a_max_deg_s2=60.0
)

print(f"Linéaire      : {len(traj_lin)} points, {dur_lin:.3f} s")
print(f"Trapézoïdal   : {len(traj_trap)} points, {dur_trap:.3f} s")
```

---

## Tests

Exécutez le module directement pour tester les fonctions:

```bash
# Tests de cinématique
python DeltaCoord.py

# Tests de trajectoires
python trajectoires.py
```

---

## Notes techniques

### Synchronisation des moteurs
Tous les moteurs d'une trajectoire trapézoïdale sont synchronisés:
- Calcul du temps minimal pour chaque moteur
- Résolution équation: `d = v_peak*T - v_peak²/a`
- Tous les moteurs utilisent le temps **t_total = max(t_i)**

### Profils de vitesse
- **Linéaire**: Changements instantanés → actuateurs réels impossible
- **Trapézoïdal**: Profils lisses → réalistes et robustes

### Format des données
- **Angles**: en **degrés** (pas radians)
- **Position XYZ**: en **millimètres**
- **Arrays**: numpy arrays de shape (n, 3)
- Vérification aller-retour
- Test de visualisation
