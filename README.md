# Robot Delta - Cinématique et Visualisation

Ce projet contient l'implémentation des algorithmes de cinématique directe et inverse pour un robot Delta, ainsi qu'une fonction de visualisation 3D.

## Fichiers

- `DeltaCoord.py` : Implémentation originale avec numpy (version optimisée)
- `DeltaCoord2.py` : Implémentation adaptée avec numpy et visualisation
- `requirements.txt` : Dépendances Python
- `test_visualisation.py` : Exemples d'utilisation de la visualisation

## Fonctions disponibles (DeltaCoord2.py)

### Cinématique Inverse
```python
delta_calcInverse(f, e, rf, re, coordinates)
```
- **Paramètres**:
  - `f` : rayon de la base (100.0 mm)
  - `e` : rayon de la plateforme (40.0 mm)
  - `rf` : longueur des bras moteurs (200.0 mm)
  - `re` : longueur des connecteurs (430.0 mm)
  - `coordinates` : array numpy de shape (n, 3) avec les coordonnées [x, y, z]
- **Retour** : array numpy de shape (n, 3) avec les angles [theta1, theta2, theta3] en degrés

### Cinématique Directe
```python
delta_calcForward(f, e, rf, re, angles)
```
- **Paramètres**:
  - `f, e, rf, re` : mêmes paramètres que ci-dessus
  - `angles` : array numpy de shape (n, 3) avec les angles [theta1, theta2, theta3] en degrés
- **Retour** : array numpy de shape (n, 3) avec les coordonnées [x, y, z]

### Visualisation 3D
```python
visualisation(position=None, angles=None, ax=None, show_plot=True)
```
- **Paramètres**:
  - `position` : coordonnées [x, y, z] de l'effecteur (calcule les angles automatiquement)
  - `angles` : angles [theta1, theta2, theta3] des moteurs (calcule la position automatiquement)
  - `ax` : axes matplotlib pour intégration dans des subplots
  - `show_plot` : afficher la figure (défaut: True)

## Exemples d'utilisation

### Calcul d'angles à partir d'une position
```python
import numpy as np
from DeltaCoord2 import delta_calcInverse

# Paramètres du robot
f, e, rf, re = 100.0, 40.0, 200.0, 430.0

# Position cible
position = np.array([[250, 250, -200]])
angles = delta_calcInverse(f, e, rf, re, position)
print(f"Angles: {angles[0]}")  # [65.8°, -55.1°, 43.6°]
```

### Calcul de position à partir d'angles
```python
from DeltaCoord2 import delta_calcForward

angles = np.array([[78.5, -61.8, 57.4]])
position = delta_calcForward(f, e, rf, re, angles)
print(f"Position: {position[0]}")  # [261.6, 281.1, -150.9]
```

### Visualisation
```python
from DeltaCoord2 import visualisation

# Visualiser avec des angles
visualisation(angles=[78.5, -61.8, 57.4])

# Visualiser avec une position
visualisation(position=[250, 250, -200])

# Visualisation par défaut
visualisation()
```

## Installation

```bash
pip install -r requirements.txt
```

## Paramètres du robot

- `e = 40.0` : Rayon de la plateforme (distance centre → point d'attache)
- `f = 100.0` : Rayon de la base (distance centre → moteur)
- `re = 430.0` : Longueur des bras parallèles (connecteurs)
- `rf = 200.0` : Longueur des bras moteurs (biceps)

## Tests

Exécutez `python DeltaCoord2.py` pour voir les tests automatiques incluant:
- Tests de cinématique inverse et directe
- Vérification aller-retour
- Test de visualisation
