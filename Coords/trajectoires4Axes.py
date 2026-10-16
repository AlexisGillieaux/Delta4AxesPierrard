"""
Trajectoires et diagrammes temps du robot delta 4 axes.

Les moteurs 1, 2 et 3 actionnent les bras et positionnent le plateau en XYZ.
Le moteur 4 fait tourner le plateau autour de Z (rotation parallèle au sol).

Conventions
-----------
- Pose cartésienne : [x, y, z, rz]  (mm, mm, mm, degrés)
- Angles moteurs   : [θ1, θ2, θ3, θ4]  (degrés)
- Les poses de départ et d'arrivée des trajectoires peuvent avoir 3 composantes
  [x, y, z] : au départ rz vaut alors 0, à l'arrivée rz reste celui du départ
  (le plateau ne tourne pas).

Modèle du 4ème axe
------------------
Le plateau d'un robot delta reste toujours parallèle à la base. La rotation rz
du plateau est donc indépendante de la position XYZ, et l'angle du moteur 4 vaut :

    θ4 = RAPPORT_AXE4 * rz + OFFSET_AXE4

Ce modèle est valable que le moteur 4 soit fixé sur la base (arbre télescopique
à deux cardans) ou directement sur le plateau.
"""

import importlib.util
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

# Dossier racine du projet : le dossier « Output » des images y est créé.
RACINE_PROJET = Path(__file__).resolve().parent.parent

# Cinématique du robot (version à jour) : « DeltaCoord_fixed (1).py », à côté de
# ce script. Son nom (espace et parenthèses) interdit « import » : on charge le
# fichier par son chemin, ce qui marche quel que soit le dossier de lancement.
FICHIER_DELTACOORD = Path(__file__).resolve().parent / "DeltaCoord_fixed (1).py"


def _charger_module(nom, chemin):
    """Charge un fichier Python par son chemin et retourne le module."""
    spec = importlib.util.spec_from_file_location(nom, chemin)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


d1 = _charger_module("DeltaCoord_fixed_v1", FICHIER_DELTACOORD)


# ============================================================================
#                      PARAMÈTRES DU 4ÈME AXE (À ADAPTER)
# ============================================================================

# Tours du moteur 4 pour un tour du plateau (réducteur, poulies...).
# 1.0 = entraînement direct.
RAPPORT_AXE4 = 1.0

# Angle du moteur 4 (degrés) quand le plateau est à rz = 0.
OFFSET_AXE4 = 0.0

# Limites du moteur 4, en degrés MOTEUR. Valeurs d'exemple : à remplacer par
# celles du moteur réellement monté.
V_MAX_AXE4_DEG_S = 90.0
A_MAX_AXE4_DEG_S2 = 180.0


# ============================================================================
#                      STYLE DES DIAGRAMMES TEMPS
# ============================================================================

# Une couleur fixe par moteur, dans l'ordre de la palette catégorielle.
COULEURS_MOTEURS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
ENCRE = "#0b0b0b"
ENCRE_SECONDAIRE = "#52514e"
GRILLE = "#e1e0d9"
AXES = "#c3c2b7"
SURFACE = "#fcfcfb"
EPAISSEUR_TRAIT = 1.5


# ============================================================================
#                      CINÉMATIQUE 4 AXES
# ============================================================================

def axe4_inverse(rz_deg):
    """Convertit la rotation du plateau rz (degrés) en angle du moteur 4 (degrés)."""
    return RAPPORT_AXE4 * np.asarray(rz_deg, dtype=float) + OFFSET_AXE4


def axe4_direct(theta4_deg):
    """Convertit l'angle du moteur 4 (degrés) en rotation du plateau rz (degrés)."""
    return (np.asarray(theta4_deg, dtype=float) - OFFSET_AXE4) / RAPPORT_AXE4


def _pose4(pose, rz_defaut=0.0):
    """Retourne une pose [x, y, z, rz] en float. Une pose [x, y, z] reçoit rz = rz_defaut."""
    p = np.asarray(pose, dtype=float).reshape(-1)
    if p.size == 3:
        p = np.append(p, rz_defaut)
    if p.size != 4:
        raise ValueError(f"Pose attendue [x, y, z, rz] ou [x, y, z], reçu : {pose}")
    return p


def DeltaInverse4(poses):
    """Cinématique inverse du robot delta 4 axes.

    Args:
        poses (array-like): pose [x, y, z, rz] ou array de shape (n, 4)

    Returns:
        numpy.ndarray: array de shape (n, 4) avec [θ1, θ2, θ3, θ4] en degrés.
            Les angles des bras valent NaN si la position est hors de portée.
    """
    p = np.atleast_2d(np.asarray(poses, dtype=float))
    if p.ndim != 2 or p.shape[1] != 4:
        raise ValueError(f"Poses attendues de shape (n, 4), reçu {p.shape}")

    angles_bras = np.asarray(d1.DeltaInverse(p[:, :3])).reshape(-1, 3)
    theta4 = axe4_inverse(p[:, 3])
    return np.column_stack((angles_bras, theta4))


def DeltaForward4(angles):
    """Cinématique directe du robot delta 4 axes.

    Args:
        angles (array-like): angles [θ1, θ2, θ3, θ4] ou array de shape (n, 4), en degrés

    Returns:
        numpy.ndarray: array de shape (n, 4) avec [x, y, z, rz] (mm et degrés)
    """
    q = np.atleast_2d(np.asarray(angles, dtype=float))
    if q.ndim != 2 or q.shape[1] != 4:
        raise ValueError(f"Angles attendus de shape (n, 4), reçu {q.shape}")

    xyz = np.asarray(d1.DeltaForward(q[:, :3])).reshape(-1, 3)
    rz = axe4_direct(q[:, 3])
    return np.column_stack((xyz, rz))


def _poses_depart_arrivee(start, end, plus_court_chemin=False):
    """Prépare les poses de départ et d'arrivée.

    Une pose [x, y, z] sans rz reçoit rz = 0 au départ ; à l'arrivée elle garde
    le rz du départ, le plateau ne tourne donc pas.

    Si plus_court_chemin est vrai, la rotation rz de l'arrivée est ramenée à
    180° au plus de celle du départ (ex. 170° -> -170° fait +20° et non -340°).
    À exactement 180° d'écart, le sens demandé est conservé (0° -> 180° tourne
    de +180°, 0° -> -180° de -180°).

    La rz d'arrivée est alors REMPLACÉE par une valeur équivalente modulo 360°
    (ex. 540° devient 180° si le départ est à 0°) : l'orientation finale du
    plateau est la même, mais l'angle θ4 final renvoyé diffère de celui qu'on
    obtiendrait sans cette option. À n'activer que si l'outil peut tourner sans
    limite (pas de câble qui s'enroule).
    """
    p0 = _pose4(start)
    p1 = _pose4(end, rz_defaut=p0[3])
    if plus_court_chemin:
        ecart_brut = p1[3] - p0[3]
        delta_rz = (ecart_brut + 180.0) % 360.0 - 180.0
        # Le modulo donne -180° pour un demi-tour : on rend le sens demandé.
        if delta_rz <= -180.0 + 1e-9 and ecart_brut > 0.0:
            delta_rz = 180.0
        p1[3] = p0[3] + delta_rz
    return p0, p1


def _verifier_atteignable(angles, poses):
    """Lève une ValueError si une pose de la trajectoire est hors de l'espace de travail."""
    invalides = np.isnan(angles).any(axis=1)
    if invalides.any():
        i = int(np.flatnonzero(invalides)[0])
        raise ValueError(
            f"Pose hors de l'espace de travail : {np.round(poses[i], 2)} "
            f"(point {i} de la trajectoire)"
        )


def _verifier_chemin_articulaire(angles, tolerance_deg=1e-3):
    """Lève une ValueError si une configuration des bras d'un chemin articulaire est impossible.

    Une trajectoire interpolée dans l'espace articulaire n'est sûre qu'aux
    extrémités : rien ne garantit que les points intermédiaires correspondent
    à une pose réelle du robot. Chaque configuration [θ1, θ2, θ3] est donc
    passée en cinématique directe puis inverse. Elle est refusée si le retour
    par la cinématique inverse ne redonne pas les mêmes angles (à tolerance_deg
    près) : pas de solution directe (les barres ne peuvent pas se fermer sur le
    plateau), autre mode d'assemblage ou configuration singulière.
    Seuls les 3 moteurs des bras sont concernés : θ4 est indépendant de XYZ.
    """
    q = angles[:, :3]
    xyz = np.asarray(d1.DeltaForward(q)).reshape(-1, 3)
    retour = np.asarray(d1.DeltaInverse(xyz)).reshape(-1, 3)
    valides = (np.abs(retour - q) <= tolerance_deg).all(axis=1)    # un NaN compte comme invalide
    invalides = ~valides
    if invalides.any():
        i = int(np.flatnonzero(invalides)[0])
        raise ValueError(
            f"Configuration des bras impossible sur le chemin articulaire : "
            f"angles {np.round(angles[i], 2)} (point {i} de la trajectoire)"
        )


def _verifier_limites(**limites):
    """Lève une ValueError si une vitesse, une accélération ou un pas de temps n'est pas > 0."""
    for nom, valeur in limites.items():
        if not valeur > 0:
            raise ValueError(f"{nom} doit être strictement positif (reçu {valeur})")


# ============================================================================
#                      PROFILS TRAPÉZOÏDAUX
# ============================================================================

def _temps_min_trapeze(d, v_max, a_max):
    """Temps minimal pour parcourir la distance d avec v_max et a_max.

    Fonctionne avec des scalaires ou des arrays (un élément par moteur).
    """
    d = np.asarray(d, dtype=float)
    v_max = np.asarray(v_max, dtype=float)
    a_max = np.asarray(a_max, dtype=float)

    t_triangle = 2.0 * np.sqrt(d / a_max)           # n'atteint pas v_max
    t_trapeze = d / v_max + v_max / a_max            # accél + croisière + décél
    t = np.where(v_max**2 / a_max > d, t_triangle, t_trapeze)
    return np.where(d <= 0.0, 0.0, t)


def _profil_trapeze(t, d, T, a):
    """Position s(t) d'un profil trapézoïdal qui parcourt d en exactement T secondes.

    L'accélération a est gardée et la vitesse de croisière est réduite pour finir
    à T. Il faut T >= temps minimal du mouvement, ce qui garantit que la vitesse
    de croisière reste sous v_max.

    Le pic de vitesse v vérifie d = v*T - v²/a, soit v² - a*T*v + a*d = 0.
    """
    t = np.clip(np.asarray(t, dtype=float), 0.0, T)
    if d <= 0.0 or T <= 0.0:
        return np.zeros_like(t)

    discriminant = max((a * T)**2 - 4.0 * a * d, 0.0)
    v_pic = (a * T - np.sqrt(discriminant)) / 2.0
    t_acc = v_pic / a

    return np.where(
        t < t_acc,
        0.5 * a * t**2,                                   # accélération
        np.where(
            t <= T - t_acc,
            0.5 * a * t_acc**2 + v_pic * (t - t_acc),     # vitesse constante
            d - 0.5 * a * (T - t)**2,                     # décélération
        ),
    )


def _axe_temps(T, dt):
    """Axe du temps de 0 à T inclus, avec un pas effectif <= dt.

    Le dernier échantillon tombe exactement sur T : la trajectoire finit bien
    sur la pose d'arrivée, et timediagram retrouve les bons instants.
    """
    n = max(1, int(np.ceil(T / dt - 1e-9)))
    return np.linspace(0.0, T, n + 1)


def _facteur_ralentissement_bras(angles, T, v_max_deg_s, a_max_deg_s2):
    """Facteur k >= 1 par lequel il faut ralentir le mouvement pour respecter les
    limites des moteurs des bras (colonnes 0 à 2 de angles).

    Vitesses et accélérations sont estimées par différences finies sur les
    échantillons. Ralentir le mouvement d'un facteur k (même trajet, durée × k)
    divise les vitesses par k et les accélérations par k², d'où :
        k = max(1, v_mesurée / v_max, sqrt(a_mesurée / a_max))
    Les bords sont ignorés pour l'accélération : la différence finie y est
    d'ordre 1 et donne des valeurs parasites.
    """
    n = len(angles)
    if n < 2:
        return 1.0
    t = np.linspace(0.0, T, n)
    v = np.gradient(angles[:, :3], t, axis=0)
    k_v = np.max(np.abs(v)) / v_max_deg_s
    k_a = 0.0
    if n >= 6:
        a = np.gradient(v, t, axis=0)[2:-2]
        k_a = np.sqrt(np.max(np.abs(a)) / a_max_deg_s2)
    return max(1.0, float(k_v), float(k_a))


# ============================================================================
#                      TRAJECTOIRES
# ============================================================================

def lineartrajectory(start, end, stepsmm, stepsdeg=1.0, plus_court_chemin=False):
    """Discrétise une ligne droite en poses [x, y, z, rz].

    La rotation rz est interpolée en même temps que la position. Aucun pas ne
    dépasse stepsmm en translation ni stepsdeg en rotation.

    Args:
        start (array-like): pose initiale [x, y, z, rz]
        end (array-like): pose finale [x, y, z, rz]
        stepsmm (float): pas maximal en translation (mm)
        stepsdeg (float): pas maximal en rotation du plateau (degrés)
        plus_court_chemin (bool): faire tourner rz par le plus court chemin
            (180° au plus). Le rz d'arrivée est alors remplacé par un équivalent
            modulo 360° : voir _poses_depart_arrivee.

    Returns:
        numpy.ndarray: poses de shape (n_steps + 1, 4)
    """
    _verifier_limites(stepsmm=stepsmm, stepsdeg=stepsdeg)
    p0, p1 = _poses_depart_arrivee(start, end, plus_court_chemin)

    distance = np.linalg.norm(p1[:3] - p0[:3])
    rotation = abs(p1[3] - p0[3])
    steps = max(int(np.ceil(distance / stepsmm)), int(np.ceil(rotation / stepsdeg)), 1)
    return np.linspace(p0, p1, steps + 1)


def jointmotangles(start, end, deg_per_step, steps_per_second=1.0, plus_court_chemin=False):
    """Trajectoire articulaire linéaire (vitesse constante) sur les 4 moteurs.

    Le moteur qui a le plus grand angle à parcourir avance de deg_per_step par
    pas. Les trois autres ralentissent pour finir en même temps que lui.

    Le chemin est une droite dans l'espace des angles, PAS dans l'espace XYZ :
    le centre du plateau suit une courbe. Chaque configuration intermédiaire
    est vérifiée (ValueError si le robot ne peut pas la prendre). Pour une
    droite en XYZ, utiliser cartesian_trapezoidal_trajectory.

    Args:
        start (array-like): pose initiale [x, y, z, rz]
        end (array-like): pose finale [x, y, z, rz]
        deg_per_step (float): déplacement du moteur principal par pas (degrés)
        steps_per_second (float): nombre de pas par seconde
        plus_court_chemin (bool): faire tourner rz par le plus court chemin
            (180° au plus). Le rz d'arrivée est alors remplacé par un équivalent
            modulo 360° : voir _poses_depart_arrivee.

    Returns:
        tuple:
            numpy.ndarray: angles [θ1, θ2, θ3, θ4] de shape (n_steps + 1, 4)
            float: durée du mouvement en secondes
    """
    p0, p1 = _poses_depart_arrivee(start, end, plus_court_chemin)
    poses = np.vstack((p0, p1))
    q0, q1 = DeltaInverse4(poses)
    _verifier_atteignable(np.vstack((q0, q1)), poses)

    max_delta = np.max(np.abs(q1 - q0))
    if max_delta == 0 or deg_per_step == 0:
        return np.array([q0], dtype=float), 0.0

    if steps_per_second <= 0:
        raise ValueError("steps_per_second doit être strictement positif")

    n_steps = max(1, int(np.ceil(max_delta / abs(deg_per_step))))
    duration_sec = n_steps / steps_per_second
    trajectoire = np.linspace(q0, q1, n_steps + 1)
    _verifier_chemin_articulaire(trajectoire)
    return trajectoire, duration_sec


def jointmotangles_trapezoidal(start, end, v_max_deg_s, a_max_deg_s2, dt=0.001,
                               v_max_axe4_deg_s=None, a_max_axe4_deg_s2=None,
                               plus_court_chemin=False):
    """Trajectoire articulaire synchronisée à profils trapézoïdaux sur les 4 moteurs.

    Chaque moteur suit un profil accélération / vitesse constante / décélération.
    Le moteur le plus lent fixe la durée totale, et les autres réduisent leur
    vitesse de croisière pour finir au même instant.

    Le moteur 4 a ses propres limites, car ce n'est pas le même moteur que ceux
    des bras.

    Chaque moteur ayant son propre profil, le chemin n'est même pas une droite
    dans l'espace des angles, et le centre du plateau ne suit pas une droite
    en XYZ (écarts de plusieurs mm à plusieurs dizaines de mm). Chaque
    configuration échantillonnée est vérifiée (ValueError si le robot ne peut
    pas la prendre). Pour une droite en XYZ, utiliser
    cartesian_trapezoidal_trajectory.

    Args:
        start (array-like): pose initiale [x, y, z, rz]
        end (array-like): pose finale [x, y, z, rz]
        v_max_deg_s (float): vitesse max des moteurs des bras (degrés/s)
        a_max_deg_s2 (float): accélération max des moteurs des bras (degrés/s²)
        dt (float): pas de temps maximal (s), défaut 0.001
        v_max_axe4_deg_s (float): vitesse max du moteur 4 (degrés moteur/s),
            défaut V_MAX_AXE4_DEG_S
        a_max_axe4_deg_s2 (float): accélération max du moteur 4 (degrés moteur/s²),
            défaut A_MAX_AXE4_DEG_S2
        plus_court_chemin (bool): faire tourner rz par le plus court chemin
            (180° au plus). Le rz d'arrivée est alors remplacé par un équivalent
            modulo 360° : voir _poses_depart_arrivee.

    Returns:
        tuple:
            numpy.ndarray: angles [θ1, θ2, θ3, θ4] de shape (n, 4) en degrés
            float: durée totale du mouvement en secondes
    """
    v4 = V_MAX_AXE4_DEG_S if v_max_axe4_deg_s is None else v_max_axe4_deg_s
    a4 = A_MAX_AXE4_DEG_S2 if a_max_axe4_deg_s2 is None else a_max_axe4_deg_s2
    _verifier_limites(v_max_deg_s=v_max_deg_s, a_max_deg_s2=a_max_deg_s2,
                      v_max_axe4_deg_s=v4, a_max_axe4_deg_s2=a4, dt=dt)

    p0, p1 = _poses_depart_arrivee(start, end, plus_court_chemin)
    poses = np.vstack((p0, p1))
    q0, q1 = DeltaInverse4(poses)
    _verifier_atteignable(np.vstack((q0, q1)), poses)

    v_max = np.array([v_max_deg_s] * 3 + [v4], dtype=float)
    a_max = np.array([a_max_deg_s2] * 3 + [a4], dtype=float)
    deltas = q1 - q0
    distances = np.abs(deltas)
    signes = np.sign(deltas)

    # 1) Durée imposée par le moteur le plus lent
    t_total = float(np.max(_temps_min_trapeze(distances, v_max, a_max)))
    if t_total == 0.0:
        return np.array([q0], dtype=float), 0.0

    # 2) Profil synchronisé de chaque moteur sur la même durée
    temps = _axe_temps(t_total, dt)
    q_path = np.empty((len(temps), 4))
    for i in range(4):
        q_path[:, i] = q0[i] + signes[i] * _profil_trapeze(temps, distances[i], t_total, a_max[i])

    _verifier_chemin_articulaire(q_path)
    return q_path, t_total


def cartesian_trapezoidal_trajectory(start, end, v_max_mm_s, a_max_mm_s2, dt=0.001,
                                     v_max_axe4_deg_s=None, a_max_axe4_deg_s2=None,
                                     plus_court_chemin=False,
                                     v_max_deg_s=None, a_max_deg_s2=None):
    """Ligne droite en XYZ et rotation rz synchronisées, avec profil trapézoïdal.

    La translation et la rotation partagent le même profil normalisé
    λ(t) de 0 à 1 :
        pose(t) = départ + λ(t) * (arrivée - départ)
    Le plateau suit donc une ligne droite et tourne proportionnellement à
    l'avancement. Le mouvement le plus lent (translation ou rotation) fixe la
    durée, et l'autre est ralenti pour finir en même temps.

    Les limites de rotation du plateau sont déduites de celles du moteur 4 en
    divisant par RAPPORT_AXE4.

    Les moteurs des bras 1 à 3 ne suivent pas un profil trapézoïdal : leurs
    vitesses et accélérations dépendent de la position sur la droite. Si
    v_max_deg_s et a_max_deg_s2 sont donnés, le mouvement est ralenti (même
    trajet, durée plus longue) jusqu'à ce qu'aucun de ces trois moteurs ne les
    dépasse, avec une tolérance de 0,1 % (mesure par différences finies au pas
    dt). Sans ces deux limites, rien ne borne les moteurs des bras : à vérifier
    soi-même sur le diagramme.

    Args:
        start (array-like): pose initiale [x, y, z, rz]
        end (array-like): pose finale [x, y, z, rz]
        v_max_mm_s (float): vitesse linéaire max du plateau (mm/s)
        a_max_mm_s2 (float): accélération linéaire max du plateau (mm/s²)
        dt (float): pas de temps maximal (s), défaut 0.001
        v_max_axe4_deg_s (float): vitesse max du moteur 4 (degrés moteur/s),
            défaut V_MAX_AXE4_DEG_S
        a_max_axe4_deg_s2 (float): accélération max du moteur 4 (degrés moteur/s²),
            défaut A_MAX_AXE4_DEG_S2
        plus_court_chemin (bool): faire tourner rz par le plus court chemin
            (180° au plus). Le rz d'arrivée est alors remplacé par un équivalent
            modulo 360° : voir _poses_depart_arrivee.
        v_max_deg_s (float): vitesse max des moteurs des bras (degrés/s), optionnel
        a_max_deg_s2 (float): accélération max des moteurs des bras (degrés/s²),
            optionnel. À donner en même temps que v_max_deg_s.

    Returns:
        tuple:
            numpy.ndarray: angles [θ1, θ2, θ3, θ4] de shape (n, 4) en degrés
            float: durée totale du mouvement en secondes
    """
    v4 = V_MAX_AXE4_DEG_S if v_max_axe4_deg_s is None else v_max_axe4_deg_s
    a4 = A_MAX_AXE4_DEG_S2 if a_max_axe4_deg_s2 is None else a_max_axe4_deg_s2
    _verifier_limites(v_max_mm_s=v_max_mm_s, a_max_mm_s2=a_max_mm_s2,
                      v_max_axe4_deg_s=v4, a_max_axe4_deg_s2=a4, dt=dt)
    if (v_max_deg_s is None) != (a_max_deg_s2 is None):
        raise ValueError("v_max_deg_s et a_max_deg_s2 vont ensemble : donner les deux ou aucun")
    limiter_bras = v_max_deg_s is not None
    if limiter_bras:
        _verifier_limites(v_max_deg_s=v_max_deg_s, a_max_deg_s2=a_max_deg_s2)

    p0, p1 = _poses_depart_arrivee(start, end, plus_court_chemin)
    delta = p1 - p0
    distance = np.linalg.norm(delta[:3])      # mm
    rotation = abs(delta[3])                  # degrés plateau

    if distance == 0.0 and rotation == 0.0:
        q0 = DeltaInverse4(p0)
        _verifier_atteignable(q0, np.atleast_2d(p0))
        return q0, 0.0

    # 1) Limites sur l'avancement λ : chaque mouvement impose les siennes
    v_lambda, a_lambda = np.inf, np.inf
    if distance > 0.0:
        v_lambda = min(v_lambda, v_max_mm_s / distance)
        a_lambda = min(a_lambda, a_max_mm_s2 / distance)
    if rotation > 0.0:
        rapport = abs(RAPPORT_AXE4)
        v_lambda = min(v_lambda, (v4 / rapport) / rotation)
        a_lambda = min(a_lambda, (a4 / rapport) / rotation)

    # 2) Profil trapézoïdal de λ entre 0 et 1, puis poses intermédiaires et
    #    cinématique inverse (vectorisée). Si les bras sont limités, on ralentit
    #    d'un facteur k (v/k, a/k²) jusqu'à ce que leurs limites soient respectées.
    #    Le facteur est mesuré sur la trajectoire échantillonnée, donc on itère.
    k = 1.0
    for _ in range(10):
        v_l, a_l = v_lambda / k, a_lambda / k**2
        t_total = float(_temps_min_trapeze(1.0, v_l, a_l))
        temps = _axe_temps(t_total, dt)
        lam = _profil_trapeze(temps, 1.0, t_total, a_l)

        poses = p0 + lam[:, np.newaxis] * delta
        angles = DeltaInverse4(poses)
        _verifier_atteignable(angles, poses)

        if not limiter_bras:
            break
        facteur = _facteur_ralentissement_bras(angles, t_total, v_max_deg_s, a_max_deg_s2)
        if facteur <= 1.001:
            break
        k *= facteur
    else:
        raise RuntimeError("Limites des moteurs des bras non respectées après 10 itérations")

    return angles, t_total


# ============================================================================
#                      DIAGRAMMES TEMPS
# ============================================================================

def _styliser(ax, titre, xlabel, ylabel):
    """Applique un style sobre : grille discrète, axes gris, textes en encre neutre."""
    ax.set_facecolor(SURFACE)
    ax.set_title(titre, loc="left", fontsize=12, fontweight="semibold", color=ENCRE)
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=10, color=ENCRE_SECONDAIRE)
    ax.set_ylabel(ylabel, fontsize=10, color=ENCRE_SECONDAIRE)
    ax.grid(True, color=GRILLE, linewidth=0.6)
    ax.set_axisbelow(True)
    for cote in ("top", "right"):
        ax.spines[cote].set_visible(False)
    for cote in ("left", "bottom"):
        ax.spines[cote].set_color(AXES)
    ax.tick_params(colors=ENCRE_SECONDAIRE, labelsize=9)


def _etiquettes_fin(ax, x_fin, y_fins, etiquettes):
    """Écrit le nom de chaque courbe à sa fin, en écartant les noms trop proches."""
    y_min, y_max = ax.get_ylim()
    ecart_min = 0.07 * (y_max - y_min)

    ordre = np.argsort(y_fins)
    positions = np.array(y_fins, dtype=float)[ordre]
    for k in range(1, len(positions)):
        positions[k] = max(positions[k], positions[k - 1] + ecart_min)

    for k, i in enumerate(ordre):
        ax.annotate(etiquettes[i], xy=(x_fin, positions[k]), xytext=(6, 0),
                    textcoords="offset points", va="center", fontsize=10,
                    color=ENCRE_SECONDAIRE, annotation_clip=False)


def timediagram(trajectory, duration_sec, titre=None):
    """Trace les angles et les vitesses des 4 moteurs en fonction du temps.

    Graphique du haut : angles des 4 moteurs.
    Graphique du bas  : vitesses angulaires des 4 moteurs.

    Args:
        trajectory (numpy.ndarray): angles [θ1, θ2, θ3, θ4] de shape (n_steps + 1, 4),
            échantillonnés à intervalles réguliers
        duration_sec (float): durée totale du mouvement en secondes
        titre (str): titre de la figure (optionnel)

    Returns:
        matplotlib.figure.Figure: la figure, ou None si la trajectoire est vide
    """
    trajectory = np.asarray(trajectory, dtype=float)
    if trajectory.ndim != 2 or trajectory.shape[1] != 4:
        raise ValueError(f"Trajectoire attendue de shape (n, 4), reçu {trajectory.shape}")

    if len(trajectory) < 2:
        print("Avertissement : trajectoire vide, impossible de tracer")
        return None

    if duration_sec > 0:
        temps = np.linspace(0.0, duration_sec, len(trajectory))
        label_temps = "Temps (s)"
        unite_vitesse = "degrés/s"
    else:
        temps = np.arange(len(trajectory), dtype=float)
        label_temps = "Pas"
        unite_vitesse = "degrés/pas"

    # Vitesses angulaires par différences centrées
    vitesses = np.gradient(trajectory, temps, axis=0)

    fig, (ax_pos, ax_vel) = plt.subplots(2, 1, figsize=(12, 8), sharex=True,
                                         layout="constrained")
    fig.set_facecolor(SURFACE)

    noms = [r"$\theta_1$", r"$\theta_2$", r"$\theta_3$", r"$\theta_4$"]
    legendes = [f"{noms[i]} (Moteur {i + 1})" for i in range(3)]
    legendes.append(f"{noms[3]} (Moteur 4, rotation du plateau)")

    for i in range(4):
        ax_pos.plot(temps, trajectory[:, i], color=COULEURS_MOTEURS[i],
                    linewidth=EPAISSEUR_TRAIT, label=legendes[i])
        ax_vel.plot(temps, vitesses[:, i], color=COULEURS_MOTEURS[i],
                    linewidth=EPAISSEUR_TRAIT, label=legendes[i])

    _styliser(ax_pos, "Angles des moteurs", None, "Angle (degrés)")
    _styliser(ax_vel, "Vitesses angulaires des moteurs", label_temps,
              f"Vitesse angulaire ({unite_vitesse})")

    _etiquettes_fin(ax_pos, temps[-1], trajectory[-1, :], noms)

    fig.suptitle(titre or "Diagramme temps des moteurs du robot delta 4 axes",
                 fontsize=14, fontweight="bold", color=ENCRE)
    # Une seule légende sous les graphiques : elle ne cache jamais une courbe
    fig.legend(*ax_pos.get_legend_handles_labels(), loc="outside lower center", ncols=4,
               fontsize=10, frameon=False, labelcolor=ENCRE_SECONDAIRE)
    return fig


def resume_trajectoire(trajectory, duration_sec):
    """Affiche pour chaque moteur l'angle de départ, d'arrivée et la vitesse max atteinte."""
    trajectory = np.asarray(trajectory, dtype=float)
    print(f"Nombre de points : {len(trajectory)}")
    print(f"Durée            : {duration_sec:.3f} s")
    if len(trajectory) > 1 and duration_sec > 0:
        temps = np.linspace(0.0, duration_sec, len(trajectory))
        v_max = np.max(np.abs(np.gradient(trajectory, temps, axis=0)), axis=0)
    else:
        v_max = np.zeros(4)
    print(f"{'Moteur':>8} {'Départ (°)':>12} {'Arrivée (°)':>12} {'|v| max (°/s)':>15}")
    for i in range(4):
        print(f"{i + 1:>8} {trajectory[0, i]:>12.3f} {trajectory[-1, i]:>12.3f} {v_max[i]:>15.2f}")


# ============================================================================
#                      TESTS
# ============================================================================

def test(dossier_sortie=None, afficher=True):
    """Calcule trois trajectoires d'exemple et enregistre leurs diagrammes temps.

    Args:
        dossier_sortie (str | Path): dossier des images, défaut « Output » à la racine du projet
        afficher (bool): ouvrir les fenêtres matplotlib à la fin
    """
    dossier = Path(dossier_sortie) if dossier_sortie else RACINE_PROJET / "Output"
    dossier.mkdir(parents=True, exist_ok=True)

    depart = [0, 0, -300, 0]
    arrivee = [100, 150, -350, 90]

    print("=" * 70)
    print("TEST 1 : Trajectoire articulaire LINÉAIRE (jointmotangles)")
    print("=" * 70)
    traj_lin, duree_lin = jointmotangles(depart, arrivee, deg_per_step=0.5, steps_per_second=250)
    resume_trajectoire(traj_lin, duree_lin)

    print("\n" + "=" * 70)
    print("TEST 2 : Trajectoire articulaire TRAPÉZOÏDALE (jointmotangles_trapezoidal)")
    print("=" * 70)
    traj_trap, duree_trap = jointmotangles_trapezoidal(
        depart, arrivee, v_max_deg_s=30.0, a_max_deg_s2=60.0, dt=0.001
    )
    resume_trajectoire(traj_trap, duree_trap)
    fig = timediagram(traj_trap, duree_trap,
                      "Trajectoire articulaire trapézoïdale – robot delta 4 axes")
    if fig:
        chemin = dossier / "trajectoire4axes_trapezoidal.png"
        fig.savefig(chemin, dpi=150, bbox_inches="tight")
        print(f"Graphique sauvegardé : {chemin}")

    print("\n" + "=" * 70)
    print("TEST 3 : Ligne droite CARTÉSIENNE + rotation (cartesian_trapezoidal_trajectory)")
    print("=" * 70)
    traj_cart, duree_cart = cartesian_trapezoidal_trajectory(
        [-200, 200, -200, 0], [175, -250, -350, 180],
        v_max_mm_s=200.0, a_max_mm_s2=500.0, dt=0.01,
        v_max_deg_s=30.0, a_max_deg_s2=60.0
    )
    resume_trajectoire(traj_cart, duree_cart)
    fig = timediagram(traj_cart, duree_cart,
                      "Ligne droite cartésienne avec rotation – robot delta 4 axes")
    if fig:
        chemin = dossier / "trajectoire4axes_lineaire.png"
        fig.savefig(chemin, dpi=150, bbox_inches="tight")
        print(f"Graphique sauvegardé : {chemin}")

    if afficher:
        plt.show()


if __name__ == "__main__":
    test()
