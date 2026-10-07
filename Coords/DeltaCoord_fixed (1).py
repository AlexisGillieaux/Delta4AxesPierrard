"""
Cinématique du robot delta (3 bras) : inverse, directe, visualisation et résolution.

Repère : x, y horizontaux, z vertical (négatif sous la base). Angles moteurs en degrés,
θ = 0 quand le bras moteur est horizontal, positif quand il pointe vers le bas.
Positions en mm.

Un point hors de portée (cinématique inverse) ou des angles impossibles (cinématique
directe) donnent NaN. Une forme d'entrée invalide donne aussi [[NaN, NaN, NaN]].
"""

from math import sqrt

import numpy as np


# ============================================================================
#                      GÉOMÉTRIE DU ROBOT (mm)
# ============================================================================
# Une seule définition, utilisée par la cinématique inverse, la cinématique
# directe et la visualisation.

R_BASE = 200                    # Rayon du cercle sur lequel sont les moteurs
W_BASE = R_BASE / 2             # Rayon de la base du triangle équilatéral formé par les moteurs
S_BASE = R_BASE * 3 / sqrt(3)   # Côté du triangle équilatéral inscrit dans ce cercle
R_PLAT = 40                     # Rayon de la plateforme (distance du centre à un point d'attache)
W_PLAT = R_PLAT / 2             # Rayon de la base du triangle équilatéral formé par les points d'attache
S_PLAT = R_PLAT * 3 / sqrt(3)   # Côté du triangle équilatéral inscrit dans le cercle de la plateforme
L_BRAS = 200                    # Longueur des bras moteurs
L_BARRE = 430                   # Longueur des bras parallèles (extrémité du bras moteur -> point d'attache)


def rotate(x, y, angle_deg):
    """Rotation compatible avec arrays numpy et scalaires"""
    a = np.radians(angle_deg)
    return x*np.cos(a) - y*np.sin(a),  x*np.sin(a) + y*np.cos(a)

def solve_one_arm(xr, yr, z, wB, rP, L, l):
    """Résout pour un bras dont la direction est l'axe Y local
    Fonctionne avec des scalaires et arrays numpy. Retourne NaN si le point est hors de portée.

    Si G = E, tan(θ/2) est infini : θ vaut ±180° ou NaN, hors de toute plage physique."""
    xr, yr, z = (np.asarray(v, dtype=float) for v in (xr, yr, z))
    a = wB - rP
    E = 2*L*(yr + a)
    F = 2*L*z
    G = xr**2 + yr**2 + z**2 + a**2 + L**2 + 2*a*yr - l**2
    disc = E**2 + F**2 - G**2

    with np.errstate(invalid="ignore", divide="ignore"):
        t_minus = (-F - np.sqrt(disc)) / (G - E)
    t_minus = np.where(disc < 0, np.nan, t_minus)
    return np.degrees(2*np.arctan(t_minus))


def DeltaInverse(coordinates):
    """
    Calcule l'inverse cinématique pour un robot Delta.

    Paramètres:
    -----------
    coordinates : array numpy de shape (n, 3)
        Chaque ligne contient les coordonnées [x, y, z] d'un point

    Retour:
    -------
    array numpy de shape (n, 3)
        Chaque ligne contient les angles [t1, t2, t3] des trois moteurs.
        Une ligne vaut NaN si le point est hors de portée. Si la forme de
        coordinates est invalide, retourne [[NaN, NaN, NaN]].
    """
    # Convertir en array numpy si nécessaire
    coords = np.atleast_2d(np.array(coordinates, dtype=float))

    # Vérifier la forme
    if (coords.ndim != 2) | (coords.shape[1] != 3):
        return np.array([[np.nan, np.nan, np.nan]])

    x = coords[:, 0]
    y = coords[:, 1]
    z = coords[:, 2]

    # Bras 1 : pas de rotation
    x1, y1 = rotate(x, y, 0)
    t1 = solve_one_arm(x1, y1, z, W_BASE, R_PLAT, L_BRAS, L_BARRE)

    # Bras 2 : rotation de -120°
    x2, y2 = rotate(x, y, -120)
    t2 = solve_one_arm(x2, y2, z, W_BASE, R_PLAT, L_BRAS, L_BARRE)

    # Bras 3 : rotation de +120°
    x3, y3 = rotate(x, y, 120)
    t3 = solve_one_arm(x3, y3, z, W_BASE, R_PLAT, L_BRAS, L_BARRE)

    # Retourner un array de shape (n, 3)
    return np.column_stack((t1, t2, t3))


def DeltaForward(angles):
    """
    Calcule la cinématique directe pour un robot Delta.
    Fonctionne complètement vectorisé avec les arrays numpy.

    Paramètres:
    -----------
    angles : array numpy de shape (n, 3)
        Chaque ligne contient les angles [t1, t2, t3] des trois moteurs (en degrés)

    Retour:
    -------
    array numpy de shape (n, 3)
        Chaque ligne contient les coordonnées [x, y, z] de la plateforme
        (solution avec la plateforme sous les moteurs). Une ligne vaut NaN si les
        angles ne correspondent à aucune position (les barres ne peuvent pas se
        fermer sur la plateforme). Si la forme de angles est invalide, retourne
        [[NaN, NaN, NaN]].
    """
    np_angles = np.atleast_2d(np.array(angles, dtype=float))

    # Vérifier la forme
    if (np_angles.ndim != 2) | (np_angles.shape[1] != 3):
        return np.array([[np.nan, np.nan, np.nan]])

    t1_rad = np.radians(np_angles[:, 0])
    t2_rad = np.radians(np_angles[:, 1])
    t3_rad = np.radians(np_angles[:, 2])

    # Centres des trois sphères : extrémités des bras moteurs, décalées de l'attache
    # des barres sur la plateforme (le point cherché est à L_BARRE de chacun)
    A1 = np.column_stack((np.zeros_like(t1_rad), -W_BASE - L_BRAS*np.cos(t1_rad) + R_PLAT, -L_BRAS*np.sin(t1_rad)))
    A2 = np.column_stack(((sqrt(3)/2)*(W_BASE + L_BRAS*np.cos(t2_rad)) - S_PLAT/2, (1/2)*(W_BASE + L_BRAS*np.cos(t2_rad)) - W_PLAT, -L_BRAS*np.sin(t2_rad)))
    A3 = np.column_stack((-(sqrt(3)/2)*(W_BASE + L_BRAS*np.cos(t3_rad)) + S_PLAT/2, (1/2)*(W_BASE + L_BRAS*np.cos(t3_rad)) - W_PLAT, -L_BRAS*np.sin(t3_rad)))

    # Les trois sphères de rayon L_BARRE centrées sur A1, A2 et A3 se coupent en
    # deux points, symétriques par rapport au plan (A1, A2, A3). On les obtient par
    # trilatération, dans le repère (ex, ey, ez) lié aux trois centres :
    #   ex : de A1 vers A2,  ey : dans le plan des centres, perpendiculaire à ex,
    #   ez : normale au plan.
    # Aucune division par une différence de hauteur des centres : la méthode est
    # valable pour toutes les configurations, y compris quand deux moteurs (plans
    # de symétrie du robot) ou les trois ont le même angle.
    # Si les sphères ne se coupent pas (ou si les centres sont alignés), le
    # résultat est NaN.
    with np.errstate(invalid="ignore", divide="ignore"):
        v12 = A2 - A1
        v13 = A3 - A1
        d = np.linalg.norm(v12, axis=1)                    # |A1A2|
        ex = v12 / d[:, None]
        i = np.sum(ex * v13, axis=1)                       # composante de A1A3 sur ex
        ey = v13 - i[:, None] * ex
        j = np.linalg.norm(ey, axis=1)                     # composante de A1A3 sur ey
        ey = ey / j[:, None]
        ez = np.cross(ex, ey)

        # Dans ce repère, A1 = (0, 0), A2 = (d, 0), A3 = (i, j)
        xl = d / 2.0
        yl = (i**2 + j**2 - 2.0*i*xl) / (2.0*j)
        zl = np.sqrt(L_BARRE**2 - xl**2 - yl**2)           # NaN si pas d'intersection

        base_pts = A1 + xl[:, None]*ex + yl[:, None]*ey
        p_plus = base_pts + zl[:, None]*ez
        p_moins = base_pts - zl[:, None]*ez

    # Choisir la solution avec z le plus bas (plateforme en dessous des moteurs)
    mask_plus = p_plus[:, 2] < p_moins[:, 2]
    return np.where(mask_plus[:, None], p_plus, p_moins)


def test_aller_retour(tolerance_mm=1e-6, n_points=2000, graine=0):
    """Vérifie que DeltaForward(DeltaInverse(P)) redonne P, sans rien afficher d'autre que le bilan.

    Les poses testées sont : des points quelconques, puis des points sur les trois
    plans de symétrie du robot (deux moteurs ont le même angle) et sur l'axe
    vertical (les trois moteurs ont le même angle). Ce sont les cas où la
    cinématique directe a déjà été fausse (erreurs de plusieurs dizaines de mm).

    Parametres:
    -----------
    tolerance_mm : float
        Erreur maximale acceptée entre la pose de départ et la pose retrouvée
    n_points : int
        Nombre de points par famille
    graine : int
        Graine du générateur aléatoire

    Retour:
    -----------
    float : l'erreur maximale trouvée (mm). Lève AssertionError si elle dépasse tolerance_mm.
    """
    rng = np.random.default_rng(graine)
    y = rng.uniform(-120, 120, n_points)
    z = rng.uniform(-430, -200, n_points)
    plan_x0 = np.column_stack((np.zeros(n_points), y, z))      # theta2 = theta3

    familles = {
        "points quelconques": np.column_stack((rng.uniform(-200, 200, n_points),
                                               rng.uniform(-200, 200, n_points), z)),
        "plan theta2 = theta3": plan_x0,
        "plan theta1 = theta3": np.column_stack(rotate(plan_x0[:, 0], plan_x0[:, 1], 120) + (z,)),
        "plan theta1 = theta2": np.column_stack(rotate(plan_x0[:, 0], plan_x0[:, 1], -120) + (z,)),
        "axe vertical": np.column_stack((np.zeros(n_points), np.zeros(n_points), z)),
    }

    pire = 0.0
    for nom, poses in familles.items():
        angles = DeltaInverse(poses)
        atteignables = ~np.isnan(angles).any(axis=1)
        erreurs = np.linalg.norm(DeltaForward(angles[atteignables]) - poses[atteignables], axis=1)
        erreur_max = float(np.max(erreurs))
        print(f"{nom:<20} {int(atteignables.sum()):>5} poses atteignables, erreur max = {erreur_max:.2e} mm")
        pire = max(pire, erreur_max)

    assert pire <= tolerance_mm, f"Aller-retour FK(IK(P)) trop imprécis : {pire:.3e} mm > {tolerance_mm} mm"
    return pire


def test_forward_inverse():
    """Valide les cas inverse forward simples ainsi que les edges cases en printant les résultats obtenus.

    Vérifie aussi (assertions) que l'aller-retour cinématique redonne les angles de départ,
    puis lance test_aller_retour() et ouvre les deux visualisations.

    Parramètres:
    -----------
    Void

    Retour:
    -----------
    Void

    """
    # Affichage sans notation scientifique, limité à ce test (pas de réglage global de numpy)
    with np.printoptions(suppress=True, precision=9, linewidth=200):
        # Exemple d'utilisation avec un array unique
        print("Inverse (single point):", DeltaInverse([[250, 250, -200]]))
        print("Inverse (single point):", DeltaInverse([[0, 0, -300]]))

        # Exemple d'utilisation avec plusieurs points
        test_coords = np.array([
            [250, 250, -200],
            [0, 0, -300],
            [0, 0, -342.5],
            [2500,2500,-2500],
            [50,50,0]
        ])
        print("\nInverse (multiple points):")
        print(DeltaInverse(test_coords))
        print("Inverse (erreur format):", DeltaInverse([[0, 0]]))

        # Exemple d'utilisation avec un array unique
        print("Forward (single angles):", DeltaForward([[78.5, -61.8, 57.4]]))
        print("Forward (single angles):", DeltaForward([[-13.5, -13.5, -13.5]]))

        # Exemple d'utilisation avec plusieurs points
        test_angles = np.array([
            [78.5, -61.8, 57.4],
            [-13.5, -13.5, -13.5],
            [0, 0, 0]
        ])
        print("\nForward (multiple angles):")
        print(DeltaForward(test_angles))
        print("\nInverse(Forward(test_angles)):")
        retour = DeltaInverse(DeltaForward(test_angles))
        print(retour)
        assert np.allclose(retour, test_angles, atol=1e-6), "Inverse(Forward(angles)) ne redonne pas les angles"
        print("Forward (erreur format):", DeltaForward([[0, 0]]))

    print("\nAller-retour sur les plans de symétrie :")
    test_aller_retour()

    visualisation(angles=test_angles[0])

    visualisation(position=DeltaForward(test_angles[0])[0])


def visualisation(position=None, angles=None, ax=None, show_plot=True):
    """
    Affiche une visualisation 3D de la configuration du robot Delta.

    Paramètres:
    -----------
    position : array-like de shape (3,), optionnel
        Coordonnées [x, y, z] de la plateforme outil
    angles : array-like de shape (3,), optionnel
        Angles [theta1, theta2, theta3] des trois moteurs en degrés
    ax : matplotlib Axes3D, optionnel
        Axes sur lesquels tracer (pour sous-plots)
    show_plot : bool
        Si True, affiche la figure avec plt.show()

    Si position est fourni, calcule les angles correspondants.
    Si angles est fourni, calcule la position correspondante.
    Si aucun n'est fourni, utilise une position par défaut.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (enregistre la projection '3d' des anciennes versions)

    # Calculer angles et position
    if position is not None:
        angles_calc = DeltaInverse([position])[0]
        print(f"Angles calculés pour position {position}: theta1={angles_calc[0]:.2f}°, theta2={angles_calc[1]:.2f}°, theta3={angles_calc[2]:.2f}°")
        if np.isnan(angles_calc[0]):
            print("Position hors de portée du robot")
            return
        angles = angles_calc
    elif angles is not None:
        position_calc = DeltaForward([angles])[0]
        print(f"Position calculée pour angles {angles}: x={position_calc[0]:.2f} mm, y={position_calc[1]:.2f} mm, z={position_calc[2]:.2f} mm")
        if np.isnan(position_calc[0]):
            print("Angles invalides")
            return
        position = position_calc
    else:
        # Position par défaut
        position = [0, 0, -300]
        angles = DeltaInverse([position])[0]

    x0, y0, z0 = position
    theta1, theta2, theta3 = angles

    # Créer la figure si nécessaire
    if ax is None:
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
    else:
        fig = ax.figure

    # Positions des moteurs (base fixe)
    # Le bras 1 pointe dans la direction -Y ; les bras 2 et 3 sont tournés de +120° et +240°.
    motor1 = (0, -W_BASE, 0)
    motor2 = ( W_BASE*sqrt(3)/2,  W_BASE/2, 0)   # rotate((0,-wB,0), +120°)
    motor3 = (-W_BASE*sqrt(3)/2,  W_BASE/2, 0)   # rotate((0,-wB,0), +240°)

    base1 = (S_BASE/2,  W_BASE, 0)
    base2 = (-S_BASE/2, W_BASE, 0)
    base3 = (0, -R_BASE, 0)

    # Positions des extrémités des bras moteurs
    # Pour le bras i, le coude est à : motor_i + L*(0, -cos(θ_i), -sin(θ_i)) dans le repère local,
    # ce qui donne après rotation par +k*120° :
    arm1_end = (0,
                -W_BASE - L_BRAS*np.cos(np.radians(theta1)),
                -L_BRAS*np.sin(np.radians(theta1)))
    arm2_end = ( (W_BASE + L_BRAS*np.cos(np.radians(theta2)))*sqrt(3)/2,
                 (W_BASE + L_BRAS*np.cos(np.radians(theta2)))/2,
                -L_BRAS*np.sin(np.radians(theta2)))
    arm3_end = (-(W_BASE + L_BRAS*np.cos(np.radians(theta3)))*sqrt(3)/2,
                 (W_BASE + L_BRAS*np.cos(np.radians(theta3)))/2,
                -L_BRAS*np.sin(np.radians(theta3)))

    # Positions des points d'attache sur la plateforme outil
    # Le point d'attache du bras 1 est décalé de -rP selon Y (même convention que -Y pour le bras 1).
    platform1 = (x0,                       y0 - R_PLAT,    z0)
    platform2 = (x0 + R_PLAT*sqrt(3)/2,    y0 + R_PLAT/2,  z0)   # rotate((0,-rP), +120°) + centre
    platform3 = (x0 - R_PLAT*sqrt(3)/2,    y0 + R_PLAT/2,  z0)   # rotate((0,-rP), +240°) + centre

    # Tracer la base fixe (plateforme supérieure)
    base_points = np.array([base1, base2, base3, base1])
    ax.plot(base_points[:, 0], base_points[:, 1], base_points[:, 2],
            'k-', linewidth=3, label='Base fixe')

    # Tracer les moteurs
    ax.scatter(*motor1, color='red', s=100, label='Moteur 1')
    ax.scatter(*motor2, color='green', s=100, label='Moteur 2')
    ax.scatter(*motor3, color='blue', s=100, label='Moteur 3')

    # Tracer les bras moteurs
    ax.plot([motor1[0], arm1_end[0]], [motor1[1], arm1_end[1]], [motor1[2], arm1_end[2]],
            'r-', linewidth=4, label='Bras moteur 1')
    ax.plot([motor2[0], arm2_end[0]], [motor2[1], arm2_end[1]], [motor2[2], arm2_end[2]],
            'g-', linewidth=4, label='Bras moteur 2')
    ax.plot([motor3[0], arm3_end[0]], [motor3[1], arm3_end[1]], [motor3[2], arm3_end[2]],
            'b-', linewidth=4, label='Bras moteur 3')

    # Tracer les extrémités des bras moteurs
    ax.scatter(*arm1_end, color='red', s=50, marker='s')
    ax.scatter(*arm2_end, color='green', s=50, marker='s')
    ax.scatter(*arm3_end, color='blue', s=50, marker='s')

    # Tracer les bras parallèles (connecteurs)
    ax.plot([arm1_end[0], platform1[0]], [arm1_end[1], platform1[1]], [arm1_end[2], platform1[2]],
            'r--', linewidth=2, label='Connecteur 1')
    ax.plot([arm2_end[0], platform2[0]], [arm2_end[1], platform2[1]], [arm2_end[2], platform2[2]],
            'g--', linewidth=2, label='Connecteur 2')
    ax.plot([arm3_end[0], platform3[0]], [arm3_end[1], platform3[1]], [arm3_end[2], platform3[2]],
            'b--', linewidth=2, label='Connecteur 3')
    length1 = np.sqrt((arm1_end[0]-platform1[0])**2 + (arm1_end[1]-platform1[1])**2 + (arm1_end[2]-platform1[2])**2)
    length2 = np.sqrt((arm2_end[0]-platform2[0])**2 + (arm2_end[1]-platform2[1])**2 + (arm2_end[2]-platform2[2])**2)
    length3 = np.sqrt((arm3_end[0]-platform3[0])**2 + (arm3_end[1]-platform3[1])**2 + (arm3_end[2]-platform3[2])**2)
    print(f"Longueurs des connecteurs: Connecteur 1 = {length1:.2f} mm, Connecteur 2 = {length2:.2f} mm, Connecteur 3 = {length3:.2f} mm")

    # Tracer la plateforme outil (plateforme inférieure)
    platform_points = np.array([platform1, platform2, platform3, platform1])
    ax.plot(platform_points[:, 0], platform_points[:, 1], platform_points[:, 2],
            'k-', linewidth=3, label='Plateforme outil')

    # Tracer les points d'attache sur la plateforme
    ax.scatter(*platform1, color='red', s=50, marker='^')
    ax.scatter(*platform2, color='green', s=50, marker='^')
    ax.scatter(*platform3, color='blue', s=50, marker='^')

    # Configuration de l'affichage
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title(f"Robot Delta : plateforme en ({x0:.1f}, {y0:.1f}, {z0:.1f}) mm\n"
                 f"angles moteurs ({theta1:.1f}°, {theta2:.1f}°, {theta3:.1f}°)")

    # Ajuster les limites pour une meilleure visualisation
    all_points = np.vstack([base_points, platform_points])

    ax.set_xlim([np.min(all_points[:, 0]) - 50, np.max(all_points[:, 0]) + 50])
    ax.set_ylim([np.min(all_points[:, 1]) - 50, np.max(all_points[:, 1]) + 50])
    ax.set_zlim([np.min(all_points[:, 2]) - 50, np.max(all_points[:, 2]) + 50])

    ax.legend()
    ax.grid(True)

    # Inverser l'axe Z pour que la base soit en haut
    ax.invert_zaxis()

    if show_plot:
        plt.tight_layout()
        plt.show()

    # Afficher les informations
    print(f"Position de la plateforme: x={x0:.2f}, y={y0:.2f}, z={z0:.2f} mm")
    print(f"Angles des moteurs: theta1={theta1:.2f}°, theta2={theta2:.2f}°, theta3={theta3:.2f}°")

    return fig, ax


# ============================================================================
#                      RÉSOLUTION DANS L'ESPACE DE TRAVAIL
# ============================================================================

# Plage angulaire supposée atteignable par les moteurs (degrés ; θ = 0 : bras
# horizontal). Une configuration hors de cette plage est exclue de l'analyse de
# résolution. Ce n'est PAS une propriété de la géométrie (la cinématique inverse
# n'a qu'une seule branche, et elle donne des angles au-delà de ±90° pour les
# coins de la zone de travail) : à adapter aux butées réelles des moteurs.
ANGLE_MIN_DEG = -90.0
ANGLE_MAX_DEG =  90.0

# Les 8 combinaisons de décalages (+/- un demi-pas) appliquées aux 3 moteurs : shape (8, 3)
_SIGNES_DEMI_PAS = np.array([
    [ 1.0,  1.0,  1.0],
    [ 1.0,  1.0, -1.0],
    [ 1.0, -1.0,  1.0],
    [ 1.0, -1.0, -1.0],
    [-1.0,  1.0,  1.0],
    [-1.0,  1.0, -1.0],
    [-1.0, -1.0,  1.0],
    [-1.0, -1.0, -1.0],
], dtype=float)


def _angles_are_physical(angles):
    """
    Retourne un masque booléen : True si les angles sont dans la plage
    [ANGLE_MIN_DEG, ANGLE_MAX_DEG]. Un NaN donne False.

    Paramètres:
    -----------
    angles : np.array de shape (n, 3)

    Retour:
    -------
    np.array de shape (n,) de booléens
    """
    return np.all(
        (angles >= ANGLE_MIN_DEG) & (angles <= ANGLE_MAX_DEG),
        axis=1
    )


def _deplacement_max_demi_pas(angles, half_step_deg):
    """Déplacement cartésien maximal quand les 3 moteurs sont décalés d'un demi-pas.

    Pour chaque configuration, les 8 combinaisons de +/- un demi-pas sur les 3 moteurs
    sont essayées (pire cas, pas une résolution par moteur). Une combinaison est ignorée
    si un angle sort de la plage [ANGLE_MIN_DEG, ANGLE_MAX_DEG] ou si DeltaForward donne NaN.

    Paramètres:
    -----------
    angles : np.array de shape (k, 3), configurations valides
    half_step_deg : float, demi-pas des moteurs (degrés)

    Retour:
    -------
    np.array de shape (k,) : déplacement maximal en mm, NaN si aucune combinaison n'est valide
    """
    nominal_positions = DeltaForward(angles)                                  # (k, 3)

    # Angles perturbés pour les 8 combinaisons : (8, k, 3)
    perturbed_angles = angles[np.newaxis, :, :] + _SIGNES_DEMI_PAS[:, np.newaxis, :] * half_step_deg
    perturbed_physical = np.all(
        (perturbed_angles >= ANGLE_MIN_DEG) & (perturbed_angles <= ANGLE_MAX_DEG),
        axis=2
    )                                                                         # (8, k)

    # Toutes les positions perturbées en un seul appel DeltaForward
    candidate_pos = DeltaForward(perturbed_angles.reshape(-1, 3)).reshape(8, -1, 3)
    displacements = np.linalg.norm(candidate_pos - nominal_positions[np.newaxis, :, :], axis=2)  # (8, k)

    # -inf : combinaison ignorée, sans passer par nanmax (qui avertit sur les colonnes toutes NaN)
    ignoree = ~perturbed_physical | np.isnan(displacements)
    maximum = np.where(ignoree, -np.inf, displacements).max(axis=0)
    maximum[np.isneginf(maximum)] = np.nan
    return maximum


def resolution_xyz(pas_par_tour, chunk_size=4096):
    """Détermine le déplacement cartésien maximal dû à un demi-pas moteur, pour tous les points XYZ de la zone de travail.

    Pour chaque point, on calcule les angles moteurs (DeltaInverse), puis on décale
    les 3 moteurs ensemble d'un demi-pas, en + ou en - (8 combinaisons), et on mesure
    le déplacement de la plateforme avec DeltaForward. C'est un pire cas, pas une
    résolution par moteur. La grille va de -250 à 250 en x et y et de -450 à -150 en z,
    avec un pas de 1 mm.

    Hypothèses :
    - pas_par_tour est le nombre de pas par tour de l'articulation du bras elle-même
      (pas de rapport de réduction supplémentaire) : demi-pas = 180° / pas_par_tour.
    - Seuls les points dont les 3 angles sont dans [ANGLE_MIN_DEG, ANGLE_MAX_DEG] sont
      évalués, et les combinaisons de demi-pas qui sortent de cette plage sont ignorées.

    Args:
        pas_par_tour (int): nombre de pas par tour des moteurs
        chunk_size (int): nombre de points traités par chunk

    Returns:
        resolutions (np.array): déplacement maximal (mm) pour chaque point évalué
        max_resolution (float): déplacement maximal trouvé (mm)
        max_resolution_position (tuple): position (x, y, z) du pire cas
    """
    pas_par_tour = int(pas_par_tour)
    if pas_par_tour <= 0:
        return np.array([], dtype=np.float32), np.nan, (np.nan, np.nan, np.nan)

    half_step_deg = (360.0 / pas_par_tour) / 2.0

    x_values = np.arange(-250, 251, 1, dtype=float)
    y_values = np.arange(-250, 251, 1, dtype=float)
    z_values = np.arange(-450, -149, 1, dtype=float)

    # Construction une seule fois du meshgrid XY
    xx, yy = np.meshgrid(x_values, y_values, indexing='xy')
    coords_xy = np.column_stack((xx.ravel(), yy.ravel()))
    n_xy = coords_xy.shape[0]

    max_resolution = 0.0
    max_resolution_position = (np.nan, np.nan, np.nan)
    resolutions_chunks = []

    for z in z_values:
        z_column = np.full(n_xy, z, dtype=float)

        for start in range(0, n_xy, chunk_size):
            stop = min(start + chunk_size, n_xy)
            coords = np.column_stack((coords_xy[start:stop], z_column[start:stop]))
            angles = DeltaInverse(coords)

            # Points atteignables ET dont les 3 angles sont dans la plage supposée
            valid_mask = ~np.isnan(angles).any(axis=1) & _angles_are_physical(angles)
            if not np.any(valid_mask):
                continue

            max_displacements = _deplacement_max_demi_pas(angles[valid_mask], half_step_deg)
            utilisables = ~np.isnan(max_displacements)
            if not np.any(utilisables):
                continue

            resultats = max_displacements[utilisables]
            meilleur = int(np.argmax(resultats))
            if resultats[meilleur] > max_resolution:
                max_resolution = float(resultats[meilleur])
                # Indice dans coords_xy : décalage du chunk + rang parmi les points valides, puis utilisables
                indice = start + np.flatnonzero(valid_mask)[np.flatnonzero(utilisables)[meilleur]]
                max_resolution_position = (
                    float(coords_xy[indice, 0]),
                    float(coords_xy[indice, 1]),
                    float(z)
                )

            resolutions_chunks.append(resultats.astype(np.float32))

    resolutions = np.concatenate(resolutions_chunks) if resolutions_chunks \
                  else np.array([], dtype=np.float32)

    return resolutions, float(max_resolution), max_resolution_position


def visualisation_resolution(pas_par_tour, z_slice=None, chunk_size=4096, show_plot=True):
    """
    Visualise la carte du déplacement maximal dû à un demi-pas moteur, dans le plan XY
    pour une tranche Z donnée (même grandeur que resolution_xyz, grille de 2 mm).

    Paramètres:
    -----------
    pas_par_tour : int
        Nombre de pas par tour des moteurs
    z_slice : float, optionnel
        Hauteur Z à visualiser. Si None, utilise Z = -300 mm (milieu du workspace).
    chunk_size : int
        Conservé pour compatibilité, non utilisé (la tranche est calculée d'un bloc).
    show_plot : bool
        Si True, affiche la figure avec plt.show()

    Retour:
    -------
    fig : matplotlib Figure, ou None si aucun point de la tranche n'est exploitable
    """
    import matplotlib.pyplot as plt

    if z_slice is None:
        z_slice = -300.0

    pas_par_tour = int(pas_par_tour)
    if pas_par_tour <= 0:
        raise ValueError(f"pas_par_tour doit être strictement positif (reçu {pas_par_tour})")
    half_step_deg = (360.0 / pas_par_tour) / 2.0

    x_values = np.arange(-250, 251, 2, dtype=float)   # pas de 2mm pour la visu
    y_values = np.arange(-250, 251, 2, dtype=float)
    xx, yy = np.meshgrid(x_values, y_values, indexing='xy')
    coords_xy = np.column_stack((xx.ravel(), yy.ravel()))
    n_xy = coords_xy.shape[0]

    coords = np.column_stack((coords_xy, np.full(n_xy, z_slice)))
    angles = DeltaInverse(coords)

    atteignable = ~np.isnan(angles).any(axis=1)
    physique = _angles_are_physical(angles)
    valid_mask = atteignable & physique
    hors_plage = atteignable & ~physique        # atteignable, mais au moins un moteur hors plage

    resolution_map = np.full(n_xy, np.nan)
    t1_map = np.full(n_xy, np.nan)              # pour diagnostiquer
    if np.any(valid_mask):
        resolution_map[valid_mask] = _deplacement_max_demi_pas(angles[valid_mask], half_step_deg)
        t1_map[valid_mask] = angles[valid_mask, 0]

    shape_2d = (len(y_values), len(x_values))
    res_2d = resolution_map.reshape(shape_2d)
    t1_2d = t1_map.reshape(shape_2d)
    hors_plage_2d = hors_plage.reshape(shape_2d)

    if not np.isfinite(res_2d).any():
        print(f"Aucun point exploitable à Z = {z_slice:.0f} mm")
        return None

    etiquette = 'Déplacement max pour ±½ pas\nsur les 3 moteurs (mm)'
    vmax_95 = float(np.nanpercentile(res_2d, 95))
    vmax_max = float(np.nanmax(res_2d))
    extent = [x_values[0], x_values[-1], y_values[0], y_values[-1]]

    # ---- Affichage ----
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle(f'Analyse résolution — Z = {z_slice:.0f} mm  |  {pas_par_tour} pas/tour',
                 fontsize=14, fontweight='bold')

    # --- Subplot 1 : échelle limitée au 95e percentile (meilleur contraste) ---
    ax1 = axes[0]
    im1 = ax1.imshow(res_2d, origin='lower', extent=extent, cmap='viridis', vmin=0, vmax=vmax_95)
    plt.colorbar(im1, ax=ax1, label=etiquette, extend='max')
    ax1.set_title('Échelle limitée au 95e percentile\n(au-delà : couleur saturée)')
    ax1.set_xlabel('X (mm)'); ax1.set_ylabel('Y (mm)')

    # --- Subplot 2 : échelle complète, de 0 au maximum réel ---
    ax2 = axes[1]
    im2 = ax2.imshow(res_2d, origin='lower', extent=extent, cmap='viridis', vmin=0, vmax=vmax_max)
    ax2.imshow(np.where(np.isnan(res_2d), 1.0, np.nan), origin='lower', extent=extent,
               cmap='Greys', vmin=0, vmax=1, alpha=0.4)
    plt.colorbar(im2, ax=ax2, label=etiquette)
    ax2.set_title(f'Échelle complète (maximum {vmax_max:.2f} mm)\nGris = hors portée ou angle hors plage')
    ax2.set_xlabel('X (mm)'); ax2.set_ylabel('Y (mm)')

    # --- Subplot 3 : angle t1, et points exclus car un moteur est hors plage ---
    ax3 = axes[2]
    im3 = ax3.imshow(t1_2d, origin='lower', extent=extent,
                     cmap='RdYlGn', vmin=ANGLE_MIN_DEG, vmax=ANGLE_MAX_DEG)
    ax3.imshow(np.where(hors_plage_2d, 1.0, np.nan), origin='lower', extent=extent,
               cmap='Reds', vmin=0, vmax=1, alpha=0.6)
    plt.colorbar(im3, ax=ax3, label='θ1 (degrés)')
    ax3.set_title(f'Angle θ1 — rouge = au moins un moteur hors [{ANGLE_MIN_DEG:.0f}°, {ANGLE_MAX_DEG:.0f}°]\n'
                  f'(points exclus de l\'analyse)')
    ax3.set_xlabel('X (mm)'); ax3.set_ylabel('Y (mm)')

    # Stats
    valid_res = res_2d[np.isfinite(res_2d)]
    print(f"\n{'='*55}")
    print(f"  Résolution à Z = {z_slice:.0f} mm  |  {pas_par_tour} pas/tour")
    print(f"{'='*55}")
    print(f"  Points évalués        : {valid_res.size}")
    print(f"  Points hors plage     : {int(hors_plage.sum())}")
    print(f"  Résolution min        : {np.min(valid_res):.3f} mm")
    print(f"  Résolution médiane    : {np.median(valid_res):.3f} mm")
    print(f"  95e percentile        : {vmax_95:.3f} mm")
    print(f"  Résolution max        : {vmax_max:.3f} mm")
    print(f"{'='*55}\n")

    plt.tight_layout()
    if show_plot:
        plt.show()
    return fig


if __name__ == "__main__":
    # --- Test rapide de résolution ---
    print("Calcul de la résolution (peut prendre quelques minutes)...")
    res, max_res, max_res_pos = resolution_xyz(875, chunk_size=4096)
    print(f"Résolution maximale : {max_res:.4f} mm  "
          f"au point (x={max_res_pos[0]:.1f}, y={max_res_pos[1]:.1f}, z={max_res_pos[2]:.1f})")
    if res.size:
        print(f"Résolution médiane  : {np.median(res):.4f} mm")
        print(f"Résolution 95e pct  : {np.percentile(res, 95):.4f} mm")

    # # --- Visualisation pour Z = -300 mm ---
    visualisation_resolution(pas_par_tour=650, z_slice=-300.0)
