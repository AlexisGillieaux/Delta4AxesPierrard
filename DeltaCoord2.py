from math import sqrt, sin, cos, atan2, pi, sqrt, tan
import numpy as np

# Configuration affichage numpy : supprimer la notation scientifique
np.set_printoptions(suppress=True, precision=9, linewidth=200)

# Constantes trigonométriques
sqrt3  = sqrt(3.0)
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60  = sqrt3
sin30  = 0.5
tan30  = 1.0 / sqrt3


def delta_calcAngleYZ(f, e, rf, re, x0, y0, z0):
    """Calcule l'angle theta pour le plan YZ (bras 1)
    Fonctionne avec des scalaires et arrays numpy"""
    y1 = -0.5 * 0.57735 * f   # f/2 * tan(30°)
    y0_adj = y0 - 0.5 * 0.57735 * e   # décalage du centre vers le bord

    # z = a + b*y
    a = (x0*x0 + y0_adj*y0_adj + z0*z0 + rf*rf - re*re - y1*y1) / (2.0 * z0)
    b = (y1 - y0_adj) / z0

    # discriminant
    d = -(a + b*y1)*(a + b*y1) + rf*(b*b*rf + rf)
    
    # Créer un array pour theta
    theta = np.zeros_like(x0)
    
    # Masque pour les cas valides
    valid_mask = d >= 0
    invalid_mask = d < 0
    
    if np.any(valid_mask):
        yj = (y1 - a[valid_mask]*b[valid_mask] - np.sqrt(d[valid_mask])) / (b[valid_mask]*b[valid_mask] + 1)  # point extérieur
        zj = a[valid_mask] + b[valid_mask]*yj
        theta[valid_mask] = np.degrees(np.arctan2(-zj, y1 - yj)) + np.where(yj > y1, 180.0, 0.0)
    
    # Pour les cas invalides, mettre NaN
    theta[invalid_mask] = np.nan
    
    return theta


def delta_calcInverse(f, e, rf, re, coordinates):
    """
    Cinématique inverse : coordinates -> angles
    Fonctionne avec arrays numpy
    
    Paramètres:
    -----------
    coordinates : array numpy de shape (n, 3)
        Chaque ligne contient les coordonnées [x, y, z] d'un point
    
    Retour:
    -------
    array numpy de shape (n, 3)
        Chaque ligne contient les angles [theta1, theta2, theta3]
    """
    # Convertir en array numpy si nécessaire
    coords = np.atleast_2d(np.array(coordinates, dtype=float))
    
    # Vérifier la forme
    if (coords.ndim != 2) or (coords.shape[1] != 3):
        return np.array([[np.nan, np.nan, np.nan]])
    
    x0 = coords[:, 0]
    y0 = coords[:, 1]
    z0 = coords[:, 2]
    
    # Bras 1 : pas de rotation
    theta1 = delta_calcAngleYZ(f, e, rf, re, x0, y0, z0)
    
    # Bras 2 : rotation de -120°
    x2 = x0*cos120 + y0*sin120
    y2 = y0*cos120 - x0*sin120
    theta2 = delta_calcAngleYZ(f, e, rf, re, x2, y2, z0)
    
    # Bras 3 : rotation de +120°
    x3 = x0*cos120 - y0*sin120
    y3 = y0*cos120 + x0*sin120
    theta3 = delta_calcAngleYZ(f, e, rf, re, x3, y3, z0)
    
    return np.column_stack((theta1, theta2, theta3))


def delta_calcForward(f, e, rf, re, angles):
    """
    Cinématique directe : angles -> coordinates
    Fonctionne avec arrays numpy
    
    Paramètres:
    -----------
    angles : array numpy de shape (n, 3)
        Chaque ligne contient les angles [theta1, theta2, theta3] en degrés
    
    Retour:
    -------
    array numpy de shape (n, 3)
        Chaque ligne contient les coordonnées [x0, y0, z0]
    """
    # Convertir en array numpy si nécessaire
    np_angles = np.atleast_2d(np.array(angles, dtype=float))
    
    # Vérifier la forme
    if (np_angles.ndim != 2) or (np_angles.shape[1] != 3):
        return np.array([[np.nan, np.nan, np.nan]])
    
    dtr = pi / 180.0  # degrés -> radians
    theta1_rad = np_angles[:, 0] * dtr
    theta2_rad = np_angles[:, 1] * dtr
    theta3_rad = np_angles[:, 2] * dtr

    t = (f - e) * tan30 / 2.0

    y1 = -(t + rf*np.cos(theta1_rad))
    z1 = -rf*np.sin(theta1_rad)

    y2 = (t + rf*np.cos(theta2_rad)) * sin30
    x2 = y2 * tan60
    z2 = -rf*np.sin(theta2_rad)

    y3 = (t + rf*np.cos(theta3_rad)) * sin30
    x3 = -y3 * tan60
    z3 = -rf*np.sin(theta3_rad)

    dnm = (y2 - y1)*x3 - (y3 - y1)*x2

    w1 = y1*y1 + z1*z1
    w2 = x2*x2 + y2*y2 + z2*z2
    w3 = x3*x3 + y3*y3 + z3*z3

    # x = (a1*z + b1) / dnm
    a1 = (z2 - z1)*(y3 - y1) - (z3 - z1)*(y2 - y1)
    b1 = -((w2 - w1)*(y3 - y1) - (w3 - w1)*(y2 - y1)) / 2.0

    # y = (a2*z + b2) / dnm
    a2 = -(z2 - z1)*x3 + (z3 - z1)*x2
    b2 =  ((w2 - w1)*x3  - (w3 - w1)*x2) / 2.0

    # a*z² + b*z + c = 0
    a = a1*a1 + a2*a2 + dnm*dnm
    b = 2.0*(a1*b1 + a2*(b2 - y1*dnm) - z1*dnm*dnm)
    c = (b2 - y1*dnm)*(b2 - y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re)

    # discriminant
    d = b*b - 4.0*a*c
    
    # Initialiser les arrays de sortie
    n = len(np_angles)
    x0 = np.zeros(n)
    y0 = np.zeros(n)
    z0 = np.zeros(n)
    
    # Masque pour les cas valides
    valid_mask = d >= 0
    invalid_mask = d < 0
    
    if np.any(valid_mask):
        idx = np.where(valid_mask)[0]
        
        # Solution avec le discriminant positif
        z0_pos = -0.5*(b[idx] + np.sqrt(d[idx])) / a[idx]
        z0_neg = -0.5*(b[idx] - np.sqrt(d[idx])) / a[idx]
        
        x0_pos = (a1[idx]*z0_pos + b1[idx]) / dnm[idx]
        y0_pos = (a2[idx]*z0_pos + b2[idx]) / dnm[idx]
        
        x0_neg = (a1[idx]*z0_neg + b1[idx]) / dnm[idx]
        y0_neg = (a2[idx]*z0_neg + b2[idx]) / dnm[idx]
        
        # Choisir la solution avec z négatif (plateforme en dessous des moteurs)
        mask_neg = z0_neg < 0
        z0[idx[mask_neg]] = z0_neg[mask_neg]
        x0[idx[mask_neg]] = x0_neg[mask_neg]
        y0[idx[mask_neg]] = y0_neg[mask_neg]
        
        z0[idx[~mask_neg]] = z0_pos[~mask_neg]
        x0[idx[~mask_neg]] = x0_pos[~mask_neg]
        y0[idx[~mask_neg]] = y0_pos[~mask_neg]
    
    # Pour les cas invalides, mettre NaN
    x0[invalid_mask] = np.nan
    y0[invalid_mask] = np.nan
    z0[invalid_mask] = np.nan
    
    return np.column_stack((x0, y0, z0))


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
    from mpl_toolkits.mplot3d import Axes3D

    # Paramètres du robot (correspondant à DeltaCoord2.py)
    e = 40.0   # Rayon de la plateforme (rP)
    f = 200.0  # Rayon de la base (wB)
    s =6*f/sqrt(3) # longueure du triangle de base (f*sqrt(3))
    u = sqrt(3)*s/3 # distance du centre de la base au sommet du triangle (f*sqrt(3)/3)
    re = 430.0 # Longueur des connecteurs (l)
    rf = 200.0 # Longueur des bras moteurs (L)

    # Calculer angles et position
    if position is not None:
        angles_calc = delta_calcInverse(f, e, rf, re, [position])
        if np.isnan(angles_calc[0, 0]):
            print("Position hors de portée du robot")
            return
        angles = angles_calc[0]
    elif angles is not None:
        position_calc = delta_calcForward(f, e, rf, re, [angles])
        if np.isnan(position_calc[0, 0]):
            print("Angles invalides")
            return
        position = position_calc[0]
    else:
        # Position par défaut
        position = [0, 0, -300]
        angles = delta_calcInverse(f, e, rf, re, [position])[0]

    x0, y0, z0 = position
    theta1, theta2, theta3 = angles

    # Créer la figure si nécessaire
    if ax is None:
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
    else:
        fig = ax.figure

    # Positions des moteurs (base fixe)
    # Dans DeltaCoord2.py, les moteurs sont positionnés différemment
    motor1 = (0, f, 0)  # Bras 1
    motor2 = (f*np.sin(np.radians(-120)), f*np.cos(np.radians(-120)), 0)  # Bras 2 (rotation -120°)
    motor3 = (f*np.sin(np.radians(120)), f*np.cos(np.radians(120)), 0)  # Bras 3 (rotation +120°)

    # Positions des extrémités des bras moteurs
    # Calcul basé sur delta_calcForward

    theta1_rad = np.radians(theta1)
    theta2_rad = np.radians(theta2)
    theta3_rad = np.radians(theta3)

    t = (f - e) * tan(np.radians(30)) / 2.0

    y1 = rf*np.cos(theta1_rad)
    z1 = -rf*np.sin(theta1_rad)
    arm1_end = (0, f + y1, z1)  # Bras 1

    y2 =  -rf*np.cos(theta2_rad) * sin(np.radians(30))
    x2 = y2 * tan(np.radians(60))
    z2 = -rf*np.sin(theta2_rad)
    arm2_end = (x2 + f*np.sin(np.radians(-120)), y2 + f*np.cos(np.radians(-120)), z2)  # Bras 2

    y3 =  -rf*np.cos(theta3_rad) * sin(np.radians(30))
    x3 = -y3 * tan(np.radians(60))
    z3 = -rf*np.sin(theta3_rad)
    arm3_end = (x3 + f*np.sin(np.radians(120)), y3 + f*np.cos(np.radians(120)), z3)  # Bras 3

    # Positions des points d'attache sur la plateforme outil
    platform1 = (x0, y0 + e, z0)  # Bras 1
    platform2 = (x0 + e*np.sin(np.radians(-120)), y0 + e*np.cos(np.radians(-120)), z0)  # Bras 2 (rotation -120°)
    platform3 = (x0 + e*np.sin(np.radians(120)), y0 + e*np.cos(np.radians(120)), z0)  # Bras 3 (rotation +120°)

    # Tracer la base fixe (triangle approximatif)
    # Calculer les positions des sommets du triangle de base
    base1 = (s/2, f , 0)
    base2 = (-s/2, f , 0)
    base3 = (0, -u, 0)
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
    lengtharm1 = np.sqrt((arm1_end[0]-motor1[0])**2 + (arm1_end[1]-motor1[1])**2 + (arm1_end[2]-motor1[2])**2)
    lengtharm2 = np.sqrt((arm2_end[0]-motor2[0])**2 + (arm2_end[1]-motor2[1])**2 + (arm2_end[2]-motor2[2])**2)
    lengtharm3 = np.sqrt((arm3_end[0]-motor3[0])**2 + (arm3_end[1]-motor3[1])**2 + (arm3_end[2]-motor3[2])**2)
    print(f"Longueurs des bras moteurs: Bras 1 = {lengtharm1:.2f} mm, Bras 2 = {lengtharm2:.2f} mm, Bras 3 = {lengtharm3:.2f} mm, longueur théorique = {rf:.2f} mm")
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

    # Calculer et afficher les longueurs des connecteurs
    length1 = np.sqrt((arm1_end[0]-platform1[0])**2 + (arm1_end[1]-platform1[1])**2 + (arm1_end[2]-platform1[2])**2)
    length2 = np.sqrt((arm2_end[0]-platform2[0])**2 + (arm2_end[1]-platform2[1])**2 + (arm2_end[2]-platform2[2])**2)
    length3 = np.sqrt((arm3_end[0]-platform3[0])**2 + (arm3_end[1]-platform3[1])**2 + (arm3_end[2]-platform3[2])**2)
    print(f"Longueurs des connecteurs: Connecteur 1 = {length1:.2f} mm, Connecteur 2 = {length2:.2f} mm, Connecteur 3 = {length3:.2f} mm, longueur théorique = {re:.2f} mm")

    # Tracer la plateforme outil (triangle)
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
    ax.set_title('.1f'
                 '.1f')

    # Ajuster les limites pour une meilleure visualisation
    all_points = np.vstack([base_points, platform_points])
    max_range = max(np.ptp(all_points[:, 0]), np.ptp(all_points[:, 1]), np.ptp(all_points[:, 2]))

    ax.set_xlim([np.min(all_points[:, 0]) - 50, np.max(all_points[:, 0]) + 50])
    ax.set_ylim([np.min(all_points[:, 1]) - 50, np.max(all_points[:, 1]) + 50])
    ax.set_zlim([np.min(all_points[:, 2]) - 50, np.max(all_points[:, 2]) + 50])

    ax.legend()

    if show_plot:
        plt.show()


# --- Tests ---
if __name__ == "__main__":
    # Dimensions du robot (valeurs par défaut du site)
    e  = 40.0   # rayon end effector
    f  = 200.0   # rayon base
    re = 430.0   # longueur avant-bras
    rf = 200.0   # longueur bicep

    print("=== Test cinématique inverse ===")
    # Test avec un point unique
    result = delta_calcInverse(f, e, rf, re, [[0, 0, -300]])
    print(f"Position (0, 0, -300) -> theta1={result[0,0]:.3f}°  theta2={result[0,1]:.3f}°  theta3={result[0,2]:.3f}°")
    
    result = delta_calcInverse(f, e, rf, re, [[250, 250, -200]])
    print(f"Position (250, 250, -200) -> theta1={result[0,0]:.3f}°  theta2={result[0,1]:.3f}°  theta3={result[0,2]:.3f}°")
    
    result = delta_calcInverse(f, e, rf, re, [[0, 0, -342.5]])
    print(f"Position (0, 0, -342.5) -> theta1={result[0,0]:.3f}°  theta2={result[0,1]:.3f}°  theta3={result[0,2]:.3f}°")

    # Test avec plusieurs points
    test_coords = np.array([
        [0, 0, -300],
        [250, 250, -200],
        [0, 0, -342.5],
        [10, 20, -250]
    ])
    print("\nInverse (multiple points):")
    results = delta_calcInverse(f, e, rf, re, test_coords)
    for i, (coord, angles) in enumerate(zip(test_coords, results)):
        print(f"Point {i+1}: {coord} -> {angles}")

    print("\n=== Test cinématique directe ===")
    # Test avec un set d'angles unique
    result = delta_calcForward(f, e, rf, re, [[0, 0, 0]])
    print(f"Angles (0, 0, 0) -> x={result[0,0]:.3f}  y={result[0,1]:.3f}  z={result[0,2]:.3f}")
    
    result = delta_calcForward(f, e, rf, re, [[-13.5, -13.5, -13.5]])
    print(f"Angles (-13.5, -13.5, -13.5) -> x={result[0,0]:.3f}  y={result[0,1]:.3f}  z={result[0,2]:.3f}")
    
    result = delta_calcForward(f, e, rf, re, [[78.5, -61.8, 57.4]])
    print(f"Angles (78.5, -61.8, 57.4) -> x={result[0,0]:.3f}  y={result[0,1]:.3f}  z={result[0,2]:.3f}")

    # Test avec plusieurs sets d'angles
    test_angles = np.array([
        [0, 0, 0],
        [-13.5, -13.5, -13.5],
        [78.5, -61.8, 57.4]
    ])
    print("\nForward (multiple angles):")
    results = delta_calcForward(f, e, rf, re, test_angles)
    for i, (angles, coord) in enumerate(zip(test_angles, results)):
        print(f"Angles {i+1}: {angles} -> {coord}")

    print("\n=== Vérification aller-retour ===")
    # Test aller-retour
    original_coords = np.array([[10, 20, -250]])
    angles = delta_calcInverse(f, e, rf, re, original_coords)
    print(f"IK: {original_coords[0]} -> {angles[0]}")
    coords_back = delta_calcForward(f, e, rf, re, angles)
    print(f"FK: {angles[0]} -> {coords_back[0]}  (attendu: {original_coords[0]})")

    print("\n=== Test visualisation ===")
    # Test de la visualisation avec des angles
    visualisation(position=[10, 20, -250])

    print("\n=== Exemples d'utilisation de la visualisation ===")
    print("Pour visualiser avec des angles moteurs:")
    print("visualisation(angles=[theta1, theta2, theta3])")
    print("Exemple: visualisation(angles=[78.5, -61.8, 57.4])")
    print()
    print("Pour visualiser avec une position de l'effecteur:")
    print("visualisation(position=[x, y, z])")
    print("Exemple: visualisation(position=[250, 250, -200])")
    print()
    print("Pour la position par défaut:")
    print("visualisation()")