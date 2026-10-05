from email import generator
from math import *
import numpy as np

# Configuration affichage numpy : supprimer la notation scientifique
np.set_printoptions(suppress=True, precision=9, linewidth=200)

def rotate(x, y, angle_deg):
    """Rotation compatible avec arrays numpy et scalaires"""
    a = np.radians(angle_deg)
    return x*np.cos(a) - y*np.sin(a),  x*np.sin(a) + y*np.cos(a)

def solve_one_arm(xr, yr, z, wB, rP, L, l):
    """Résout pour un bras dont la direction est l'axe Y local
    Fonctionne avec des scalaires et arrays numpy"""
    a = wB - rP
    E = 2*L*(yr + a)
    F = 2*L*z
    G = xr**2 + yr**2 + z**2 + a**2 + L**2 + 2*a*yr - l**2
    disc = E**2 + F**2 - G**2
    
    mask_disc_neg = disc < 0
    mask_disc_pos = ~mask_disc_neg
    t_minus = np.zeros_like(xr)
    t_minus[mask_disc_neg] =  np.nan  
    t_minus[mask_disc_pos] = (-F[mask_disc_pos] - np.sqrt(disc[mask_disc_pos])) / (G[mask_disc_pos] - E[mask_disc_pos])
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
        Chaque ligne contient les angles [t1, t2, t3] des trois moteurs
    """
    # Convertir en array numpy si nécessaire
    coords = np.atleast_2d(np.array(coordinates, dtype=float))
    
    # Vérifier la forme
    if (coords.ndim != 2) | (coords.shape[1] != 3):
        return np.array([[np.nan, np.nan, np.nan]])
    else:
    
        # Paramètres du robot
        rB = 200    # Rayon du cercle sur lequel sont les moteurs
        wB = rB/2               # Rayon de la base du triangle équilatéral formé par les moteurs
        rP = 40                 # Rayon de la plateforme (distance du centre à un point d'attache)
        L = 200                 # Longueur des bras moteurs
        l = 430                 # Longueur des bras parallèles
        
        x = coords[:, 0]
        y = coords[:, 1]
        z = coords[:, 2]
        
        # Bras 1 : pas de rotation
        x1, y1 = rotate(x, y, 0)
        t1 = solve_one_arm(x1, y1, z, wB, rP, L, l)
        
        # Bras 2 : rotation de -120°
        x2, y2 = rotate(x, y, -120)
        t2 = solve_one_arm(x2, y2, z, wB, rP, L, l)
        
        # Bras 3 : rotation de +120°
        x3, y3 = rotate(x, y, 120)
        t3 = solve_one_arm(x3, y3, z, wB, rP, L, l)
        
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
    """
    np_angles = np.atleast_2d(np.array(angles, dtype=float))

    # Vérifier la forme
    if (np_angles.ndim != 2) | (np_angles.shape[1] != 3):
        return np.array([[np.nan, np.nan, np.nan]])
    
    
    else:
        rP = 40              # Rayon de la plateforme (distance du centre à un point d'attache)
        rB = 200              # Rayon du cercle sur lequel sont les moteurs
        wP = rP/2               # Rayon de la base du triangle équilatéral formé par les points d'attache sur la plateforme
        wB = rB/2               # Rayon de la base du triangle équilatéral formé par les moteurs
        sP = rP*3/sqrt(3)       # Rayon du cercle sur lequel sont les points d'attache sur la plateforme
        sB = rB*3/sqrt(3)       # Rayon du cercle sur lequel sont les moteurs
        L = 200                 # Longueur des bras moteurs
        l = 430                 # Longueur des bras parallèles (entre les points d'attache sur la plateforme et les extrémités des bras moteurs)
        t1_rad = np.radians(np_angles[:, 0])
        t2_rad = np.radians(np_angles[:, 1])
        t3_rad = np.radians(np_angles[:, 2])
        
        A1 = np.column_stack((np.zeros_like(t1_rad), -wB - L*np.cos(t1_rad)+rP, -L*np.sin(t1_rad)))
        A2 = np.column_stack(((np.sqrt(3)/2)*(wB + L*np.cos(t2_rad))-sP/2, (1/2)*(wB + L*np.cos(t2_rad))-wP, -L*np.sin(t2_rad)))
        A3 = np.column_stack((-(np.sqrt(3)/2)*(wB + L*np.cos(t3_rad))+sP/2, (1/2)*(wB + L*np.cos(t3_rad))-wP, -L*np.sin(t3_rad)))
        
        a11 = 2*(A3[:,0] - A1[:,0])
        a12 = 2*(A3[:,1] - A1[:,1])
        a13 = 2*(A3[:,2] - A1[:,2])
        a21 = 2*(A3[:,0] - A2[:,0])
        a22 = 2*(A3[:,1] - A2[:,1])
        a23 = 2*(A3[:,2] - A2[:,2])
        
        b1 = l**2 - l**2 - A1[:,0]**2 - A1[:,1]**2 - A1[:,2]**2 + A3[:,0]**2 + A3[:,1]**2 + A3[:,2]**2
        b2 = l**2 - l**2 - A2[:,0]**2 - A2[:,1]**2 - A2[:,2]**2 + A3[:,0]**2 + A3[:,1]**2 + A3[:,2]**2
        
        # Créer des masques pour cas singulier et normal
        a1_test = np.where((np.abs(a13) > 1e-10) & (np.abs(a23) > 1e-10),a11/np.where(np.abs(a13)>1e-10, a13, 1) - a21/np.where(np.abs(a23)>1e-10, a23, 1),np.zeros_like(a13))
        singular_mask = (np.abs(a13) < 1e-10) | (np.abs(a23) < 1e-10) | (np.abs(a1_test) < 1e-10)
        normal_mask = ~singular_mask
        
        n = len(np_angles)
        x = np.full(n, np.nan)
        y = np.full(n, np.nan)
        z = np.full(n, np.nan)
        
        # Traiter cas singulier
        if np.any(singular_mask):
            idx = np.where(singular_mask)[0]
            
            aS = a11[idx]
            bS = a12[idx]
            cS = l**2 - l**2 - A1[idx,0]**2 - A1[idx,1]**2 + A3[idx,0]**2 + A3[idx,1]**2
            dS = a21[idx]
            eS = a22[idx]
            fS = l**2 - l**2 - A2[idx,0]**2 - A2[idx,1]**2 + A3[idx,0]**2 + A3[idx,1]**2
            
            x[idx] = (cS*eS - fS*bS) / (aS*eS - dS*bS)
            y[idx] = (aS*fS - dS*cS) / (aS*eS - dS*bS)
            
            B = -2*A1[idx,2]
            C = A1[idx,2]**2 - l**2 + (x[idx]-A1[idx,0])**2 + (y[idx]-A1[idx,1])**2
            disc_s = B**2 - 4*C
            valid_s = disc_s >= 0
            zplus  = np.where(valid_s, (-B + np.sqrt(np.maximum(disc_s, 0))) / 2, np.nan)
            zminus = np.where(valid_s, (-B - np.sqrt(np.maximum(disc_s, 0))) / 2, np.nan)

            
            # Choisir zplus si négatif, sinon zminus
            mask_zplus = zplus < 0
            z[idx[mask_zplus]] = zplus[mask_zplus]
            z[idx[~mask_zplus]] = zminus[~mask_zplus]
        
        # Traiter cas normal
        if np.any(normal_mask):
            idx = np.where(normal_mask)[0]
            
            a1 = a11[idx]/a13[idx] - a21[idx]/a23[idx]
            a2 = a12[idx]/a13[idx] - a22[idx]/a23[idx]
            a3 = b2[idx]/a23[idx] - b1[idx]/a13[idx]
            a4 = -a2/a1
            a5 = -a3/a1
            a6 = (-a21[idx]*a4 - a22[idx])/a23[idx]
            a7 = (b2[idx] - a21[idx]*a5)/a23[idx]
            
            aa = a4**2 + 1 + a6**2
            bb = 2*a4*(a5 - A1[idx,0]) - 2*A1[idx,1] + 2*a6*(a7 - A1[idx,2])
            cc = a5*(a5 - 2*A1[idx,0]) + a7*(a7 - 2*A1[idx,2]) + A1[idx,0]**2 + A1[idx,1]**2 + A1[idx,2]**2 - l**2
            
            yplus = (-bb + np.sqrt(bb**2 - 4*aa*cc)) / (2*aa)
            yminus = (-bb - np.sqrt(bb**2 - 4*aa*cc)) / (2*aa)
            xplus = a4*yplus + a5
            zplus = a6*yplus + a7
            xminus = a4*yminus + a5
            zminus = a6*yminus + a7
            
            # Choisir la solution avec z négatif (plateforme en dessous des moteurs)
            mask_zplus = zplus < 0
            x[idx[mask_zplus]] = xplus[mask_zplus]
            y[idx[mask_zplus]] = yplus[mask_zplus]
            z[idx[mask_zplus]] = zplus[mask_zplus]
            
            x[idx[~mask_zplus]] = xminus[~mask_zplus]
            y[idx[~mask_zplus]] = yminus[~mask_zplus]
            z[idx[~mask_zplus]] = zminus[~mask_zplus]
        
        return np.column_stack((x, y, z))

def test_forward_inverse():
    """Valide les cas inverse forward simples ainsi que les edges cases en printant les résultats obtenus
    
    Parramètres:
    -----------
    Void
    
    Retour:
    -----------
    Void

    """
    
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
    print(DeltaInverse(DeltaForward(test_angles)))
    print("Forward (erreur format):", DeltaInverse([[0, 0]]))
    

    visualisation(angles= test_angles[0])
    
    visualisation(position = DeltaForward(test_angles[0]))


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

    # Paramètres du robot
    rB = 200    # Rayon du cercle sur lequel sont les moteurs
    wB = rB/2   # Rayon de la base du triangle équilatéral formé par les moteurs
    rP = 40     # Rayon de la plateforme (distance du centre à un point d'attache)
    wP = rP/2   # Rayon de la base du triangle équilatéral formé par les points d'attache
    sP = rP*3/sqrt(3)  # Rayon du cercle sur lequel sont les points d'attache sur la plateforme
    sB = rB*3/sqrt(3)  # Rayon du cercle sur lequel sont les moteurs
    L = 200     # Longueur des bras moteurs
    l = 430     # Longueur des bras parallèles

    # Calculer angles et position
    if position is not None:
        angles_calc = DeltaInverse([position])[0]
        print(f"Angles calculés pour position {position}: θ1={angles_calc[0]:.2f}°, θ2={angles_calc[1]:.2f}°, θ3={angles_calc[2]:.2f}°")
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
    motor1 = (0, -wB, 0)
    motor2 = ( wB*sqrt(3)/2,  wB/2, 0)   # rotate((0,-wB,0), +120°)
    motor3 = (-wB*sqrt(3)/2,  wB/2, 0)   # rotate((0,-wB,0), +240°)

    base1 = (sB/2,  wB, 0)
    base2 = (-sB/2, wB, 0)
    base3 = (0, -rB, 0)

    # Positions des extrémités des bras moteurs
    # Pour le bras i, le coude est à : motor_i + L*(0, -cos(θ_i), -sin(θ_i)) dans le repère local,
    # ce qui donne après rotation par +k*120° :
    arm1_end = (0,
                -wB - L*np.cos(np.radians(theta1)),
                -L*np.sin(np.radians(theta1)))
    arm2_end = ( (wB + L*np.cos(np.radians(theta2)))*sqrt(3)/2,
                 (wB + L*np.cos(np.radians(theta2)))/2,
                -L*np.sin(np.radians(theta2)))
    arm3_end = (-(wB + L*np.cos(np.radians(theta3)))*sqrt(3)/2,
                 (wB + L*np.cos(np.radians(theta3)))/2,
                -L*np.sin(np.radians(theta3)))

    # Positions des points d'attache sur la plateforme outil
    # Le point d'attache du bras 1 est décalé de -rP selon Y (même convention que -Y pour le bras 1).
    platform1 = (x0,               y0 - rP,      z0)
    platform2 = (x0 + rP*sqrt(3)/2, y0 + rP/2,  z0)   # rotate((0,-rP), +120°) + centre
    platform3 = (x0 - rP*sqrt(3)/2, y0 + rP/2,  z0)   # rotate((0,-rP), +240°) + centre

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
    ax.set_title('.1f'
                 '.1f')

    # Ajuster les limites pour une meilleure visualisation
    all_points = np.vstack([base_points, platform_points])
    max_range = max(np.ptp(all_points[:, 0]), np.ptp(all_points[:, 1]), np.ptp(all_points[:, 2]))

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
    print(f"Angles des moteurs: θ1={theta1:.2f}°, θ2={theta2:.2f}°, θ3={theta3:.2f}°")

    return fig, ax


# Seuil angulaire physique : angles en dehors de cette plage = elbow down ou singularité
ANGLE_MIN_DEG = -90.0
ANGLE_MAX_DEG =  90.0
# Seuil de cohérence : si un pas produit un déplacement supérieur à cette valeur,
# c'est un saut de branche (elbow up -> elbow down), pas une vraie résolution.
# Valeur max théorique : L * sin(pas_en_radians) ≈ 200 * sin(0.9°) ≈ 3.1 mm
# On prend 20 mm comme marge très confortable.
COHERENCE_THRESHOLD_MM = 20.0


def _angles_are_physical(angles):
    """
    Retourne un masque booléen : True si les angles sont dans la plage physique.
    Filtre les configurations elbow-down et les singularités.

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


def resolution_xyz(pas_par_tour, chunk_size=4096):
    """Détermine la résolution linéaire pour tous les points XYZ dans la zone de travail.

    Pour chaque point valide, on calcule la position nominale via DeltaForward, puis on
    perturbe chaque moteur d'un demi-pas (±) et on mesure le déplacement cartésien résultant.
    La résolution maximale sur l'ensemble du workspace est retournée.

    Corrections appliquées par rapport à la version précédente :
    - Filtre angulaire : seuls les angles dans [-90°, +90°] sont acceptés (elbow-up).
      Les angles hors plage indiquent une configuration elbow-down ou une singularité.
    - Filtre de cohérence : les déplacements > COHERENCE_THRESHOLD_MM sont ignorés.
      Ils indiquent un saut de branche (elbow-up → elbow-down) dû à la perturbation,
      et non une vraie résolution linéaire.

    Args:
        pas_par_tour (int): nombre de pas par tour des moteurs
        chunk_size (int): nombre de points traités par chunk

    Returns:
        resolutions (np.array): résolutions (mm) pour chaque point valide
        max_resolution (float): résolution maximale trouvée (mm/pas)
        max_resolution_position (tuple): position (x, y, z) du pire cas
    """
    pas_par_tour = int(pas_par_tour)
    if pas_par_tour <= 0:
        return np.array([], dtype=np.float32), np.nan, (np.nan, np.nan, np.nan)

    half_step_deg = (360.0 / pas_par_tour) / 2.0

    sign_combinations = np.array([
        [ 1.0,  1.0,  1.0],
        [ 1.0,  1.0, -1.0],
        [ 1.0, -1.0,  1.0],
        [ 1.0, -1.0, -1.0],
        [-1.0,  1.0,  1.0],
        [-1.0,  1.0, -1.0],
        [-1.0, -1.0,  1.0],
        [-1.0, -1.0, -1.0],
    ], dtype=float)  # shape (8, 3)

    x_values = np.arange(-250, 251, 1, dtype=float)
    y_values = np.arange(-250, 251, 1, dtype=float)
    z_values = np.arange(-450, -149, 1, dtype=float)

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

            # --- CORRECTION 1 : filtre NaN + filtre angulaire physique ---
            # Un angle hors [-90°, 90°] signifie elbow-down : on l'exclut.
            nan_mask      = ~np.isnan(angles).any(axis=1)
            physical_mask = _angles_are_physical(angles)
            valid_mask    = nan_mask & physical_mask

            if not np.any(valid_mask):
                continue

            valid_angles      = angles[valid_mask]           # (k, 3)
            nominal_positions = DeltaForward(valid_angles)   # (k, 3)

            # Positions perturbées pour les 8 combinaisons de signes : (8, k, 3)
            perturbed_angles = (
                valid_angles[np.newaxis, :, :]               # (1, k, 3)
                + sign_combinations[:, np.newaxis, :]        # (8, 1, 3)
                * half_step_deg
            )  # → (8, k, 3)

            # Vérifier que les angles perturbés restent eux aussi dans la plage physique
            perturbed_physical = np.all(
                (perturbed_angles >= ANGLE_MIN_DEG) & (perturbed_angles <= ANGLE_MAX_DEG),
                axis=2
            )  # (8, k)

            # Calculer toutes les positions perturbées en un seul appel DeltaForward
            perturbed_flat    = perturbed_angles.reshape(-1, 3)          # (8k, 3)
            positions_flat    = DeltaForward(perturbed_flat)             # (8k, 3)
            candidate_pos     = positions_flat.reshape(8, -1, 3)         # (8, k, 3)

            # Déplacements cartésiens : (8, k)
            diff         = candidate_pos - nominal_positions[np.newaxis, :, :]
            displacements = np.linalg.norm(diff, axis=2)                 # (8, k)

            # --- CORRECTION 2 : masquer les sauts de branche ---
            # Si les angles perturbés sont hors plage physique OU si le déplacement
            # dépasse le seuil de cohérence → c'est un artefact, pas une résolution.
            invalid_perturbation = (
                ~perturbed_physical                          # hors plage angulaire
                | np.isnan(displacements)                   # NaN dans DeltaForward
                | (displacements > COHERENCE_THRESHOLD_MM)  # saut de branche
            )
            displacements = np.where(invalid_perturbation, np.nan, displacements)

            # Résolution = pire déplacement parmi les 8 perturbations pour chaque point
            max_displacements = np.nanmax(displacements, axis=0)         # (k,)

            # Ignorer les points où toutes les perturbations sont invalides
            all_invalid = np.all(invalid_perturbation, axis=0)           # (k,)
            max_displacements[all_invalid] = np.nan

            valid_results = max_displacements[~np.isnan(max_displacements)]
            if valid_results.size:
                local_best_idx   = int(np.argmax(valid_results))
                local_best_value = float(valid_results[local_best_idx])

                if local_best_value > max_resolution:
                    max_resolution = local_best_value
                    # Retrouver l'indice absolu dans coords_xy
                    valid_indices  = np.flatnonzero(valid_mask)
                    non_nan_valid  = valid_indices[~np.isnan(max_displacements)]
                    absolute_idx   = non_nan_valid[local_best_idx]
                    max_resolution_position = (
                        float(coords_xy[start + absolute_idx - start, 0])
                            if absolute_idx < chunk_size
                            else float(coords_xy[absolute_idx, 0]),
                        float(coords_xy[min(absolute_idx, n_xy-1), 1]),
                        float(z)
                    )
                    # Version simplifiée et correcte de la position
                    orig_idx = start + np.flatnonzero(valid_mask)[
                        np.flatnonzero(~np.isnan(max_displacements))[local_best_idx]
                    ]
                    max_resolution_position = (
                        float(coords_xy[orig_idx, 0]),
                        float(coords_xy[orig_idx, 1]),
                        float(z)
                    )

            resolutions_chunks.append(
                max_displacements[~np.isnan(max_displacements)].astype(np.float32)
            )

    resolutions = np.concatenate(resolutions_chunks) if resolutions_chunks \
                  else np.array([], dtype=np.float32)

    return resolutions, float(max_resolution), max_resolution_position


def visualisation_resolution(pas_par_tour, z_slice=None, chunk_size=4096):
    """
    Visualise la carte de résolution linéaire dans le plan XY pour une tranche Z donnée.
    Permet d'identifier visuellement où les valeurs aberrantes apparaissent.

    Paramètres:
    -----------
    pas_par_tour : int
        Nombre de pas par tour des moteurs
    z_slice : float, optionnel
        Hauteur Z à visualiser. Si None, utilise Z = -300 mm (milieu du workspace).
    chunk_size : int
        Taille des chunks de calcul

    Retour:
    -------
    fig : matplotlib Figure
    """
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors

    if z_slice is None:
        z_slice = -300.0

    pas_par_tour   = int(pas_par_tour)
    half_step_deg  = (360.0 / pas_par_tour) / 2.0

    sign_combinations = np.array([
        [ 1.0,  1.0,  1.0], [ 1.0,  1.0, -1.0],
        [ 1.0, -1.0,  1.0], [ 1.0, -1.0, -1.0],
        [-1.0,  1.0,  1.0], [-1.0,  1.0, -1.0],
        [-1.0, -1.0,  1.0], [-1.0, -1.0, -1.0],
    ], dtype=float)

    x_values = np.arange(-250, 251, 2, dtype=float)   # pas de 2mm pour la visu
    y_values = np.arange(-250, 251, 2, dtype=float)
    xx, yy   = np.meshgrid(x_values, y_values, indexing='xy')
    coords_xy = np.column_stack((xx.ravel(), yy.ravel()))
    n_xy      = coords_xy.shape[0]

    resolution_map  = np.full(n_xy, np.nan)
    angle_map       = np.full((n_xy, 3), np.nan)   # pour diagnostiquer

    z_col   = np.full(n_xy, z_slice)
    coords  = np.column_stack((coords_xy, z_col))
    angles  = DeltaInverse(coords)

    nan_mask      = ~np.isnan(angles).any(axis=1)
    physical_mask = _angles_are_physical(angles)
    valid_mask    = nan_mask & physical_mask

    angle_map[valid_mask] = angles[valid_mask]

    if np.any(valid_mask):
        valid_angles      = angles[valid_mask]
        nominal_positions = DeltaForward(valid_angles)

        perturbed_angles = (
            valid_angles[np.newaxis, :, :]
            + sign_combinations[:, np.newaxis, :]
            * half_step_deg
        )
        perturbed_physical = np.all(
            (perturbed_angles >= ANGLE_MIN_DEG) & (perturbed_angles <= ANGLE_MAX_DEG),
            axis=2
        )

        perturbed_flat  = perturbed_angles.reshape(-1, 3)
        positions_flat  = DeltaForward(perturbed_flat)
        candidate_pos   = positions_flat.reshape(8, -1, 3)

        diff          = candidate_pos - nominal_positions[np.newaxis, :, :]
        displacements = np.linalg.norm(diff, axis=2)

        invalid = (
            ~perturbed_physical
            | np.isnan(displacements)
            | (displacements > COHERENCE_THRESHOLD_MM)
        )
        displacements = np.where(invalid, np.nan, displacements)

        max_disp = np.nanmax(displacements, axis=0)
        all_inv  = np.all(invalid, axis=0)
        max_disp[all_inv] = np.nan

        resolution_map[valid_mask] = max_disp

    # ---- Affichage ----
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle(f'Analyse résolution — Z = {z_slice:.0f} mm  |  {pas_par_tour} pas/tour',
                 fontsize=14, fontweight='bold')

    res_2d = resolution_map.reshape(len(y_values), len(x_values))

    # --- Subplot 1 : carte de résolution normale ---
    ax1 = axes[0]
    vmax_normal = np.nanpercentile(res_2d, 95)   # 95e percentile pour ne pas écraser les valeurs normales
    im1 = ax1.imshow(
        res_2d, origin='lower',
        extent=[x_values[0], x_values[-1], y_values[0], y_values[-1]],
        cmap='viridis', vmin=0, vmax=vmax_normal
    )
    plt.colorbar(im1, ax=ax1, label='Résolution (mm/pas)')
    ax1.set_title('Résolution (échelle normale\njusqu\'au 95e percentile)')
    ax1.set_xlabel('X (mm)'); ax1.set_ylabel('Y (mm)')

    # --- Subplot 2 : carte de résolution — valeurs aberrantes en rouge ---
    ax2 = axes[1]
    res_display = res_2d.copy()
    aberrant_mask_2d = res_2d > vmax_normal
    cmap_custom = plt.cm.viridis.copy()
    cmap_custom.set_bad('lightgray')
    im2 = ax2.imshow(
        np.where(~aberrant_mask_2d, res_display, np.nan),
        origin='lower',
        extent=[x_values[0], x_values[-1], y_values[0], y_values[-1]],
        cmap='viridis', vmin=0, vmax=vmax_normal
    )
    # Superposer les valeurs aberrantes en rouge
    ax2.imshow(
        np.where(aberrant_mask_2d, 1.0, np.nan),
        origin='lower',
        extent=[x_values[0], x_values[-1], y_values[0], y_values[-1]],
        cmap='Reds', vmin=0, vmax=1, alpha=0.8
    )
    ax2.imshow(
        np.where(np.isnan(res_2d), 1.0, np.nan),
        origin='lower',
        extent=[x_values[0], x_values[-1], y_values[0], y_values[-1]],
        cmap='Greys', vmin=0, vmax=1, alpha=0.4
    )
    plt.colorbar(im2, ax=ax2, label='Résolution (mm/pas)')
    ax2.set_title(f'Rouge = valeurs > {vmax_normal:.2f} mm\n(hors workspace ou saut branche)')
    ax2.set_xlabel('X (mm)'); ax2.set_ylabel('Y (mm)')

    # --- Subplot 3 : angle t1 pour comprendre les zones problématiques ---
    ax3 = axes[2]
    t1_map = angle_map[:, 0].reshape(len(y_values), len(x_values))
    # Marquer les angles hors plage physique (avant filtrage) en rouge
    angles_raw     = DeltaInverse(np.column_stack((coords_xy, z_col)))
    t1_raw         = angles_raw[:, 0].reshape(len(y_values), len(x_values))
    out_of_range_2d = (t1_raw < ANGLE_MIN_DEG) | (t1_raw > ANGLE_MAX_DEG)

    im3 = ax3.imshow(
        t1_map, origin='lower',
        extent=[x_values[0], x_values[-1], y_values[0], y_values[-1]],
        cmap='RdYlGn', vmin=ANGLE_MIN_DEG, vmax=ANGLE_MAX_DEG
    )
    ax3.imshow(
        np.where(out_of_range_2d, 1.0, np.nan),
        origin='lower',
        extent=[x_values[0], x_values[-1], y_values[0], y_values[-1]],
        cmap='Reds', vmin=0, vmax=1, alpha=0.6
    )
    plt.colorbar(im3, ax=ax3, label='θ1 (degrés)')
    ax3.set_title(f'Angle θ1 — rouge = hors [{ANGLE_MIN_DEG:.0f}°, {ANGLE_MAX_DEG:.0f}°]\n(elbow-down filtré)')
    ax3.set_xlabel('X (mm)'); ax3.set_ylabel('Y (mm)')

    # Stats
    valid_res = res_2d[~np.isnan(res_2d) & ~aberrant_mask_2d]
    n_total   = np.sum(~np.isnan(res_2d))
    n_aberrant = np.sum(aberrant_mask_2d & ~np.isnan(res_2d))
    print(f"\n{'='*55}")
    print(f"  Résolution à Z = {z_slice:.0f} mm  |  {pas_par_tour} pas/tour")
    print(f"{'='*55}")
    print(f"  Points valides        : {n_total}")
    print(f"  Valeurs > 95e pct     : {n_aberrant}  ({100*n_aberrant/max(n_total,1):.1f}%)")
    if valid_res.size:
        print(f"  Résolution min        : {np.min(valid_res):.3f} mm")
        print(f"  Résolution médiane    : {np.median(valid_res):.3f} mm")
        print(f"  Résolution max (filtré): {np.max(valid_res):.3f} mm")
        print(f"  95e percentile        : {vmax_normal:.3f} mm")
    print(f"{'='*55}\n")

    plt.tight_layout()
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
    # visualisation_resolution(pas_par_tour=650, z_slice=-300.0)
