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
        (solution avec la plateforme sous les moteurs). Une ligne vaut NaN si les
        angles ne correspondent à aucune position (les barres ne peuvent pas se
        fermer sur la plateforme).
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
        
        # Les trois sphères de rayon l centrées sur A1, A2 et A3 se coupent en deux
        # points, symétriques par rapport au plan (A1, A2, A3). On les obtient par
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
            zl = np.sqrt(l**2 - xl**2 - yl**2)                 # NaN si pas d'intersection

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

def resolution_xyz(pas_par_tour, chunk_size=4096):
    """determine la resolution pour tout les xyz dans la zone de travail du robot en comparant la position nominale à la position après un pas sur chaque moteur, et retourne la résolution maximale trouvée.
        Les positions evaluées sont determinées à l'aide de la fonction DeltaInversepour tout les points alaant de -250 à 250 en x et y et de -450 à -150 en z avec un pas de 1mm, ce qui couvre la zone de travail typique du robot Delta.
        La fonction va ensuite renvoyer une liste de resolutions pour chaque point évalué lorsque chaque moteur est excentré d'un demi pas en + ou en - (pour simuler une precision de n degrées par pas).
        La fonction va aussi determnier la resolution maximale pour toute les positions du plan de travail et renvoyer la position avec le plus grand decalage.
    Args:
        pas_par_tour (int): pas par tour des moteurs
        chunk_size (int): nombre de points traités par chunk pour réduire l'utilisation mémoire
    Returns:
        resolutions (np.array): array de shape (n) contenant les differences de position pour chaque point évalué
        max_resolution (float): la resolution maximale trouvée
        max_resolution_position (tuple): la position (x, y, z) correspondant à la resolution maximale
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
    ], dtype=float)

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
            valid_mask = ~np.isnan(angles).any(axis=1)
            if not np.any(valid_mask):
                continue

            valid_angles = angles[valid_mask]
            nominal_positions = DeltaForward(valid_angles)

            candidate_positions = np.stack([
                DeltaForward(valid_angles + sign * half_step_deg)
                for sign in sign_combinations
            ], axis=1)

            displacements = np.linalg.norm(candidate_positions - nominal_positions[:, np.newaxis, :], axis=2)
            max_displacements = np.nanmax(displacements, axis=1)

            if max_displacements.size:
                best_idx = int(np.nanargmax(max_displacements))
                best_value = float(max_displacements[best_idx])
                if best_value > max_resolution:
                    max_resolution = best_value
                    absolute_idx = start + np.flatnonzero(valid_mask)[best_idx]
                    max_resolution_position = (
                        coords_xy[absolute_idx, 0],
                        coords_xy[absolute_idx, 1],
                        float(z)
                    )

            resolutions_chunks.append(max_displacements.astype(np.float32))

    if resolutions_chunks:
        resolutions = np.concatenate(resolutions_chunks)
    else:
        resolutions = np.array([], dtype=np.float32)

    return resolutions, float(max_resolution), max_resolution_position
if __name__ == "__main__":
    res, max_res, max_res_pos = resolution_xyz(1000, chunk_size=4096)
    print(f"Résolution maximale trouvée: {max_res:.4f} mm au point (x={max_res_pos[0]:.2f}, y={max_res_pos[1]:.2f}, z={max_res_pos[2]:.2f})")
