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
        singular_mask = (np.abs(a13) < 1e-10) | (np.abs(a23) < 1e-10)
        normal_mask = ~singular_mask
        
        n = len(np_angles)
        x = np.zeros(n)
        y = np.zeros(n)
        z = np.zeros(n)
        
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
            zplus = (-B + np.sqrt(B**2 - 4*C)) / 2
            zminus = (-B - np.sqrt(B**2 - 4*C)) / 2
            
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
    motor1 = (0, wB, 0)  # Bras 1
    motor2 = (wB*sin(np.radians(-120)), wB*cos(np.radians(-120)), 0)  # Bras 2 (rotation -120°)
    motor3 = (wB*sin(np.radians(120)), wB*cos(np.radians(120)), 0)  # Bras 3 (rotation +120°)
    
    base1 = (sB/2,wB,0)
    base2 = (-sB/2,wB,0)
    base3 = (0,-rB,0)

    # Positions des extrémités des bras moteurs
    arm1_end = (0, wB + L*np.cos(np.radians(-theta1)), -L*np.sin(np.radians(-theta1)))
    arm2_end = (-(np.sqrt(3)/2)*(wB + L*np.cos(np.radians(-theta2))),
                -(1/2)*(wB + L*np.cos(np.radians(-theta2))),
                -L*np.sin(np.radians(-theta2)))
    arm3_end = ((np.sqrt(3)/2)*(wB + L*np.cos(np.radians(-theta3))),
                -(1/2)*(wB + L*np.cos(np.radians(-theta3))),
                -L*np.sin(np.radians(-theta3)))

    # Positions des points d'attache sur la plateforme outil
    platform1 = (x0, y0 + rP, z0)
    platform2 = (x0 + rP*sin(np.radians(-120)), y0 + rP*cos(np.radians(-120)), z0)
    platform3 = (x0 + rP*sin(np.radians(120)), y0 + rP*cos(np.radians(120)), z0)

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
# test_forward_inverse()
# visualisation(position=[200,-120,-350])