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
    
    # Vérifier que le discriminant est positif
    if np.any(disc < 0): #pas raise de value error mais changer les valeurs dans l'output
        raise ValueError("Coordonnées hors de la zone de travail - discriminant négatif")
    
    t_minus = (-F - np.sqrt(disc)) / (G - E)
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
    if coords.ndim != 2 or coords.shape[1] != 3:
        raise ValueError("Les coordonnées doivent être de shape (n, 3)")
    
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

# Exemple d'utilisation avec un array unique
print("Inverse (single point):", DeltaInverse([[250, 250, -200]]))
print("Inverse (single point):", DeltaInverse([[0, 0, -300]]))

# Exemple d'utilisation avec plusieurs points
test_coords = np.array([
    [250, 250, -200],
    [0, 0, -300],
    [0, 0, -342.5]
])
print("\nInverse (multiple points):")
print(DeltaInverse(test_coords))

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