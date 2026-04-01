from math import *

def rotate(x, y, angle_deg):
    a = radians(angle_deg)
    return x*cos(a) - y*sin(a),  x*sin(a) + y*cos(a)

def solve_one_arm(xr, yr, z, wB, rP, L, l):
    """Résout pour un bras dont la direction est l'axe Y local"""
    a = wB - rP
    E = 2*L*(yr + a)
    F = 2*L*z
    G = xr**2 + yr**2 + z**2 + a**2 + L**2 + 2*a*yr - l**2
    disc = E**2 + F**2 - G**2
    t_minus = (-F - sqrt(disc)) / (G - E)
    return degrees(2*atan(t_minus))

def DeltaInverse(x, y, z):
    rB = 0.2*sqrt(3)/3    # Rayon du cercle sur lequel sont les moteurs
    wB = rB/2               # Rayon de la base du triangle équilatéral formé par les moteurs
    rP = 0.04              # Rayon de la plateforme (distance du centre à un point d'attache)
    L = 0.2               # Longueur des bras moteurs
    l = 0.43               # Longueur des bras parallèles (entre les points d'attache sur la plateforme et les extrémités des bras moteurs)

    # Bras 1 : pas de rotation
    x1, y1 = rotate(x, y, 0)
    t1 = solve_one_arm(x1, y1, z, wB, rP, L, l)

    # Bras 2 : rotation de -120°
    x2, y2 = rotate(x, y, -120)
    t2 = solve_one_arm(x2, y2, z, wB, rP, L, l)

    # Bras 3 : rotation de +120°
    x3, y3 = rotate(x, y, 120)
    t3 = solve_one_arm(x3, y3, z, wB, rP, L, l)

    return t1, t2, t3

#def print_inverse():
print("General (simplified):", DeltaInverse(0.3, 0.5, -1.1))
print("General (simplified):", DeltaInverse(0, 0, -0.9))

def DeltaForward(t1, t2, t3):
    rP = 0.044              # Rayon de la plateforme (distance du centre à un point d'attache)
    rB = 0.567*sqrt(3)/3    # Rayon du cercle sur lequel sont les moteurs
    wP = rP/2               # Rayon de la base du triangle équilatéral formé par les points d'attache sur la plateforme
    wB = rB/2               # Rayon de la base du triangle équilatéral formé par les moteurs
    sP = rP*3/sqrt(3)       # Rayon du cercle sur lequel sont les points d'attache sur la plateforme
    sB = rB*3/sqrt(3)       # Rayon du cercle sur lequel sont les moteurs
    L = 0.524               # Longueur des bras moteurs
    l = 1.244               # Longueur des bras parallèles (entre les points d'attache sur la plateforme et les extrémités des bras moteurs)
    t1_rad = radians(t1)
    t2_rad = radians(t2)
    t3_rad = radians(t3)
    A1 = [0, -wB - L*cos(t1_rad)+rP, -L*sin(t1_rad)]
    A2 = [(sqrt(3)/2)*(wB + L*cos(t2_rad))-sP/2, (1/2)*(wB + L*cos(t2_rad))-wP, -L*sin(t2_rad)]
    A3 = [-(sqrt(3)/2)*(wB + L*cos(t3_rad))+sP/2, (1/2)*(wB + L*cos(t3_rad))-wP, -L*sin(t3_rad)]
    a11 = 2*(A3[0] - A1[0])
    a12 = 2*(A3[1] - A1[1])
    a13 = 2*(A3[2] - A1[2])
    a21 = 2*(A3[0] - A2[0])
    a22 = 2*(A3[1] - A2[1])
    a23 = 2*(A3[2] - A2[2])
    b1 = l**2 -l**2 - A1[0]**2 -A1[1]**2 -A1[2]**2 + A3[0]**2 + A3[1]**2 + A3[2]**2
    b2 = l**2 -l**2 - A2[0]**2 -A2[1]**2 -A2[2]**2 + A3[0]**2 + A3[1]**2 + A3[2]**2
    if a13 == 0 or a23 == 0:
        aS = a11
        bS = a12
        cS = l**2 -l**2 - A1[0]**2 -A1[1]**2 + A3[0]**2 + A3[1]**2
        dS = a21
        eS = a22
        fS = l**2 -l**2 - A2[0]**2 -A2[1]**2 + A3[0]**2 + A3[1]**2
        x = (cS*eS - fS*bS) / (aS*eS - dS*bS)
        y = (aS*fS - dS*cS) / (aS*eS - dS*bS)
        B = -2*A1[2]
        C = A1[2]**2 -l**2 + (x-A1[0])**2 + (y-A1[1])**2
        zplus = (-B + sqrt(B**2 - 4*C)) / 2
        zminus = (-B - sqrt(B**2 - 4*C)) / 2
        # On choisit la solution avec z négatif (la plateforme est en dessous des moteurs)
        if zplus < 0:
            return x, y, zplus
        else:
            return x, y, zminus
    else:
        a1 = a11/a13 - a21/a23
        a2 = a12/a13 - a22/a23
        a3 = b2/a23 - b1/a13
        a4 = -a2/a1
        a5 = -a3/a1
        a6 = (-a21*a4 -a22)/a23
        a7 = (b2-a21*a5)/a23
        a = a4**2 + 1 + a6**2
        b = 2*a4*(a5 - A1[0]) - 2*A1[1] + 2*a6*(a7 - A1[2])
        c = a5*(a5 - 2*A1[0]) + a7*(a7 - 2*A1[2]) + A1[0]**2 + A1[1]**2 + A1[2]**2 - l**2
        yplus = (-b + sqrt(b**2 - 4*a*c)) / (2*a)
        yminus = (-b - sqrt(b**2 - 4*a*c)) / (2*a)
        xplus = a4*yplus + a5
        zplus = a6*yplus + a7
        xminus = a4*yminus + a5
        zminus = a6*yminus + a7
        # On choisit la solution avec z négatif (la plateforme est en dessous des moteurs)
        if zplus < 0:
            return xplus, yplus, zplus
        else:
            return xminus, yminus, zminus

#def print_directe():
print("General (simplified):", DeltaForward(47.5,-11.5,21.4))
print("General (simplified):", DeltaForward(-20.5,-20.5,-20.5))