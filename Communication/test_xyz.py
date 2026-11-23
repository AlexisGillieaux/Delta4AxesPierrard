"""
test_xyz.py : PHASE 4 (côté PC, plus tard le Raspberry Pi), aller à une position X Y Z.

Le PC fait les calculs, comme le Raspberry Pi du robot final :
  - cinématique inverse : X Y Z -> angles θ1 à θ4. Seuls les NB_MOTEURS premiers (θ1 et θ2)
    sont envoyés : 2 moteurs sont branchés pour ce test ;
  - trajectoire, avec Coords/trajectoires4Axes.py :
      j : mouvement articulaire (jointmotangles_trapezoidal). Le PC envoie au Teensy les
          angles d'arrivée, la durée et l'accélération ; chaque variateur calcule lui-même
          son trapèze ;
      l : ligne droite (cartesian_trapezoidal_trajectory). Le PC découpe la trajectoire en
          16 segments de même durée et envoie les angles de fin de chaque segment.
Le Teensy (src/test_xyz.cpp) convertit les angles en pas et pilote les variateurs.
Après chaque mouvement, la position X Y Z atteinte est recalculée par cinématique directe
à partir des angles mesurés (les moteurs non branchés sont supposés exacts).

Préparation :
  1. pyserial dans le .venv du projet. Le paquet « serial » qui y est installé n'est PAS le
     bon (il n'a pas serial.Serial) :
         pip uninstall serial
         pip install pyserial
  2. PlatformIO > PROJECT TASKS > test_xyz > Upload (pas « Upload and Monitor » : le
     moniteur occuperait le port USB du Teensy).

Utilisation, depuis le dossier du projet :
    python Communication/test_xyz.py          (recherche le Teensy automatiquement)
    python Communication/test_xyz.py COM5     (port imposé)
Au lancement, les moteurs sont considérés à la pose POSE_DEPART (commande Z envoyée au
Teensy). Taper ensuite, par exemple :
    0 -150 -300 l     ligne droite jusqu'à X = 0, Y = -150, Z = -300 mm
    100 150 -350 j    mouvement articulaire jusqu'à X = 100, Y = 150, Z = -350 mm
    p                 angles actuels des moteurs
    z                 les moteurs sont de nouveau considérés à POSE_DEPART
    off               couper les moteurs
    q                 quitter (les moteurs restent dans leur état)
"""

import sys
import time
from pathlib import Path

import numpy as np

RACINE_PROJET = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RACINE_PROJET / "Coords"))
import trajectoires4Axes as traj  # noqa: E402

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None


# ============================ RÉGLAGES ============================

PORT = None                             # None : recherche automatique du Teensy, ou ex. "COM5"
DEBIT = 115200
POSE_DEPART = [0.0, 0.0, -300.0, 0.0]   # pose [x, y, z, rz] des moteurs au lancement (mm, °)

# Limites des moteurs des bras et de la ligne droite : les mêmes que test_articulaire.cpp
# et test_lineaire.cpp (même mouvement pour "0 150 -300 l" depuis 0 -150 -300)
V_MAX_DEG_S = 60.0
A_MAX_DEG_S2 = 120.0
V_MAX_MM_S = 200.0
A_MAX_MM_S2 = 600.0

NB_MOTEURS = 2                          # moteurs branchés (θ1, θ2) : autant que d'ADRESSES
                                        # dans test_xyz.cpp
NB_SEGMENTS = 16                        # doit valoir NB_SEGMENTS de test_xyz.cpp
TIMEOUT_S = 60.0                        # attente maximale de la réponse du Teensy

# ==================================================================

VID_PJRC = 0x16C0                       # identifiant USB du fabricant des cartes Teensy


def trouver_port():
    """Port série du premier Teensy branché, ou None."""
    for port in list_ports.comports():
        if port.vid == VID_PJRC:
            return port.device
    return None


def envoyer(liaison, commande):
    """Envoie une commande au Teensy et affiche ses lignes jusqu'à « OK » ou « ERREUR ».

    Returns:
        str: la dernière ligne (« OK θ1 θ2 » ou « ERREUR ... »)
    """
    apercu = commande if len(commande) <= 100 else commande[:97] + "..."
    print(f">>> {apercu}")
    liaison.write((commande + "\n").encode("ascii"))
    limite = time.monotonic() + TIMEOUT_S
    while time.monotonic() < limite:
        brut = liaison.readline()
        if not brut:
            continue
        ligne = brut.decode("utf-8", errors="replace").rstrip("\r\n")
        print(f"    {ligne}")
        if ligne.startswith("OK") or ligne.startswith("ERREUR"):
            return ligne
    return "ERREUR pas de réponse du Teensy"


def angles_mgi(pose):
    """Angles [θ1, θ2, θ3, θ4] (degrés) de la pose [x, y, z, rz] ; ValueError si inatteignable."""
    q = traj.tf.delta_inverse_from_poses4(np.asarray(pose, dtype=float))[0]
    if np.isnan(q).any():
        raise ValueError(f"Pose inatteignable : {pose}")
    return q


def commande_articulaire(depart, arrivee):
    """Commande J : profils de jointmotangles_trapezoidal (durée T et accélération).

    Returns:
        tuple: (commande, angles d'arrivée), commande None si le robot y est déjà
    """
    angles, duree = traj.jointmotangles_trapezoidal(depart, arrivee, V_MAX_DEG_S, A_MAX_DEG_S2,
                                                    dt=0.01)
    q = angles[-1]
    if duree <= 0.0:
        return None, q
    return f"J {duree:.4f} {A_MAX_DEG_S2:.3f} {texte_angles(q[:NB_MOTEURS])}", q


def commande_ligne(depart, arrivee):
    """Commande L : cartesian_trapezoidal_trajectory découpée en NB_SEGMENTS segments.

    Returns:
        tuple: (commande, angles d'arrivée), commande None si le robot y est déjà
    """
    angles, duree = traj.cartesian_trapezoidal_trajectory(
        depart, arrivee, V_MAX_MM_S, A_MAX_MM_S2, dt=0.001,
        v_max_deg_s=V_MAX_DEG_S, a_max_deg_s2=A_MAX_DEG_S2)
    q = np.atleast_2d(angles)[-1]
    if duree <= 0.0:
        return None, q
    temps = np.linspace(0.0, duree, len(angles))
    fins = np.linspace(0.0, duree, NB_SEGMENTS + 1)[1:]       # fin de chaque segment
    # points[k] : angles des moteurs branchés à la fin du segment k
    points = np.column_stack([np.interp(fins, temps, angles[:, m]) for m in range(NB_MOTEURS)])
    dt_ms = max(1, round(duree * 1000.0 / NB_SEGMENTS))
    return f"L {dt_ms} {texte_angles(points.ravel())}", q


def texte_angles(angles):
    """Angles séparés par des espaces, au 1/10000 de degré."""
    return " ".join(f"{a:.4f}" for a in angles)


def definir_depart(liaison):
    """Commande Z : les moteurs sont considérés à POSE_DEPART. Retourne la pose, ou None."""
    q = angles_mgi(POSE_DEPART)
    if not envoyer(liaison, f"Z {texte_angles(q[:NB_MOTEURS])}").startswith("OK"):
        return None
    print(f"Moteurs considérés à la pose {POSE_DEPART[:3]} mm ({texte_thetas(q[:NB_MOTEURS])})")
    return list(POSE_DEPART)


def texte_thetas(angles):
    """« θ1 = ...°, θ2 = ...° » pour l'affichage."""
    return ", ".join(f"θ{k + 1} = {a:.3f}°" for k, a in enumerate(angles))


def deplacer(liaison, pose, texte):
    """Traite « X Y Z [j|l] ». Retourne la nouvelle pose (inchangée en cas d'échec)."""
    morceaux = texte.split()
    type_mouvement = "j"
    if len(morceaux) == 4:
        type_mouvement = morceaux.pop()
    if len(morceaux) != 3 or type_mouvement not in ("j", "l"):
        print("Tapez X Y Z puis j ou l, ex. 0 -150 -300 l")
        return pose
    try:
        cible = [float(v) for v in morceaux] + [pose[3]]       # le plateau ne tourne pas
        if type_mouvement == "l":
            commande, q = commande_ligne(pose, cible)
        else:
            commande, q = commande_articulaire(pose, cible)
    except ValueError as erreur:
        print(f"Refusé : {erreur}")
        return pose
    if commande is None:
        print("Le robot est déjà à cette position.")
        return pose

    print(f"Angles visés (cinématique inverse) : {texte_thetas(q)} "
          f"(seuls θ1 à θ{NB_MOTEURS} sont envoyés)")
    reponse = envoyer(liaison, commande)
    if not reponse.startswith("OK"):
        print("Échec : la pose suivie par le PC n'est pas mise à jour.")
        return pose
    atteints = [float(v) for v in reponse.split()[1:1 + NB_MOTEURS]]
    ecarts = ", ".join(f"θ{k + 1} {a - q[k]:+.3f}°" for k, a in enumerate(atteints))
    print(f"Angles atteints : {texte_thetas(atteints)} (écarts : {ecarts})")

    # Position réellement atteinte : cinématique directe des angles mesurés
    q_atteint = np.array(q, dtype=float)
    q_atteint[:NB_MOTEURS] = atteints
    xyz = traj.tf.delta_forward_to_poses4(q_atteint)[0, :3]
    ecart_mm = np.linalg.norm(xyz - np.asarray(cible[:3]))
    print(f"Position atteinte : X = {xyz[0]:.2f}, Y = {xyz[1]:.2f}, Z = {xyz[2]:.2f} mm "
          f"(écart {ecart_mm:.2f} mm à la cible)")
    return cible


def main():
    # Une console qui ne connaît pas « θ » l'affiche « ? » au lieu de planter
    sys.stdout.reconfigure(errors="replace")
    if serial is None or not hasattr(serial, "Serial"):
        print("pyserial n'est pas installé (le paquet « serial » n'est pas le bon) :\n"
              "    pip uninstall serial\n    pip install pyserial")
        return 1
    port = sys.argv[1] if len(sys.argv) > 1 else (PORT or trouver_port())
    if port is None:
        print("Teensy introuvable : branchez-le, ou donnez le port (ex. python test_xyz.py COM5).")
        return 1
    print(f"Teensy sur {port}")

    with serial.Serial(port, DEBIT, timeout=0.2) as liaison:
        # Messages de démarrage du Teensy (il attend l'ouverture du port pour démarrer)
        fin = time.monotonic() + 1.5
        while time.monotonic() < fin:
            brut = liaison.readline()
            if brut:
                print(f"    {brut.decode('utf-8', errors='replace').rstrip()}")

        pose = definir_depart(liaison)
        if pose is None:
            return 1
        while True:
            try:
                texte = input("\nX Y Z [j|l], p, z, off ou q : ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break
            if texte in ("q", "quit"):
                break
            if texte == "":
                continue
            if texte == "p":
                envoyer(liaison, "P")
            elif texte == "off":
                envoyer(liaison, "OFF")
                print("Si vous tournez les moteurs à la main, tapez ensuite z après les avoir "
                      "replacés à la pose de départ.")
            elif texte == "z":
                pose = definir_depart(liaison) or pose
            else:
                pose = deplacer(liaison, pose, texte)
    print("Fin (les moteurs restent dans leur état : sous couple s'ils l'étaient).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
