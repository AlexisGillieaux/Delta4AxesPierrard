"""
Exporte une trajectoire calculée par Coords/trajectoires4Axes.py dans un fichier CSV
que lit Code_C_V5_Trajectoire.c pour faire bouger un moteur.

Format du CSV (une ligne par point, à intervalles de temps réguliers) :
    t_s,theta1_deg,theta2_deg,theta3_deg,theta4_deg
    0.000000,-13.470000,...

Utilisation :
    python Mouvements_Robot/exporter_trajectoire.py
puis, dans le dossier Mouvements_Robot :
    .\\Code_C_V5_Trajectoire.exe trajectoire_moteurs.csv 1 COM3

Pour exporter une autre trajectoire depuis votre propre code :
    from exporter_trajectoire import exporter_csv
    exporter_csv(angles, duree)
"""

import sys
from pathlib import Path

import numpy as np

DOSSIER = Path(__file__).resolve().parent
sys.path.insert(0, str(DOSSIER.parent / "Coords"))
import trajectoires4Axes as traj  # noqa: E402

FICHIER_CSV = DOSSIER / "trajectoire_moteurs.csv"

# Pas de temps du CSV. Le programme C n'envoie de toute façon pas plus d'un point
# toutes les PERIODE_MS (200 ms par défaut, à 9600 bauds) : inutile d'exporter plus fin.
DT_EXPORT_S = 0.2


def exporter_csv(angles, duree_s, chemin=FICHIER_CSV, dt=DT_EXPORT_S):
    """Écrit les angles [θ1, θ2, θ3, θ4] d'une trajectoire dans un CSV.

    La trajectoire est rééchantillonnée (interpolation linéaire) avec un pas régulier
    au moins égal à dt, pour que le programme C n'ait aucun point à sauter. Le
    dernier point tombe exactement sur duree_s, la pose d'arrivée est donc conservée.

    Args:
        angles (array-like): angles des moteurs de shape (n, 4), en degrés, échantillonnés
            à intervalles réguliers entre 0 et duree_s (sortie de jointmotangles,
            jointmotangles_trapezoidal ou cartesian_trapezoidal_trajectory)
        duree_s (float): durée totale du mouvement en secondes
        chemin (str | Path): fichier CSV à écrire
        dt (float): pas de temps du fichier (s)

    Returns:
        Path: le chemin du fichier écrit
    """
    angles = np.asarray(angles, dtype=float)
    if angles.ndim != 2 or angles.shape[1] != 4:
        raise ValueError(f"Angles attendus de shape (n, 4), reçu {angles.shape}")
    if not np.isfinite(angles).all():
        raise ValueError("La trajectoire contient des NaN : une pose est inatteignable")
    if not (np.isfinite(dt) and dt > 0):
        raise ValueError(f"dt doit être strictement positif (reçu {dt})")

    if len(angles) < 2 or duree_s <= 0:
        temps_csv = np.array([0.0])
        angles_csv = angles[:1]
    else:
        temps = np.linspace(0.0, duree_s, len(angles))
        n = max(1, int(np.floor(duree_s / dt + 1e-9)))      # pas = duree_s / n >= dt
        temps_csv = np.linspace(0.0, duree_s, n + 1)
        angles_csv = np.column_stack(
            [np.interp(temps_csv, temps, angles[:, i]) for i in range(4)]
        )

    chemin = Path(chemin)
    np.savetxt(chemin, np.column_stack((temps_csv, angles_csv)), delimiter=",", fmt="%.6f",
               header="t_s,theta1_deg,theta2_deg,theta3_deg,theta4_deg", comments="")
    return chemin


if __name__ == "__main__":
    # Trajectoire d'essai volontairement lente : 10 °/s et 20 °/s² au maximum.
    # Le moteur 1 passe de -13,5° à 42,2° (55,6° en 6 s).
    depart = [0, 0, -300, 0]
    arrivee = [100, 150, -350, 90]
    angles, duree = traj.jointmotangles_trapezoidal(
        depart, arrivee, v_max_deg_s=10.0, a_max_deg_s2=20.0, dt=0.01
    )
    traj.resume_trajectoire(angles, duree)
    chemin = exporter_csv(angles, duree)
    print(f"\nTrajectoire exportée : {chemin}")
