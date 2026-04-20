import DeltaCoord_fixed as d1
import numpy as np
import matplotlib.pyplot as plt

def lineartrajectory(start, end, stepsmm):
    steps = int(np.linalg.norm(np.array(end) - np.array(start)) / stepsmm)
    trajectory = np.linspace(start, end, steps + 1)
    return trajectory

def jointmotangles_trapezoidal(start, end, v_max_deg_s, a_max_deg_s2, dt=0.001):
    """Génère une trajectoire articulaire synchronisée avec profils trapézoïdaux
    pour un robot delta. Tous les moteurs terminent leur mouvement au même moment
    avec des profils accélération-vitesse-décélération.
    
    Args:
        start (array-like): Position xyz initiale
        end (array-like): Position xyz finale
        v_max_deg_s (float): Vitesse angulaire max des moteurs (degrés/s)
        a_max_deg_s2 (float): Accélération angulaire max des moteurs (degrés/s²)
        dt (float): Pas de temps (s), défaut 0.001
        
    Returns:
        tuple:
            numpy.ndarray: Trajectoire de shape (n_steps, 3) avec angles en degrés
            float: Durée totale du mouvement en secondes
    """
    # Calculer les angles de départ et d'arrivée
    start_angles = np.asarray(d1.DeltaInverse(start)).reshape(-1, 3)[0]
    end_angles = np.asarray(d1.DeltaInverse(end)).reshape(-1, 3)[0]
    
    q_start = np.array(start_angles, dtype=float)
    q_end = np.array(end_angles, dtype=float)
    delta_q = np.abs(q_end - q_start)
    signs = np.sign(q_end - q_start)
    
    # 1) Calculer le temps minimal pour chaque moteur avec v_max et a_max
    min_times = []
    for d in delta_q:
        if d <= 0:
            min_times.append(0.0)
            continue
        if v_max_deg_s**2 / a_max_deg_s2 > d:
            # Profil triangulaire (n'atteint pas v_max)
            t = 2.0 * np.sqrt(d / a_max_deg_s2)
        else:
            # Profil trapézoïdal (accél + vitesse constante + décél)
            t = d / v_max_deg_s + v_max_deg_s / a_max_deg_s2
        min_times.append(t)
    
    t_total = max(min_times)
    
    if t_total < dt or t_total == 0.0:
        return np.array([q_start], dtype=float), 0.0
    
    # 2) Générer l'axe du temps
    time_array = np.arange(0.0, t_total + dt/2, dt)
    q_path = np.zeros((len(time_array), len(q_start)))
    
    # 3) Pour chaque moteur, calculer le profil trapezoïdal synchronisé
    for i, d in enumerate(delta_q):
        if d == 0.0:
            q_path[:, i] = q_start[i]
            continue
        
        # Résoudre v_peak pour une distance d en temps total T
        # d = v_peak*T - v_peak^2/a  =>  v^2 - a*T*v + a*d = 0
        a = a_max_deg_s2
        T = t_total
        discr = (a * T)**2 - 4.0 * a * d
        
        if discr < 0:
            v_peak = v_max_deg_s
        else:
            v_peak_candidate = (a * T - np.sqrt(discr)) / 2.0
            v_peak = min(v_peak_candidate, v_max_deg_s)
        
        # Calculer les temps d'accélération et de croisière
        if v_peak <= 1e-12:
            t_acc = T / 2.0
            v_peak = a * t_acc
            t_cruise = 0.0
        else:
            t_acc = v_peak / a
            t_cruise = T - 2.0 * t_acc
            if t_cruise < 0:
                t_acc = T / 2.0
                v_peak = a * t_acc
                t_cruise = 0.0
        
        dist_acc = 0.5 * a * t_acc**2
        
        # Construire le profil de position pour ce moteur
        pos = np.zeros(len(time_array))
        for j, t in enumerate(time_array):
            if t < t_acc:
                # Phase d'accélération
                pos[j] = 0.5 * a * t**2
            elif t < t_acc + t_cruise:
                # Phase vitesse constante
                pos[j] = dist_acc + v_peak * (t - t_acc)
            else:
                # Phase de décélération
                t_decel = t - t_acc - t_cruise
                pos[j] = d - 0.5 * a * (T - t)**2
        
        # Appliquer le signe et l'offset
        q_path[:, i] = q_start[i] + signs[i] * pos
    
    return q_path, t_total

def jointmotangles(start, end, deg_per_step, steps_per_second=1.0):
    """Version linéaire simple de jointmotangles (pour compatibilité).
    
    A partir d'une position xyz initiale et une position xyz finale, va renvoyer
    un array numpy des positions angulaires des moteurs où le moteur faisant le plus
    grand angle se deplace à la vitesse speedmot tandis que les deux autres vont
    reduire leur vitesse afin de terminer leur mouvement en meme temps que le moteur
    faisant le plus grand deplacement.

    Args:
        start (numpy.ndarray): xyz initiale du mouvement
        end (numpy.ndarray): xyz finale du mouvement
        speedmot (float): vitesse du moteur principal (degrés par pas)
        steps_per_second (float): nombre de pas par seconde du moteur

    Returns:
        tuple:
            numpy.ndarray: array des positions angulaires des moteurs
                de shape (n_steps + 1, 3), où chaque ligne est [theta1, theta2, theta3]
            float: durée estimée du mouvement en secondes
    """
    start_angles = np.asarray(d1.DeltaInverse(start)).reshape(-1, 3)[0]
    end_angles = np.asarray(d1.DeltaInverse(end)).reshape(-1, 3)[0]

    deltas = end_angles - start_angles
    max_motor = np.argmax(np.abs(deltas))
    max_delta = abs(deltas[max_motor])

    if max_delta == 0 or deg_per_step == 0:
        return np.array([start_angles], dtype=float), 0.0

    if steps_per_second <= 0:
        raise ValueError("steps_per_second doit etre strictement positif")

    n_steps = int(np.ceil(max_delta / abs(deg_per_step)))
    n_steps = max(1, n_steps)
    duration_sec = n_steps / steps_per_second

    trajectory = np.linspace(start_angles, end_angles, n_steps + 1)
    return trajectory, duration_sec


def timediagram(trajectory, duration_sec):
    """Trace l'évolution des angles des trois moteurs du robot delta en fonction du temps.
    
    Args:
        trajectory (numpy.ndarray): Array de shape (n_steps + 1, 3) contenant les angles 
            [theta1, theta2, theta3] en degrés pour chaque pas
        duration_sec (float): Durée totale du mouvement en secondes
    
    Returns:
        matplotlib.figure.Figure: La figure contenant les graphiques
    """
    n_steps = len(trajectory) - 1
    
    # Créer l'axe du temps
    if n_steps == 0:
        print("Avertissement : trajectoire vide, impossible de tracer")
        return None
    
    if duration_sec == 0:
        time_array = np.arange(len(trajectory))
        time_label = "Pas"
    else:
        time_array = np.linspace(0, duration_sec, len(trajectory))
        time_label = "Temps (s)"
    
    # Créer la figure avec 2 sous-graphiques
    fig, (ax_pos, ax_vel) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Graphique 1 : Angles en fonction du temps
    ax_pos.plot(time_array, trajectory[:, 0], 'b-', label=r'$\theta_1$ (Moteur 1)', linewidth=2)
    ax_pos.plot(time_array, trajectory[:, 1], 'g-', label=r'$\theta_2$ (Moteur 2)', linewidth=2)
    ax_pos.plot(time_array, trajectory[:, 2], 'r-', label=r'$\theta_3$ (Moteur 3)', linewidth=2)
    
    ax_pos.set_title("Évolution des angles des moteurs du robot Delta", fontsize=14, fontweight='bold')
    ax_pos.set_xlabel(time_label, fontsize=11)
    ax_pos.set_ylabel("Angle (degrés)", fontsize=11)
    ax_pos.grid(True, alpha=0.3)
    ax_pos.legend(loc='best', fontsize=10)
    
    # Graphique 2 : Vitesses angulaires (dérivée numérique)
    if len(trajectory) > 1:
        # Calculer les vitesses angulaires par différentation numérique
        dt = duration_sec / n_steps if duration_sec > 0 else 1.0
        velocities = np.diff(trajectory, axis=0) / dt
        
        # Étendr le tableau des velocités pour correspondre à la taille de trajectory
        # (utiliser la première vélocité pour le premier point)
        velocities = np.vstack([velocities[0], velocities])
        
        ax_vel.plot(time_array, velocities[:, 0], 'b--', label=r'$\dot{\theta}_1$ (Moteur 1)', linewidth=2)
        ax_vel.plot(time_array, velocities[:, 1], 'g--', label=r'$\dot{\theta}_2$ (Moteur 2)', linewidth=2)
        ax_vel.plot(time_array, velocities[:, 2], 'r--', label=r'$\dot{\theta}_3$ (Moteur 3)', linewidth=2)
    
    ax_vel.set_title("Vitesses angulaires des moteurs du robot Delta", fontsize=14, fontweight='bold')
    ax_vel.set_xlabel(time_label, fontsize=11)
    ax_vel.set_ylabel("Vitesse angulaire (degrés/s)" if duration_sec > 0 else "Vitesse angulaire (degrés/pas)", fontsize=11)
    ax_vel.grid(True, alpha=0.3)
    ax_vel.legend(loc='best', fontsize=10)
    
    fig.tight_layout()
    
    return fig



def cartesian_trapezoidal_trajectory(start, end, v_max_mm_s, a_max_mm_s2, dt=0.001):
    """
    Génère une trajectoire en ligne droite (XYZ) avec profil de vitesse 
    trapézoïdal, puis convertit en angles moteurs.
    """
    start = np.array(start)
    end = np.array(end)
    distance = np.linalg.norm(end - start)
    direction = (end - start) / distance

    # 1. Calcul du profil trapézoïdal sur la distance XYZ
    if (v_max_mm_s**2 / a_max_mm_s2) > distance:
        # Profil triangulaire
        t_acc = np.sqrt(distance / a_max_mm_s2)
        t_total = 2 * t_acc
        v_peak = a_max_mm_s2 * t_acc
        t_cruise = 0
    else:
        # Profil trapézoïdal
        t_acc = v_max_mm_s / a_max_mm_s2
        t_cruise = (distance - (v_max_mm_s**2 / a_max_mm_s2)) / v_max_mm_s
        t_total = 2 * t_acc + t_cruise
        v_peak = v_max_mm_s

    time_array = np.arange(0, t_total + dt, dt)
    joint_trajectory = []

    # 2. Génération des points XYZ et conversion inverse
    for t in time_array:
        # Calcul de la distance parcourue s(t) à l'instant t
        if t < t_acc:
            s = 0.5 * a_max_mm_s2 * t**2
        elif t < t_acc + t_cruise:
            s = (0.5 * a_max_mm_s2 * t_acc**2) + v_peak * (t - t_acc)
        else:
            t_decel = t - (t_acc + t_cruise)
            s = distance - 0.5 * a_max_mm_s2 * (t_total - t)**2
        
        # Point XYZ actuel
        current_xyz = start + s * direction
        
        # Conversion en angles (Inverse Kinematics)
        angles = np.asarray(d1.DeltaInverse(current_xyz)).reshape(-1, 3)[0]
        joint_trajectory.append(angles)

    return np.array(joint_trajectory), t_total



def test():
    print("="*70)
    print("TEST 1 : Trajectoire LINÉAIRE (jointmotangles)")
    print("="*70)
    trajectory_linear, duration_linear = jointmotangles(
        [0, 0, -300], 
        [100, 150, -350], 
        deg_per_step=0.5, 
        steps_per_second=250
    )
    print(f"Nombre de points : {len(trajectory_linear)}")
    print(f"Durée estimée : {duration_linear:.3f} s")
    print(f"Premiers angles : {trajectory_linear[0]}")
    print(f"Derniers angles : {trajectory_linear[-1]}")
    
    print("\n" + "="*70)
    print("TEST 2 : Trajectoire TRAPÉZOÏDALE (jointmotangles_trapezoidal)")
    print("="*70)
    trajectory_trap, duration_trap = jointmotangles_trapezoidal(
        [0, 0, -300],
        [100, 150, -350],
        v_max_deg_s=30.0,
        a_max_deg_s2=60.0,
        dt=0.001
    )
    print(f"Nombre de points : {len(trajectory_trap)}")
    print(f"Durée estimée : {duration_trap:.3f} s")
    print(f"Premiers angles : {trajectory_trap[0]}")
    print(f"Derniers angles : {trajectory_trap[-1]}")
    
    print("\n" + "="*70)
    print("VISUALISATION : Trajectoire trapézoïdale (sigmoïde)")
    print("="*70)
    fig = timediagram(trajectory_trap, duration_trap)
    if fig:
        plt.savefig("trajectoire_trapezoidal.png", dpi=150, bbox_inches='tight')
        print("Graphique sauvegardé : trajectoire_trapezoidal.png")
    plt.show()
print(d1.DeltaInverse([0, 0, -300]))
print(d1.DeltaInverse([100, 150, -350]))

test()

# --- TEST ET AFFICHAGE AVEC SAUVEGARDE ---

start_pos = [-200, 200, -200]
end_pos = [175, -250, -350]

# Paramètres : 200 mm/s max, 500 mm/s² accélération
traj, duration = cartesian_trapezoidal_trajectory(
    start_pos, end_pos, v_max_mm_s=200.0, a_max_mm_s2=500.0, dt=0.01
)

print(f"Durée du mouvement : {duration:.3f} s")
fig = timediagram(traj, duration)

if fig:
    
    # Sauvegarde du fichier avant plt.show()
    plt.savefig("trajectoire_linéaire.png", dpi=150, bbox_inches='tight')
    print("Graphique sauvegardé avec succès : trajectoire_linéaire.png")
    
    plt.show()







