import DeltaCoord as d1
import numpy as np
import matplotlib.pyplot as plt
from trajectoires import jointmotangles, jointmotangles_trapezoidal, timediagram

# Générer les deux trajectoires avec les mêmes points de départ/arrivée
start_pos = [0, 0, -300]
end_pos = [100, 150, -350]

# 1. Trajectoire linéaire
print("Génération trajectoire linéaire...")
traj_linear, dur_linear = jointmotangles(
    start_pos, 
    end_pos, 
    speedmot=0.5, 
    steps_per_second=250
)

# 2. Trajectoire trapézoïdale
print("Génération trajectoire trapézoïdale...")
traj_trap, dur_trap = jointmotangles_trapezoidal(
    start_pos, 
    end_pos,
    v_max_deg_s=30.0,
    a_max_deg_s2=60.0,
    dt=0.001
)

# Créer la comparaison visuelle
fig, axes = plt.subplots(2, 2, figsize=(16, 10))

# Axe du temps pour chaque trajectoire
time_linear = np.linspace(0, dur_linear, len(traj_linear))
time_trap = np.linspace(0, dur_trap, len(traj_trap))

# 1. Angles - Linéaire
ax = axes[0, 0]
ax.plot(time_linear, traj_linear[:, 0], 'b-', label=r'$\theta_1$', linewidth=2)
ax.plot(time_linear, traj_linear[:, 1], 'g-', label=r'$\theta_2$', linewidth=2)
ax.plot(time_linear, traj_linear[:, 2], 'r-', label=r'$\theta_3$', linewidth=2)
ax.set_title("Angles - Trajectoire LINÉAIRE", fontsize=12, fontweight='bold')
ax.set_xlabel("Temps (s)")
ax.set_ylabel("Angle (degrés)")
ax.grid(True, alpha=0.3)
ax.legend()

# 2. Angles - Trapézoïdal
ax = axes[0, 1]
ax.plot(time_trap, traj_trap[:, 0], 'b-', label=r'$\theta_1$', linewidth=1.5)
ax.plot(time_trap, traj_trap[:, 1], 'g-', label=r'$\theta_2$', linewidth=1.5)
ax.plot(time_trap, traj_trap[:, 2], 'r-', label=r'$\theta_3$', linewidth=1.5)
ax.set_title("Angles - Trajectoire TRAPÉZOÏDALE (Sigmoïde)", fontsize=12, fontweight='bold')
ax.set_xlabel("Temps (s)")
ax.set_ylabel("Angle (degrés)")
ax.grid(True, alpha=0.3)
ax.legend()

# 3. Vitesses - Linéaire
ax = axes[1, 0]
vel_linear = np.diff(traj_linear, axis=0) / (dur_linear / (len(traj_linear) - 1)) if len(traj_linear) > 1 else np.zeros_like(traj_linear)
vel_linear = np.vstack([vel_linear[0], vel_linear])
ax.plot(time_linear, vel_linear[:, 0], 'b--', label=r'$\dot{\theta}_1$', linewidth=2)
ax.plot(time_linear, vel_linear[:, 1], 'g--', label=r'$\dot{\theta}_2$', linewidth=2)
ax.plot(time_linear, vel_linear[:, 2], 'r--', label=r'$\dot{\theta}_3$', linewidth=2)
ax.set_title("Vitesses - Trajectoire LINÉAIRE", fontsize=12, fontweight='bold')
ax.set_xlabel("Temps (s)")
ax.set_ylabel("Vitesse angulaire (degrés/s)")
ax.grid(True, alpha=0.3)
ax.legend()

# 4. Vitesses - Trapézoïdal
ax = axes[1, 1]
vel_trap = np.diff(traj_trap, axis=0) / (dur_trap / (len(traj_trap) - 1)) if len(traj_trap) > 1 else np.zeros_like(traj_trap)
vel_trap = np.vstack([vel_trap[0], vel_trap])
ax.plot(time_trap, vel_trap[:, 0], 'b--', label=r'$\dot{\theta}_1$', linewidth=1.5)
ax.plot(time_trap, vel_trap[:, 1], 'g--', label=r'$\dot{\theta}_2$', linewidth=1.5)
ax.plot(time_trap, vel_trap[:, 2], 'r--', label=r'$\dot{\theta}_3$', linewidth=1.5)
ax.set_title("Vitesses - Trajectoire TRAPÉZOÏDALE (Accél-Croisière-Décél)", fontsize=12, fontweight='bold')
ax.set_xlabel("Temps (s)")
ax.set_ylabel("Vitesse angulaire (degrés/s)")
ax.grid(True, alpha=0.3)
ax.legend()

fig.tight_layout()
plt.savefig("comparison_linear_vs_trapezoidal.png", dpi=150, bbox_inches='tight')
print("Comparaison sauvegardée : comparison_linear_vs_trapezoidal.png")

print(f"\n📊 Résumé des trajectoires :")
print(f"  Linéaire      : {len(traj_linear):4d} points | Durée : {dur_linear:6.3f} s")
print(f"  Trapézoïdale  : {len(traj_trap):4d} points | Durée : {dur_trap:6.3f} s")
