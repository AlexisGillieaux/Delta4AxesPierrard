from vpython import *

scene = canvas(title='Robot Delta - Modèle réel', width=1000, height=700, background=color.black)
scene.range = 22
scene.forward = vector(0, -0.4, -1)

# -----------------------------------------------------------
# Paramètres du robot Delta
# -----------------------------------------------------------
R_base = 7.0
R_effector = 2.8
base_z = 8.0
plate_z = -3.0
angles = [0, 2*pi/3, 4*pi/3]

current_yaw = 0.0

# -----------------------------------------------------------
# Axes fictifs pour se repérer
# -----------------------------------------------------------
axis_x = arrow(pos=vector(0, 0, 0), axis=vector(10, 0, 0), shaftwidth=0.12, color=color.red)
axis_y = arrow(pos=vector(0, 0, 0), axis=vector(0, 10, 0), shaftwidth=0.12, color=color.green)
axis_z = arrow(pos=vector(0, 0, 0), axis=vector(0, 0, 10), shaftwidth=0.12, color=color.blue)
label_x = text(text='X', pos=vector(10.8, 0, 0), color=color.red, height=0.7)
label_y = text(text='Y', pos=vector(0, 10.8, 0), color=color.green, height=0.7)
label_z = text(text='Z', pos=vector(0, 0, 10.8), color=color.blue, height=0.7)

# -----------------------------------------------------------
# Objets fixes
# -----------------------------------------------------------
base_plate = ring(pos=vector(0, 0, base_z), axis=vector(0, 0, 1), radius=R_base + 0.9, thickness=0.25, color=color.gray(0.5), opacity=0.60)
base_ring = ring(pos=vector(0, 0, base_z + 0.7), axis=vector(0, 0, 1), radius=R_base, thickness=0.18, color=color.gray(0.35), opacity=0.45)

motors = []
for a in angles:
    p = vector(cos(a) * R_base, sin(a) * R_base, base_z)
    m = sphere(pos=p, radius=0.5, color=color.orange)
    motors.append(m)

# -----------------------------------------------------------
# Partie inférieure mobile
# -----------------------------------------------------------
plate = cylinder(pos=vector(0, 0, plate_z), axis=vector(0, 0, 0.6), radius=R_effector + 0.7, color=color.cyan, opacity=0.40)
plate_center = sphere(pos=vector(0, 0, plate_z), radius=0.35, color=color.white, opacity=0.25)

platform_joints = []
for a in angles:
    jp = vector(cos(a) * R_effector, sin(a) * R_effector, plate_z)
    j = sphere(pos=jp, radius=0.22, color=color.green, opacity=0.60)
    platform_joints.append(j)

arms = []
for i, a in enumerate(angles):
    p1 = motors[i].pos
    p2 = platform_joints[i].pos
    arm = cylinder(pos=p1, axis=p2 - p1, radius=0.12, color=color.white)
    arms.append(arm)

# -----------------------------------------------------------
# Paramètres et cinématique
# -----------------------------------------------------------
step = 0.12
rot_step = 0.08

def update_effector(x, y, z, yaw=0):
    global current_yaw
    current_yaw = yaw
    plate.pos = vector(x, y, z)
    plate_center.pos = vector(x, y, z)

    for i, a in enumerate(angles):
        p_joint = vector(x, y, z) + vector(cos(a + yaw) * R_effector, sin(a + yaw) * R_effector, 0)
        platform_joints[i].pos = p_joint
        arms[i].pos = motors[i].pos
        arms[i].axis = p_joint - motors[i].pos

def retour_0_0_0():
    update_effector(0, 0, -2.5, 0)

def move(dx, dy, dz):
    x = plate.pos.x + dx
    y = plate.pos.y + dy
    z = plate.pos.z + dz
    update_effector(x, y, z, current_yaw)

def rotate_z(delta):
    current = plate.pos
    update_effector(current.x, current.y, current.z, current_yaw + delta)

# -----------------------------------------------------------
# Boutons VPython officiels (selon la documentation)
# -----------------------------------------------------------
button(text='X-', bind=lambda b: move(-step * 3, 0, 0))
button(text='X+', bind=lambda b: move(step * 3, 0, 0))
button(text='Y-', bind=lambda b: move(0, -step * 3, 0))
button(text='Y+', bind=lambda b: move(0, step * 3, 0))
button(text='Z-', bind=lambda b: move(0, 0, -step * 3))
button(text='Z+', bind=lambda b: move(0, 0, step * 3))
button(text='Rot Z-', bind=lambda b: rotate_z(-rot_step * 2))
button(text='Rot Z+', bind=lambda b: rotate_z(rot_step * 2))
button(text='Reset', bind=lambda b: retour_0_0_0())

# -----------------------------------------------------------
# Boucle principale : Mouvement continu via le clavier
# -----------------------------------------------------------
retour_0_0_0()

# Légende d'aide
scene.append_to_caption("\n<b>Contrôle continu au clavier (maintenir enfoncé) :</b>\n")
scene.append_to_caption("• Flèches Gauche / Droite : Axe X\n")
scene.append_to_caption("• Flèches Haut / Bas : Axe Y\n")
scene.append_to_caption("• Page Up / Page Down : Axe Z\n")
scene.append_to_caption("• A / D : Rotation Z\n")

while True:
    rate(60)
    
    # Capture native des touches enfoncées dans VPython
    k = keysdown()
    
    if 'left' in k: move(-step, 0, 0)
    if 'right' in k: move(step, 0, 0)
    if 'up' in k: move(0, step, 0)
    if 'down' in k: move(0, -step, 0)
    if 'pageup' in k: move(0, 0, step)
    if 'pagedown' in k: move(0, 0, -step)
    if 'a' in k: rotate_z(-rot_step)
    if 'd' in k: rotate_z(rot_step)