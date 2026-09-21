from nicegui import ui

# --- ÉTAT GLOBAL ET CONFIGURATION DU ROBOT ---

robot_state = {
    "x": 0.0,
    "y": 0.0,
    "z": 1.0,  # Position initiale en Z (au-dessus du sol)
    "rz": 0.0,  # Rotation du 4ème axe (en radians)
}

LIMITS = {
    "x": (-0.8, 0.8),
    "y": (-0.8, 0.8),
    "z": (0.3, 1.8),
}

BASE_Z = 2.0

# Variables globales gérées dynamiquement par page
end_effector = None
link_lines = []
scene = None
coord_label = None
active_jog = {"axis": None, "delta": 0.0}


def update_robot_graphics():
    """Met à jour la position de l'effecteur et redessine les bras de liaison"""
    global end_effector, link_lines, scene, coord_label

    if not end_effector or not scene or not coord_label:
        return

    x = robot_state["x"]
    y = robot_state["y"]
    z = robot_state["z"]
    rz = robot_state["rz"]

    # 1. Déplacer et orienter correctement l'effecteur mobile
    for child in list(end_effector.children):
        child.delete()
    with end_effector:
        # Plateau horizontal de la plateforme mobile
        ui.scene.box(0.4, 0.1, 0.4).material("#2c3e50").rotate(1.5708, 0, 0)
        # 4ème axe : cylindre vertical orienté selon Z
        ui.scene.cylinder(0.05, 0.05, 0.5).material("#e74c3c").rotate(
            1.5708, 0, rz
        ).move(0, 0, 0)

    end_effector.move(x, y, z)

    # 2. Redessiner les 3 bras cinématiques de liaison
    base_joints = [
        (0.6, 0.0, BASE_Z),
        (-0.3, 0.52, BASE_Z),
        (-0.3, -0.52, BASE_Z),
    ]

    for line in link_lines:
        line.delete()
    link_lines.clear()

    for bx, by, bz in base_joints:
        control1 = [
            bx + (x - bx) / 3,
            by + (y - by) / 3,
            bz + (z - bz) / 3,
        ]
        control2 = [
            bx + 2 * (x - bx) / 3,
            by + 2 * (y - by) / 3,
            bz + 2 * (z - bz) / 3,
        ]
        line = scene.curve([bx, by, bz], control1, control2, [x, y, z]).material(
            "#f1c40f"
        )
        link_lines.append(line)

    # Mettre à jour l'affichage des coordonnées
    coord_label.text = f"X: {x:.2f} m | Y: {y:.2f} m | Z: {z:.2f} m | Rz: {rz:.2f} rad"


def apply_jog():
    """Fonction appelée à intervalle régulier (timer) si un bouton est maintenu enfoncé"""
    axis = active_jog["axis"]
    delta = active_jog["delta"]

    if axis is None or delta == 0.0:
        return

    if axis in ["x", "y", "z"]:
        new_val = robot_state[axis] + delta
        if LIMITS[axis][0] <= new_val <= LIMITS[axis][1]:
            robot_state[axis] = new_val
    elif axis == "rz":
        robot_state["rz"] += delta

    update_robot_graphics()


def start_jog(axis: str, delta: float):
    active_jog["axis"] = axis
    active_jog["delta"] = delta


def stop_jog():
    active_jog["axis"] = None
    active_jog["delta"] = 0.0


def reset_robot():
    stop_jog()
    robot_state["x"] = 0.0
    robot_state["y"] = 0.0
    robot_state["z"] = 1.0
    robot_state["rz"] = 0.0
    ui.notify("Robot réinitialisé en position HOME", type="info")
    update_robot_graphics()


# --- DÉFINITION DES PAGES WEB ---


@ui.page("/")
def home_page():
    """Page d'accueil du site web"""
    with ui.column().classes(
        "w-full h-screen items-center justify-center gap-6 bg-grey-1"
    ):
        ui.markdown("# 🚀 Portail de Contrôle Industriel").classes("text-primary")
        ui.label(
            "Bienvenue sur l'interface de pilotage de cellules robotisées."
        ).classes("text-lg text-gray-600")

        with ui.card().classes("p-6 items-center gap-4 shadow-md"):
            ui.label("Cellule Robot Delta 4 Axes").classes(
                "font-bold text-xl text-gray-800"
            )
            ui.link(
                "Lancer la Simulation 3D →", "/robot"
            ).classes(  # Lien vers la page du robot
                "bg-blue-600 text-white font-bold py-2 px-4 rounded hover:bg-blue-700 no-underline"
            )


@ui.page("/robot")
def robot_page():
    """Page de simulation 3D du Robot Delta"""
    global end_effector, scene, coord_label

    # Barre de navigation supérieure pour retourner à l'accueil
    with ui.row().classes("w-full items-center justify-between bg-dark p-2 px-4"):
        ui.label("🤖 Simulation Live - Robot Delta 4 Axes").classes(
            "text-white font-bold text-lg"
        )
        ui.link("← Retour Accueil", "/").classes(
            "text-blue-300 hover:text-white no-underline font-semibold"
        )

    with ui.row().classes("w-full items-start gap-4 p-4"):

        # --- COLONNE DE GAUCHE : SCÈNE 3D ---
        with ui.card().classes("flex-grow bg-grey-2 p-0"):
            with ui.scene(width=800, height=600) as sc:
                scene = sc
                # Caméra positionnée pour voir le sol et le robot en hauteur
                sc.move_camera(
                    x=3.5,
                    y=-3.5,
                    z=2.0,
                    look_at_x=0,
                    look_at_y=0,
                    look_at_z=1.0,
                )

                # Structure fixe du Robot Delta (en haut à BASE_Z)
                with ui.scene.group().move(0, 0, BASE_Z):
                    ui.scene.cylinder(0.9, 0.9, 0.1).material(
                        "#7f8c8d"
                    ).rotate(1.5708, 0, 0)
                    for bx, by in [(0.6, 0.0), (-0.3, 0.52), (-0.3, -0.52)]:
                        ui.scene.cylinder(0.04, 0.04, BASE_Z).material(
                            "#bdc3c7"
                        ).rotate(1.5708, 0, 0).move(bx, by, -BASE_Z / 2)

                # Groupe de l'effecteur mobile
                end_effector = ui.scene.group()

        # --- COLONNE DE DROITE : TEACH PENDANT (CONTRÔLE) ---
        with ui.card().classes("w-80 p-4 flex flex-col gap-4"):
            ui.label("Teach Pendant (Jogging Continu)").classes(
                "text-lg font-bold text-primary"
            )

            coord_label = ui.label("").classes(
                "text-sm font-mono bg-dark text-white p-2 rounded"
            )

            step = (
                0.025  # Pas plus fin pour une transition fluide en continu
            )

            def create_jog_button(label, axis, delta, color_class):
                btn = ui.button(label).classes(f"{color_class} text-white flex-1")
                btn.on("mousedown", lambda: start_jog(axis, delta))
                btn.on("mouseup", lambda: stop_jog())
                btn.on("mouseleave", lambda: stop_jog())
                return btn

            # Contrôles Axe X
            ui.label("Axe X").classes("font-semibold text-gray-700 mb-[-10px]")
            with ui.row().classes("w-full justify-between"):
                create_jog_button("X -", "x", -step, "bg-red-500")
                create_jog_button("X +", "x", step, "bg-green-500")

            # Contrôles Axe Y
            ui.label("Axe Y").classes("font-semibold text-gray-700 mb-[-10px]")
            with ui.row().classes("w-full justify-between"):
                create_jog_button("Y -", "y", -step, "bg-red-500")
                create_jog_button("Y +", "y", step, "bg-green-500")

            # Contrôles Axe Z
            ui.label("Axe Z").classes("font-semibold text-gray-700 mb-[-10px]")
            with ui.row().classes("w-full justify-between"):
                create_jog_button("Z -", "z", -step, "bg-red-500")
                create_jog_button("Z +", "z", step, "bg-green-500")

            # Contrôles Axe Rz (Rotation)
            ui.label("Rotation 4e Axe (Rz)").classes(
                "font-semibold text-gray-700 mb-[-10px]"
            )
            with ui.row().classes("w-full justify-between"):
                create_jog_button("Rz -", "rz", -0.1, "bg-orange-500")
                create_jog_button("Rz +", "rz", 0.1, "bg-orange-500")

            ui.separator()

            ui.button("🏠 RESET HOME", on_click=reset_robot).classes(
                "w-full bg-blue-600 text-white font-bold"
            )

    # Initialisation de l'affichage de la page robot
    update_robot_graphics()
    ui.timer(0.03, apply_jog)


# Lancer l'application NiceGUI
ui.run(port=8080, title="Portail Robotique Multi-Pages")