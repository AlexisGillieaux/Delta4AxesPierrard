from nicegui import ui
from threading import Lock
import math


# ============================================================
#                    ÉTAT GLOBAL DU ROBOT
# ============================================================

# Cet état est PARTAGÉ entre tous les utilisateurs.
robot_state = {
    "x": 0.0,
    "y": 0.0,
    "z": 1.0,
    "rz": 0.0,
}

# Limites de déplacement
LIMITS = {
    "x": (-0.8, 0.8),
    "y": (-0.8, 0.8),
    "z": (0.3, 1.8),
    "rz": (-math.pi, math.pi),
}

# Hauteur du plateau supérieur
BASE_Z = 2.0

# Protection de l'état global
robot_lock = Lock()

# Version de l'état robot.
# Chaque modification augmente cette valeur.
robot_version = 0


# ============================================================
#                    FONCTIONS UTILITAIRES
# ============================================================

def clamp(value, minimum, maximum):
    """Limite une valeur entre minimum et maximum."""
    return max(minimum, min(maximum, value))


def get_robot_state():
    """
    Retourne une copie de l'état du robot
    ainsi que son numéro de version.
    """
    with robot_lock:
        return robot_state.copy(), robot_version


# ============================================================
#                    MOUVEMENT DU ROBOT
# ============================================================

def move_robot(axis, delta):
    """
    Modifie l'état global partagé du robot.

    Si l'utilisateur A déplace le robot,
    l'utilisateur B verra également le déplacement.
    """

    global robot_version

    with robot_lock:

        # -------------------------------
        # AXE X
        # -------------------------------

        if axis == "x":

            robot_state["x"] = clamp(
                robot_state["x"] + delta,
                LIMITS["x"][0],
                LIMITS["x"][1],
            )

        # -------------------------------
        # AXE Y
        # -------------------------------

        elif axis == "y":

            robot_state["y"] = clamp(
                robot_state["y"] + delta,
                LIMITS["y"][0],
                LIMITS["y"][1],
            )

        # -------------------------------
        # AXE Z
        # -------------------------------

        elif axis == "z":

            robot_state["z"] = clamp(
                robot_state["z"] + delta,
                LIMITS["z"][0],
                LIMITS["z"][1],
            )

        # -------------------------------
        # ROTATION RZ
        # -------------------------------

        elif axis == "rz":

            robot_state["rz"] += delta

            # Ramener l'angle dans [-pi ; +pi]
            if robot_state["rz"] > math.pi:
                robot_state["rz"] -= 2 * math.pi

            if robot_state["rz"] < -math.pi:
                robot_state["rz"] += 2 * math.pi

        # Nouvelle version de l'état partagé
        robot_version += 1


# ============================================================
#                    RESET ROBOT
# ============================================================

def reset_robot():

    global robot_version

    with robot_lock:

        robot_state["x"] = 0.0
        robot_state["y"] = 0.0
        robot_state["z"] = 1.0
        robot_state["rz"] = 0.0

        robot_version += 1


# ============================================================
#                    PAGE PRINCIPALE
# ============================================================

@ui.page("/")
def robot_page():

    # ========================================================
    # OBJETS PROPRES À CET UTILISATEUR
    # ========================================================

    scene = None

    end_effector = None

    center_point = None
    platform = None
    rz_axis = None

    coord_label = None

    # Liste des trois bras jaunes
    link_lines = []

    # Jog de cet utilisateur
    active_jog = {
        "axis": None,
        "delta": 0.0,
    }

    # Version du robot connue par cette fenêtre
    last_version = -1

    # Pas de déplacement
    STEP = 0.025

    # Pas de rotation
    RZ_STEP = 0.1


    # ========================================================
    #                    MISE À JOUR 3D
    # ========================================================

    def update_robot_graphics(force=False):

        nonlocal last_version

        if scene is None:
            return

        # ----------------------------------------------------
        # Lire l'état global
        # ----------------------------------------------------

        state, version = get_robot_state()

        # Si rien n'a changé, inutile de redessiner
        if not force and version == last_version:
            return

        x = state["x"]
        y = state["y"]
        z = state["z"]
        rz = state["rz"]


        # ====================================================
        # POSITION DE L'EFFECTEUR
        # ====================================================
        #
        # TRÈS IMPORTANT :
        #
        # center_point
        # platform
        # rz_axis
        #
        # sont tous des enfants de end_effector.
        #
        # Cette commande déplace donc toute la partie mobile.
        #
        # ====================================================

        end_effector.move(x, y, z)


        # ====================================================
        # ROTATION RZ
        # ====================================================

        if rz_axis is not None:

            rz_axis.rotate(
                math.pi / 2,
                0,
                rz
            )


        # ====================================================
        # SUPPRIMER LES ANCIENS BRAS JAUNES
        # ====================================================

        for line in link_lines:

            try:
                line.delete()
            except Exception:
                pass

        link_lines.clear()


        # ====================================================
        # POINTS D'ATTACHE SUPÉRIEURS
        # ====================================================

        base_points = [
            (0.6, 0.0, BASE_Z),
            (-0.3, 0.52, BASE_Z),
            (-0.3, -0.52, BASE_Z),
        ]


        # ====================================================
        # CRÉATION DES 3 BRAS JAUNES
        # ====================================================

        for bx, by, bz in base_points:

            # -----------------------------------------------
            # Premier point de contrôle
            # -----------------------------------------------

            c1x = bx + (x - bx) * 0.33
            c1y = by + (y - by) * 0.33
            c1z = bz + (z - bz) * 0.33


            # -----------------------------------------------
            # Deuxième point de contrôle
            # -----------------------------------------------

            c2x = bx + (x - bx) * 0.66
            c2y = by + (y - by) * 0.66
            c2z = bz + (z - bz) * 0.66


            # -----------------------------------------------
            # COURBE
            #
            # NiceGUI attend :
            #
            # curve(
            #     start,
            #     control1,
            #     control2,
            #     end
            # )
            # -----------------------------------------------

            line = scene.curve(
                [bx, by, bz],
                [c1x, c1y, c1z],
                [c2x, c2y, c2z],
                [x, y, z],
            ).material(
                "#f1c40f"
            )

            link_lines.append(line)


        # ====================================================
        # MISE À JOUR DES COORDONNÉES
        # ====================================================

        if coord_label is not None:

            coord_label.text = (
                f"X = {x:+.3f}    "
                f"Y = {y:+.3f}    "
                f"Z = {z:+.3f}    "
                f"Rz = {math.degrees(rz):+.1f}°"
            )


        # Mémoriser la version affichée
        last_version = version


    # ========================================================
    #                    JOG
    # ========================================================

    def start_jog(axis, delta):

        active_jog["axis"] = axis
        active_jog["delta"] = delta


    def stop_jog():

        active_jog["axis"] = None
        active_jog["delta"] = 0.0


    def apply_jog():

        axis = active_jog["axis"]
        delta = active_jog["delta"]

        if axis is None:
            return

        # Modifier le robot global
        move_robot(axis, delta)

        # Mise à jour immédiate de cette fenêtre
        update_robot_graphics(force=True)


    # ========================================================
    #             SYNCHRONISATION MULTI-UTILISATEURS
    # ========================================================

    def synchronize_robot():

        nonlocal last_version

        _, version = get_robot_state()

        # Un autre utilisateur a modifié le robot
        if version != last_version:

            update_robot_graphics(
                force=True
            )


    # ========================================================
    #                    INTERFACE
    # ========================================================

    with ui.column().classes(
        "w-full h-screen p-4 bg-gray-100"
    ):

        # ====================================================
        # TITRE
        # ====================================================

        ui.label(
            "Simulation Delta Robot"
        ).classes(
            "text-2xl font-bold"
        )


        # ====================================================
        # ZONE PRINCIPALE
        # ====================================================

        with ui.row().classes(
            "w-full flex-1 items-stretch"
        ):


            # =================================================
            #                    SCÈNE 3D
            # =================================================

            with ui.card().classes(
                "flex-1 h-full"
            ):

                with ui.scene(
                    width=900,
                    height=650,
                    grid=(5, 20),
                    background_color="#eeeeee",
                ) as scene:


                    # =========================================
                    # CAMÉRA
                    # =========================================

                    # Chaque navigateur possède sa propre scène
                    # donc sa propre caméra.
                    #
                    # Le robot, lui, est partagé via robot_state.

                    scene.move_camera(
                        x=3.5,
                        y=-3.5,
                        z=2.0,
                        look_at_x=0,
                        look_at_y=0,
                        look_at_z=1.0,
                        duration=0,
                    )


                    # =========================================
                    # PLATEAU SUPÉRIEUR
                    # =========================================

                    with scene.group().move(
                        0,
                        0,
                        BASE_Z
                    ):

                        scene.cylinder(
                            0.9,
                            0.9,
                            0.1
                        ).material(
                            "#7f8c8d"
                        ).rotate(
                            math.pi / 2,
                            0,
                            0
                        )


                        # =====================================
                        # COLONNES VERTICALES
                        # =====================================

                        base_points = [
                            (0.6, 0.0),
                            (-0.3, 0.52),
                            (-0.3, -0.52),
                        ]


                        for bx, by in base_points:

                            scene.cylinder(
                                0.04,
                                0.04,
                                BASE_Z
                            ).material(
                                "#bdc3c7"
                            ).rotate(
                                math.pi / 2,
                                0,
                                0
                            ).move(
                                bx,
                                by,
                                -BASE_Z / 2
                            )


                    # =========================================
                    # GROUPE DE LA PARTIE MOBILE
                    # =========================================
                    #
                    # C'EST LE POINT IMPORTANT.
                    #
                    # Tout ce qui est créé ici appartient
                    # au groupe end_effector.
                    #
                    # Lorsque nous faisons :
                    #
                    #     end_effector.move(x, y, z)
                    #
                    # tout le contenu suit.
                    #
                    # =========================================

                    with scene.group() as end_effector:


                        # =====================================
                        # POINT CENTRAL VERT
                        # =====================================
                        #
                        # Ce point représente l'origine
                        # exacte de l'effecteur.
                        #

                        center_point = scene.sphere(
                            0.06
                        ).material(
                            "#00ff00"
                        )


                        # =====================================
                        # PLATEFORME NOIRE
                        # =====================================

                        platform = scene.box(
                            0.4,
                            0.1,
                            0.4
                        ).material(
                            "#2c3e50"
                        ).rotate(
                            math.pi / 2,
                            0,
                            0
                        )


                        # =====================================
                        # AXE ROUGE DE ROTATION RZ
                        # =====================================

                        rz_axis = scene.cylinder(
                            0.05,
                            0.05,
                            0.5
                        ).material(
                            "#e74c3c"
                        ).rotate(
                            math.pi / 2,
                            0,
                            0
                        )


                    # =========================================
                    # AFFICHAGE INITIAL
                    # =========================================

                    update_robot_graphics(
                        force=True
                    )


            # =================================================
            #                 PANNEAU DE COMMANDE
            # =================================================

            with ui.card().classes(
                "w-72 p-4"
            ):

                ui.label(
                    "Commande"
                ).classes(
                    "text-xl font-bold"
                )


                # =============================================
                # COORDONNÉES
                # =============================================

                coord_label = ui.label(
                    "X = +0.000    "
                    "Y = +0.000    "
                    "Z = +1.000    "
                    "Rz = +0.0°"
                ).classes(
                    "font-mono text-sm"
                )


                ui.separator()


                # =============================================
                # AXE X
                # =============================================

                ui.label(
                    "Axe X"
                ).classes(
                    "font-bold"
                )

                with ui.row():

                    minus_x = ui.button(
                        "X −"
                    ).props(
                        "color=primary"
                    )

                    plus_x = ui.button(
                        "X +"
                    ).props(
                        "color=primary"
                    )


                # =============================================
                # AXE Y
                # =============================================

                ui.label(
                    "Axe Y"
                ).classes(
                    "font-bold"
                )

                with ui.row():

                    minus_y = ui.button(
                        "Y −"
                    ).props(
                        "color=primary"
                    )

                    plus_y = ui.button(
                        "Y +"
                    ).props(
                        "color=primary"
                    )


                # =============================================
                # AXE Z
                # =============================================

                ui.label(
                    "Axe Z"
                ).classes(
                    "font-bold"
                )

                with ui.row():

                    minus_z = ui.button(
                        "Z −"
                    ).props(
                        "color=primary"
                    )

                    plus_z = ui.button(
                        "Z +"
                    ).props(
                        "color=primary"
                    )


                # =============================================
                # ROTATION RZ
                # =============================================

                ui.label(
                    "Rotation Rz"
                ).classes(
                    "font-bold"
                )

                with ui.row():

                    minus_rz = ui.button(
                        "Rz −"
                    ).props(
                        "color=primary"
                    )

                    plus_rz = ui.button(
                        "Rz +"
                    ).props(
                        "color=primary"
                    )


                ui.separator()


                # =============================================
                # RESET
                # =============================================

                reset_button = ui.button(
                    "RESET"
                ).classes(
                    "w-full"
                ).props(
                    "color=negative"
                )


                def do_reset():

                    reset_robot()

                    update_robot_graphics(
                        force=True
                    )


                reset_button.on(
                    "click",
                    do_reset
                )


                ui.separator()


                ui.label(
                    "Maintenir un bouton pour déplacer"
                ).classes(
                    "text-sm text-gray-600"
                )


        # ====================================================
        #                    TIMERS
        # ====================================================

        # Mouvement continu pendant que le bouton est maintenu
        ui.timer(
            0.03,
            apply_jog
        )

        # Synchronisation entre utilisateurs
        ui.timer(
            0.03,
            synchronize_robot
        )


    # ========================================================
    #             CONNEXION DES BOUTONS DE JOG
    # ========================================================

    def connect_button(button, axis, delta):

        # ----------------------------------------------------
        # Souris
        # ----------------------------------------------------

        button.on(
            "mousedown",
            lambda: start_jog(axis, delta)
        )

        button.on(
            "mouseup",
            stop_jog
        )

        button.on(
            "mouseleave",
            stop_jog
        )


        # ----------------------------------------------------
        # Écran tactile
        # ----------------------------------------------------

        button.on(
            "touchstart",
            lambda: start_jog(axis, delta)
        )

        button.on(
            "touchend",
            stop_jog
        )

        button.on(
            "touchcancel",
            stop_jog
        )


    # ========================================================
    #                     AXE X
    # ========================================================

    connect_button(
        minus_x,
        "x",
        -STEP
    )

    connect_button(
        plus_x,
        "x",
        STEP
    )


    # ========================================================
    #                     AXE Y
    # ========================================================

    connect_button(
        minus_y,
        "y",
        -STEP
    )

    connect_button(
        plus_y,
        "y",
        STEP
    )


    # ========================================================
    #                     AXE Z
    # ========================================================

    connect_button(
        minus_z,
        "z",
        -STEP
    )

    connect_button(
        plus_z,
        "z",
        STEP
    )


    # ========================================================
    #                     ROTATION RZ
    # ========================================================

    connect_button(
        minus_rz,
        "rz",
        -RZ_STEP
    )

    connect_button(
        plus_rz,
        "rz",
        RZ_STEP
    )


# ============================================================
#                       LANCEMENT
# ============================================================

ui.run(
    title="Delta Robot",
    reload=False,
    port=8080
)