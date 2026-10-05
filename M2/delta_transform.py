import numpy as np
from rigid_transform import RigidTransform


def tool_frame_from_pose(pose) -> RigidTransform:
    """
    Construit le repère outil T_flange_tool à partir d'une pose connue.

    Utilisé quand la géométrie de l'outil est connue (dessin, datasheet).
    Pour une calibration expérimentale, utiliser calibrate_tool_frame().

    Paramètres
    ----------
    pose : array-like de shape (6,) [x, y, z, rx, ry, rz]
        Pose du TCP exprimée dans le repère bride (en mm et degrés, convention xyz).

    Retour
    ------
    RigidTransform
        Transformation T_flange_tool.
    """
    pass


def user_frame_from_pose(pose) -> RigidTransform:
    """
    Construit le repère utilisateur T_base_user à partir d'une pose connue.

    Utilisé quand la position du repère de travail est connue (ex. mesurée au pied à coulisse).
    Pour une calibration expérimentale, utiliser calibrate_user_frame().

    Paramètres
    ----------
    pose : array-like de shape (6,) [x, y, z, rx, ry, rz]
        Pose du repère utilisateur exprimée dans le repère base (en mm et degrés, convention xyz).

    Retour
    ------
    RigidTransform
        Transformation T_base_user.
    """
    pass


def delta_forward_with_frames(angles_4, tool_frame: RigidTransform, user_frame: RigidTransform) -> RigidTransform:
    """
    MGD avec changements de repère : convertit des angles moteurs en pose TCP exprimée dans le repère utilisateur.

    Chaîne appliquée :
        T_user_tcp = T_base_user⁻¹ @ T_base_flange @ T_flange_tool

    où T_base_flange est obtenu via DeltaForward (position xyz) + rotation du 4ème axe.

    Paramètres
    ----------
    angles_4 : array-like de shape (4,) ou (n, 4)
        Angles [t1, t2, t3, t4] des trois moteurs + 4ème axe (en degrés).
    tool_frame : RigidTransform
        Transformation T_flange_tool. Obtenue via tool_frame_from_pose() ou calibrate_tool_frame().
    user_frame : RigidTransform
        Transformation T_base_user. Obtenue via user_frame_from_pose() ou calibrate_user_frame().

    Retour
    ------
    RigidTransform
        Pose du TCP exprimée dans le repère utilisateur.
    """
    pass


def delta_inverse_with_frames(pose: RigidTransform, tool_frame: RigidTransform, user_frame: RigidTransform) -> np.ndarray:
    """
    MGI avec changements de repère : convertit une pose TCP exprimée dans le repère utilisateur en angles moteurs.

    Chaîne appliquée :
        T_base_flange = T_base_user @ T_user_tcp @ T_flange_tool⁻¹

    La position xyz de T_base_flange est passée à DeltaInverse ; la rotation donne l'angle du 4ème axe.

    Paramètres
    ----------
    pose : RigidTransform
        Pose cible du TCP exprimée dans le repère utilisateur.
    tool_frame : RigidTransform
        Transformation T_flange_tool. Obtenue via tool_frame_from_pose() ou calibrate_tool_frame().
    user_frame : RigidTransform
        Transformation T_base_user. Obtenue via user_frame_from_pose() ou calibrate_user_frame().

    Retour
    ------
    np.ndarray de shape (4,) ou (n, 4)
        Angles [t1, t2, t3, t4] des moteurs (en degrés).
        Retourne NaN si la pose est hors de l'espace de travail.
    """
    pass


def calibrate_tool_frame(flange_poses: np.ndarray) -> RigidTransform:
    """
    Calibre le repère outil à partir de plusieurs poses de la bride pointant vers le même TCP.

    Méthode (point fixe / multi-orientation) :
        On amène le TCP en contact avec un point fixe de l'espace n fois,
        avec des orientations de bride différentes à chaque prise.
        Le TCP inconnu p_tool (exprimé dans le repère bride) vérifie :
            T_base_flange_i @ p_tool = constante pour tout i
        On résout le système linéaire aux moindres carrés :
            (R_i - R_j) @ p_tool = t_j - t_i  pour toutes les paires (i, j)
        La translation de T_flange_tool est p_tool ; la rotation est identité
        (le TCP ne porte pas d'orientation propre dans cette méthode).

    Paramètres
    ----------
    flange_poses : np.ndarray de shape (n, 6) avec n >= 4
        Poses de la bride [x, y, z, rx, ry, rz] dans le repère base,
        toutes enregistrées avec le TCP posé sur le même point fixe.

    Retour
    ------
    RigidTransform
        Transformation T_flange_tool calibrée.
    """
    pass


def calibrate_user_frame(base_points: np.ndarray) -> RigidTransform:
    """
    Calibre le repère utilisateur par la méthode 3-points.

    Procédure : amener le TCP sur 3 points caractéristiques du repère utilisateur
    et enregistrer la position du TCP dans le repère base à chaque fois :
        base_points[0] : origine du repère utilisateur
        base_points[1] : un point sur l'axe X (à distance quelconque)
        base_points[2] : un point dans le plan XY (hors de l'axe X)

    La transformation T_base_user est reconstruite par orthonormalisation :
        X = (base_points[1] - base_points[0]) normalisé
        Z = X × (base_points[2] - base_points[0]) normalisé
        Y = Z × X

    Paramètres
    ----------
    base_points : np.ndarray de shape (3, 3)
        Coordonnées [x, y, z] des 3 points enregistrés dans le repère base.

    Retour
    ------
    RigidTransform
        Transformation T_base_user calibrée.
    """
    pass
