import importlib.util
from pathlib import Path

import numpy as np
from rigid_transform import RigidTransform


# Cinématique du robot delta 4 axes (DeltaInverse4 / DeltaForward4 : [x, y, z, rz] <-> [θ1..θ4]).
# Le fichier est dans ../Coords ; on le charge par son chemin, ce qui marche quel que soit
# le dossier de lancement.
FICHIER_DELTA4 = Path(__file__).resolve().parent.parent / "Coords" / "trajectoires4Axes.py"


def _charger_module(nom, chemin):
    """Charge un fichier Python par son chemin et retourne le module."""
    spec = importlib.util.spec_from_file_location(nom, chemin)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


delta4 = _charger_module("trajectoires4Axes_delta", FICHIER_DELTA4)

# Inclinaison maximale tolérée (degrés) entre la bride demandée et l'horizontale.
# La plateforme d'un robot delta reste toujours parallèle à la base : seule la rotation
# rz autour de Z est réalisable. Une pose dont la bride est plus inclinée que cette valeur
# est jugée inatteignable (NaN) ; en dessous, l'inclinaison résiduelle est ignorée
# (ex. repère utilisateur calibré à la main, jamais parfaitement horizontal).
TOL_INCLINAISON_DEG = 1.0

# Calibration outil : rapport minimal entre la plus petite et la plus grande valeur
# singulière du système. En dessous, la rotation de la bride n'est pas assez variée
# pour que le TCP soit observable.
CONDITIONNEMENT_MIN = 1e-2


def _hom(frame: RigidTransform) -> np.ndarray:
    """Matrice homogène (4, 4) d'un repère unique."""
    mat = frame.as_hom_mat()
    if mat.ndim == 3:
        if mat.shape[0] != 1:
            raise ValueError("Un repère unique est attendu (une seule matrice homogène)")
        mat = mat[0]
    return mat


def _inv_rigide(mat: np.ndarray) -> np.ndarray:
    """Inverse d'une ou plusieurs matrices homogènes, shape (..., 4, 4)."""
    inv = np.zeros_like(mat)
    rot_t = np.swapaxes(mat[..., :3, :3], -1, -2)
    inv[..., :3, :3] = rot_t
    inv[..., :3, 3] = -np.einsum("...ij,...j->...i", rot_t, mat[..., :3, 3])
    inv[..., 3, 3] = 1.0
    return inv


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
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (6,):
        raise ValueError(f"Pose attendue de shape (6,), reçu {pose.shape}")
    return RigidTransform.from_pose(pose, seq='xyz', degrees=True)


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
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (6,):
        raise ValueError(f"Pose attendue de shape (6,), reçu {pose.shape}")
    return RigidTransform.from_pose(pose, seq='xyz', degrees=True)


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
        Pour des angles impossibles (les barres ne se ferment pas), la translation vaut NaN
        et la rotation est l'identité.
    """
    angles = np.asarray(angles_4, dtype=float)
    reduced = angles.ndim == 1

    # [x, y, z, rz] de la bride dans la base (NaN si les bras sont impossibles)
    xyz_rz = delta4.DeltaForward4(angles)

    # Le plateau reste parallèle à la base : la bride n'a qu'une rotation rz autour de Z
    rz = np.radians(xyz_rz[:, 3])
    t_base_flange = np.zeros((len(xyz_rz), 4, 4))
    t_base_flange[:, 0, 0] = np.cos(rz)
    t_base_flange[:, 0, 1] = -np.sin(rz)
    t_base_flange[:, 1, 0] = np.sin(rz)
    t_base_flange[:, 1, 1] = np.cos(rz)
    t_base_flange[:, 2, 2] = 1.0
    t_base_flange[:, :3, 3] = xyz_rz[:, :3]
    t_base_flange[:, 3, 3] = 1.0

    t_user_tcp = _inv_rigide(_hom(user_frame)) @ t_base_flange @ _hom(tool_frame)

    # Angles impossibles : RigidTransform ne sait pas construire une rotation NaN. On garde une
    # translation NaN et une rotation identité, pour que les autres poses du lot restent utilisables.
    invalides = np.isnan(t_user_tcp).any(axis=(1, 2))
    t_user_tcp[invalides, :3, :3] = np.eye(3)

    return RigidTransform.from_homogeneous(t_user_tcp[0] if reduced else t_user_tcp)


def delta_inverse_with_frames(pose: RigidTransform, tool_frame: RigidTransform, user_frame: RigidTransform,
                              tol_inclinaison_deg: float = TOL_INCLINAISON_DEG) -> np.ndarray:
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
    tol_inclinaison_deg : float
        Inclinaison maximale (degrés) tolérée entre la bride et l'horizontale (voir TOL_INCLINAISON_DEG).

    Retour
    ------
    np.ndarray de shape (4,) ou (n, 4)
        Angles [t1, t2, t3, t4] des moteurs (en degrés).
        Retourne NaN si la pose est hors de l'espace de travail, ou si l'orientation
        demandée n'est pas réalisable : la plateforme du delta reste parallèle à la base,
        seule la rotation autour de Z (4ème axe) est possible.
    """
    t_user_tcp = pose.as_hom_mat()
    reduced = t_user_tcp.ndim == 2
    if reduced:
        t_user_tcp = t_user_tcp[None]

    t_base_flange = _hom(user_frame) @ t_user_tcp @ _inv_rigide(_hom(tool_frame))

    rot = t_base_flange[:, :3, :3]
    xyz = t_base_flange[:, :3, 3]

    # Inclinaison de l'axe Z de la bride par rapport à la verticale de la base
    inclinaison = np.degrees(np.arctan2(np.hypot(rot[:, 0, 2], rot[:, 1, 2]), rot[:, 2, 2]))
    rz = np.degrees(np.arctan2(rot[:, 1, 0], rot[:, 0, 0]))

    angles = delta4.DeltaInverse4(np.column_stack((xyz, rz)))
    angles[(inclinaison > tol_inclinaison_deg) | np.isnan(angles).any(axis=1)] = np.nan

    return angles[0] if reduced else angles


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

    Limite sur un robot delta : la bride ne peut tourner qu'autour de Z. Les R_i sont
    alors toutes des rotations autour de Z et la composante z de p_tool n'est pas
    observable (seuls x et y le sont). Dans ce cas la fonction lève ValueError ;
    la longueur de l'outil doit être donnée avec tool_frame_from_pose().

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
    flange_poses = np.asarray(flange_poses, dtype=float)
    if flange_poses.ndim != 2 or flange_poses.shape[1] != 6 or flange_poses.shape[0] < 4:
        raise ValueError(f"flange_poses doit avoir la shape (n, 6) avec n >= 4, reçu {flange_poses.shape}")

    hom = RigidTransform.from_pose(flange_poses, seq='xyz', degrees=True).as_hom_mat()
    rot = hom[:, :3, :3]
    trans = hom[:, :3, 3]

    # Une équation (3 lignes) par paire i < j : (R_i - R_j) p = t_j - t_i
    i, j = np.triu_indices(len(hom), k=1)
    a = (rot[i] - rot[j]).reshape(-1, 3)
    b = (trans[j] - trans[i]).reshape(-1)

    valeurs_singulieres = np.linalg.svd(a, compute_uv=False)
    if valeurs_singulieres[-1] < CONDITIONNEMENT_MIN * valeurs_singulieres[0]:
        raise ValueError(
            "Orientations de la bride insuffisamment variées : le TCP n'est pas observable "
            "sur tous ses axes (un robot delta ne tourne qu'autour de Z, donc le décalage en z "
            "de l'outil est inobservable). Utiliser tool_frame_from_pose() pour la longueur de l'outil."
        )

    p_tool, *_ = np.linalg.lstsq(a, b, rcond=None)
    return RigidTransform.from_pose(np.concatenate((p_tool, np.zeros(3))), seq='xyz', degrees=True)


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
    base_points = np.asarray(base_points, dtype=float)
    if base_points.shape != (3, 3):
        raise ValueError(f"base_points doit avoir la shape (3, 3), reçu {base_points.shape}")

    origine = base_points[0]
    vec_x = base_points[1] - origine
    vec_plan = base_points[2] - origine

    norme_x = np.linalg.norm(vec_x)
    if norme_x == 0:
        raise ValueError("Les points 0 et 1 sont confondus : l'axe X est indéfini")
    x_axe = vec_x / norme_x

    normale = np.cross(x_axe, vec_plan)
    norme_z = np.linalg.norm(normale)
    if norme_z < 1e-9 * max(np.linalg.norm(vec_plan), 1.0):
        raise ValueError("Les 3 points sont alignés : le plan XY est indéfini")
    z_axe = normale / norme_z
    y_axe = np.cross(z_axe, x_axe)

    hom = np.eye(4)
    hom[:3, :3] = np.column_stack((x_axe, y_axe, z_axe))
    hom[:3, 3] = origine
    return RigidTransform.from_homogeneous(hom)
