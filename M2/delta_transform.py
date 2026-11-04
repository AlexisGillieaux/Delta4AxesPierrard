import importlib.util
from pathlib import Path

import numpy as np
from rigid_transform import RigidTransform


# Cinématique du robot à 3 bras : « DeltaCoord_fixed (1).py », dans ../Coords. Son nom
# (espace et parenthèses) interdit « import » : on charge le fichier par son chemin, ce qui
# marche quel que soit le dossier de lancement.
FICHIER_DELTACOORD = Path(__file__).resolve().parent.parent / "Coords" / "DeltaCoord_fixed (1).py"


def _charger_module(nom, chemin):
    """Charge un fichier Python par son chemin et retourne le module."""
    spec = importlib.util.spec_from_file_location(nom, chemin)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


d1 = _charger_module("DeltaCoord_fixed_v1", FICHIER_DELTACOORD)

# ============================================================================
#                      MODÈLE DU 4ÈME AXE (À ADAPTER)
# ============================================================================
# Le plateau d'un robot delta reste toujours parallèle à la base. La rotation rz du
# plateau (autour de Z) est donc indépendante de la position XYZ, et l'angle du moteur 4 vaut :
#
#     θ4 = RAPPORT_AXE4 * rz + OFFSET_AXE4
#
# Ce modèle est valable que le moteur 4 soit fixé sur la base (arbre télescopique à deux
# cardans) ou directement sur le plateau.

# Tours du moteur 4 pour un tour du plateau (réducteur, poulies...). 1.0 = entraînement direct.
RAPPORT_AXE4 = 1.0

# Angle du moteur 4 (degrés) quand le plateau est à rz = 0.
OFFSET_AXE4 = 0.0

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


def axe4_inverse(rz_deg):
    """Convertit la rotation du plateau rz (degrés) en angle du moteur 4 (degrés)."""
    return RAPPORT_AXE4 * np.asarray(rz_deg, dtype=float) + OFFSET_AXE4


def axe4_direct(theta4_deg):
    """Convertit l'angle du moteur 4 (degrés) en rotation du plateau rz (degrés)."""
    return (np.asarray(theta4_deg, dtype=float) - OFFSET_AXE4) / RAPPORT_AXE4


def DeltaInverse4(poses):
    """Cinématique inverse du robot delta 4 axes, SANS repères (pose de la bride dans la base).

    Args:
        poses (array-like): pose [x, y, z, rz] ou array de shape (n, 4)

    Returns:
        numpy.ndarray: array de shape (n, 4) avec [θ1, θ2, θ3, θ4] en degrés.
            Les angles des bras valent NaN si la position est hors de portée.
    """
    p = np.atleast_2d(np.asarray(poses, dtype=float))
    if p.ndim != 2 or p.shape[1] != 4:
        raise ValueError(f"Poses attendues de shape (n, 4), reçu {p.shape}")

    angles_bras = np.asarray(d1.DeltaInverse(p[:, :3])).reshape(-1, 3)
    theta4 = axe4_inverse(p[:, 3])
    return np.column_stack((angles_bras, theta4))


def DeltaForward4(angles):
    """Cinématique directe du robot delta 4 axes, SANS repères (pose de la bride dans la base).

    Args:
        angles (array-like): angles [θ1, θ2, θ3, θ4] ou array de shape (n, 4), en degrés

    Returns:
        numpy.ndarray: array de shape (n, 4) avec [x, y, z, rz] (mm et degrés)
    """
    q = np.atleast_2d(np.asarray(angles, dtype=float))
    if q.ndim != 2 or q.shape[1] != 4:
        raise ValueError(f"Angles attendus de shape (n, 4), reçu {q.shape}")

    xyz = np.asarray(d1.DeltaForward(q[:, :3])).reshape(-1, 3)
    rz = axe4_direct(q[:, 3])
    return np.column_stack((xyz, rz))


def _frame_ou_identite(frame) -> RigidTransform:
    """Le repère donné, ou la transformation identité si frame vaut None (repère inchangé)."""
    if frame is None:
        return RigidTransform.identity(1)
    if not isinstance(frame, RigidTransform):
        raise TypeError(
            f"Repère attendu : RigidTransform (tool_frame_from_pose, user_frame_from_pose, "
            f"calibrate_*) ou None, reçu {type(frame).__name__}"
        )
    return frame


def _hom(frame: RigidTransform) -> np.ndarray:
    """Matrice homogène (4, 4) d'un repère unique."""
    mat = _frame_ou_identite(frame).as_hom_mat()
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
    if not np.isfinite(pose).all():
        raise ValueError(f"Pose avec NaN ou infini : {pose}")
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
    if not np.isfinite(pose).all():
        raise ValueError(f"Pose avec NaN ou infini : {pose}")
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
    xyz_rz = DeltaForward4(angles)

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

    angles = DeltaInverse4(np.column_stack((xyz, rz)))
    angles[(inclinaison > tol_inclinaison_deg) | np.isnan(angles).any(axis=1)] = np.nan

    return angles[0] if reduced else angles


# ============================================================================
#      POSES [x, y, z, rz] : INTERFACE UTILISÉE PAR LES TRAJECTOIRES
# ============================================================================
# Une pose [x, y, z, rz] désigne :
#   - x, y, z : position du TCP (origine du repère outil) dans le repère utilisateur, en mm ;
#   - rz      : rotation de la plateforme autour de Z de la base (angle du 4ème axe), en degrés,
#               sans limite à ±180° (plusieurs tours possibles).
# Les repères outil et utilisateur valent l'identité s'ils ne sont pas donnés (None) : la pose
# est alors celle de la bride dans la base, comme avant l'introduction des repères.

def _rot_z(deg) -> np.ndarray:
    """Matrices de rotation autour de Z, shape (n, 3, 3), pour un array d'angles en degrés."""
    a = np.radians(np.atleast_1d(deg))
    rot = np.zeros((len(a), 3, 3))
    rot[:, 0, 0] = np.cos(a)
    rot[:, 0, 1] = -np.sin(a)
    rot[:, 1, 0] = np.sin(a)
    rot[:, 1, 1] = np.cos(a)
    rot[:, 2, 2] = 1.0
    return rot


def tcp_pose_from_poses4(poses4, tool_frame=None, user_frame=None) -> RigidTransform:
    """
    Construit la pose du TCP dans le repère utilisateur (RigidTransform) à partir de poses [x, y, z, rz].

    La bride est toujours horizontale et ne tourne que de rz autour de Z. L'orientation du TCP
    dans le repère utilisateur est donc entièrement imposée par rz :
        R_user_tcp = R_base_user^T @ Rz(rz) @ R_flange_tool
    (déduite de T_base_flange = T_base_user @ T_user_tcp @ T_flange_tool⁻¹ avec R_base_flange = Rz(rz)).

    Paramètres
    ----------
    poses4 : array-like de shape (4,) ou (n, 4)
        Poses [x, y, z, rz] (mm, mm, mm, degrés), x, y, z dans le repère utilisateur.
    tool_frame, user_frame : RigidTransform ou None
        Repères outil et utilisateur. None = identité.

    Retour
    ------
    RigidTransform
        Pose(s) du TCP dans le repère utilisateur, à passer à delta_inverse_with_frames().
    """
    p = np.asarray(poses4, dtype=float)
    reduced = p.ndim == 1
    p = np.atleast_2d(p)
    if p.ndim != 2 or p.shape[1] != 4:
        raise ValueError(f"Poses attendues de shape (4,) ou (n, 4), reçu {p.shape}")

    rot = _hom(_frame_ou_identite(user_frame))[:3, :3].T @ _rot_z(p[:, 3]) @ _hom(_frame_ou_identite(tool_frame))[:3, :3]
    hom = np.zeros((len(p), 4, 4))
    hom[:, :3, :3] = rot
    hom[:, :3, 3] = p[:, :3]
    hom[:, 3, 3] = 1.0
    return RigidTransform.from_homogeneous(hom[0] if reduced else hom)


def delta_inverse_from_poses4(poses4, tool_frame=None, user_frame=None,
                              tol_inclinaison_deg: float = TOL_INCLINAISON_DEG) -> np.ndarray:
    """
    MGI du robot 4 axes pour des poses [x, y, z, rz] exprimées dans le repère utilisateur.

    Passe par tcp_pose_from_poses4() puis delta_inverse_with_frames() : toute la chaîne de repères
    T_base_user @ T_user_tcp @ T_flange_tool⁻¹ est appliquée, même quand les repères valent l'identité.

    La matrice de rotation ne connaît rz que modulo 360° : l'angle θ4 est donc recalculé à partir
    du rz demandé, ce qui conserve les rotations de plus d'un demi-tour (ex. rz = 270°).

    Retour
    ------
    np.ndarray de shape (n, 4)
        Angles [t1, t2, t3, t4] en degrés ; NaN sur toute la ligne si la pose est inatteignable.
    """
    p = np.atleast_2d(np.asarray(poses4, dtype=float))
    tool, user = _frame_ou_identite(tool_frame), _frame_ou_identite(user_frame)
    angles = np.atleast_2d(delta_inverse_with_frames(tcp_pose_from_poses4(p, tool, user), tool, user,
                                                     tol_inclinaison_deg))
    angles[:, 3] = np.where(np.isnan(angles[:, 3]), np.nan, axe4_inverse(p[:, 3]))
    return angles


def delta_forward_to_poses4(angles_4, tool_frame=None, user_frame=None) -> np.ndarray:
    """
    MGD du robot 4 axes vers des poses [x, y, z, rz] exprimées dans le repère utilisateur.

    x, y, z : position du TCP dans le repère utilisateur (via delta_forward_with_frames) ;
    rz      : rotation de la plateforme déduite de θ4, sans repliement à ±180°.

    Retour
    ------
    np.ndarray de shape (n, 4) ; NaN en x, y, z si les angles sont impossibles.
    """
    q = np.atleast_2d(np.asarray(angles_4, dtype=float))
    tcp = delta_forward_with_frames(q, _frame_ou_identite(tool_frame), _frame_ou_identite(user_frame))
    return np.column_stack((tcp.translation, axe4_direct(q[:, 3])))


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
