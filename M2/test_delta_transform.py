"""
Tests de delta_transform.py : repères utilisateur, repère outil (y compris excentré
et incliné), mouvements linéaires du TCP, calibrations.

Lancer : python test_delta_transform.py   (ou pytest test_delta_transform.py)

Convention des scénarios
------------------------
- Le TCP est l'origine (0, 0, 0) du repère outil. Les trajectoires sont décrites par
  la pose du TCP dans le repère utilisateur : c'est elle qui doit être linéaire.
- La bride d'un robot delta reste parallèle à la base : seule sa rotation rz est libre.
  L'orientation cible du TCP dans le repère utilisateur est donc
      R_tcp = R_base_user^T @ Rz(rz) @ R_flange_tool
  (rz = rotation de la bride, c'est-à-dire celle du 4ème axe).
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

import delta_transform as dt
from rigid_transform import RigidTransform

RACINE_PROJET = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RACINE_PROJET / "Coords"))
import trajectoires4Axes as tr      # noqa: E402  (trajectoires qui utilisent delta_transform)

# Crayon pris de biais : TCP à 30 mm de l'axe de la bride, 80 mm plus bas, incliné de 45° autour de Y.
OUTIL_CRAYON = [30, 0, -80, 0, 45, 0]
OUTIL_CENTRE = [0, 0, -100, 0, 0, 0]
OUTIL_FORT_DECALAGE = [-40, 25, -60, 20, -30, 70]

# Table 400 mm sous la base, tournée de 15° autour de Z.
USER_TABLE = [20, -30, -400, 0, 0, 15]
# Zone surélevée sur la table, tournée dans l'autre sens.
USER_ZONE = [60, 40, -390, 0, 0, -35]
# Plan de travail incliné de 10° autour de X.
USER_INCLINE = [0, 0, -400, 10, 0, 0]

IDENT = RigidTransform.identity(1)

TOL_MM = 1e-6


# ============================================================================
#                      OUTILS DE TEST
# ============================================================================

def _mat(frame):
    """Matrice homogène (4, 4) d'un repère unique."""
    return np.asarray(frame.as_hom_mat()).reshape(4, 4)


def _rot_z(deg):
    """Matrices de rotation autour de Z, shape (n, 3, 3), pour un array d'angles en degrés."""
    a = np.radians(np.atleast_1d(deg))
    rot = np.zeros((len(a), 3, 3))
    rot[:, 0, 0] = np.cos(a)
    rot[:, 0, 1] = -np.sin(a)
    rot[:, 1, 0] = np.sin(a)
    rot[:, 1, 1] = np.cos(a)
    rot[:, 2, 2] = 1.0
    return rot


def _poses_cible(user, tool, positions, rz_deg):
    """Poses du TCP dans le repère utilisateur pour une bride horizontale tournée de rz."""
    rot = _mat(user)[:3, :3].T @ _rot_z(rz_deg) @ _mat(tool)[:3, :3]
    hom = np.zeros((len(positions), 4, 4))
    hom[:, :3, :3] = rot
    hom[:, :3, 3] = positions
    hom[:, 3, 3] = 1.0
    return RigidTransform.from_homogeneous(hom)


def _ecart_a_la_droite(points, a, b):
    """Distance (mm) de chaque point à la droite (a, b)."""
    d = (b - a) / np.linalg.norm(b - a)
    v = points - a
    return np.linalg.norm(v - np.outer(v @ d, d), axis=1)


def _scenario_ligne(tool, user, p0, p1, rz0=0.0, rz1=0.0, n=101):
    """Trajectoire du TCP en ligne droite dans le repère utilisateur, résolue par MGI puis relue par MGD.

    La trajectoire « naïve » ignore l'outil : la bride suit la ligne au lieu du TCP.
    """
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    lam = np.linspace(0.0, 1.0, n)
    positions = p0 + lam[:, None] * (p1 - p0)
    rz = rz0 + lam * (rz1 - rz0)

    cible = _poses_cible(user, tool, positions, rz)
    angles = dt.delta_inverse_with_frames(cible, tool, user)
    retour = dt.delta_forward_with_frames(angles, tool, user)
    bride = dt.delta_forward_with_frames(angles, IDENT, user).translation

    hom_user = _mat(user)
    positions_base = positions @ hom_user[:3, :3].T + hom_user[:3, 3]
    angles_naif = dt.DeltaInverse4(np.column_stack((positions_base, rz)))
    tcp_naif = dt.delta_forward_with_frames(angles_naif, tool, user).translation

    return dict(positions=positions, rz=rz, cible=cible, angles=angles, retour=retour,
                bride=bride, ecart_naif=np.linalg.norm(tcp_naif - positions, axis=1))


def _verifier_ligne(sc, p0, p1, nom):
    """Vérifie que le TCP suit exactement la ligne demandée, à pas régulier, avec la bonne orientation."""
    assert not np.isnan(sc["angles"]).any(), f"{nom} : pose inatteignable sur la trajectoire"
    assert (np.abs(sc["angles"][:, :3]) <= dt.d1.ANGLE_MAX_DEG).all(), f"{nom} : moteur hors limites"

    tcp = sc["retour"].translation
    ecart_ligne = _ecart_a_la_droite(tcp, np.asarray(p0, float), np.asarray(p1, float)).max()
    ecart_pose = np.linalg.norm(tcp - sc["positions"], axis=1).max()
    pas = np.linalg.norm(np.diff(tcp, axis=0), axis=1)
    ecart_orient = np.abs(sc["retour"].as_hom_mat() - sc["cible"].as_hom_mat()).max()

    assert ecart_ligne < TOL_MM, f"{nom} : le TCP quitte la droite de {ecart_ligne:.2e} mm"
    assert ecart_pose < TOL_MM, f"{nom} : le TCP s'écarte de la pose demandée de {ecart_pose:.2e} mm"
    assert pas.max() - pas.min() < TOL_MM, f"{nom} : pas du TCP irrégulier"
    assert ecart_orient < 1e-8, f"{nom} : orientation du TCP différente de celle demandée"
    return ecart_ligne


# ============================================================================
#                      TESTS
# ============================================================================

def test_reperes_identite():
    """Avec deux repères identité, le résultat est celui de la cinématique du robot sans repères."""
    q = np.array([[20.0, 25.0, 30.0, 40.0], [5.0, 10.0, 35.0, -120.0]])
    xyz_rz = dt.DeltaForward4(q)

    pose = dt.delta_forward_with_frames(q, IDENT, IDENT).as_pose()
    assert np.allclose(pose[:, :3], xyz_rz[:, :3]) and np.allclose(pose[:, 5], xyz_rz[:, 3])
    assert np.allclose(pose[:, 3:5], 0.0), "la bride ne doit avoir qu'une rotation autour de Z"

    poses = RigidTransform.from_pose(np.column_stack((xyz_rz[:, :3], np.zeros((2, 2)), xyz_rz[:, 3])))
    assert np.allclose(dt.delta_inverse_with_frames(poses, IDENT, IDENT), q)
    print("repères identité : identique à la cinématique sans repères")


def test_aller_retour_reperes():
    """MGI(MGD(q)) = q pour toutes les combinaisons outil x repère utilisateur."""
    rng = np.random.default_rng(0)
    q = np.column_stack((rng.uniform(5, 40, (200, 3)), rng.uniform(-170, 170, 200)))
    pire = 0.0
    for nom_outil, outil in (("identité", [0] * 6), ("centré", OUTIL_CENTRE), ("crayon", OUTIL_CRAYON),
                             ("fort décalage", OUTIL_FORT_DECALAGE)):
        for nom_user, user in (("identité", [0] * 6), ("table", USER_TABLE), ("zone", USER_ZONE),
                               ("incliné", USER_INCLINE)):
            tool_f, user_f = dt.tool_frame_from_pose(outil), dt.user_frame_from_pose(user)
            retour = dt.delta_inverse_with_frames(dt.delta_forward_with_frames(q, tool_f, user_f), tool_f, user_f)
            assert not np.isnan(retour).any(), f"NaN avec outil {nom_outil} et repère {nom_user}"
            pire = max(pire, np.abs(retour - q).max())
    assert pire < 1e-6, f"aller-retour trop imprécis : {pire:.2e} °"
    print(f"aller-retour MGD/MGI : 16 combinaisons x 200 poses, erreur max = {pire:.2e} °")


def test_ligne_droite_outil_excentre():
    """Le TCP (0,0,0 du repère outil) suit une droite, pour plusieurs outils et repères utilisateur."""
    p0, p1 = [-40, -30, 20], [60, 50, 60]
    pire = 0.0
    for nom_outil, outil in (("centré", OUTIL_CENTRE), ("crayon 45°", OUTIL_CRAYON),
                             ("fort décalage", OUTIL_FORT_DECALAGE)):
        for nom_user, user in (("table", USER_TABLE), ("zone surélevée", USER_ZONE)):
            sc = _scenario_ligne(dt.tool_frame_from_pose(outil), dt.user_frame_from_pose(user), p0, p1)
            pire = max(pire, _verifier_ligne(sc, p0, p1, f"{nom_outil} / {nom_user}"))
    print(f"ligne droite du TCP (3 outils x 2 repères) : écart max à la droite = {pire:.2e} mm")


def test_ligne_avec_rotation():
    """TCP en ligne droite pendant que l'outil tourne : la bride, elle, doit décrire une courbe."""
    tool, user = dt.tool_frame_from_pose(OUTIL_CRAYON), dt.user_frame_from_pose(USER_TABLE)
    p0, p1 = [-40, -30, 20], [60, 50, 60]
    sc = _scenario_ligne(tool, user, p0, p1, rz0=0, rz1=90)
    ecart = _verifier_ligne(sc, p0, p1, "ligne + rotation")

    courbure_bride = _ecart_a_la_droite(sc["bride"], sc["bride"][0], sc["bride"][-1]).max()
    assert courbure_bride > 1.0, "la bride devrait s'écarter de la corde pour compenser l'excentration"
    assert sc["ecart_naif"].max() > 10.0, "sans repère outil, le TCP devrait s'écarter nettement de la droite"
    print(f"ligne + rotation de 90° : TCP droit à {ecart:.2e} mm près, "
          f"bride courbe de {courbure_bride:.1f} mm, sans repère outil le TCP dévie de {sc['ecart_naif'].max():.1f} mm")


def test_pivot_autour_du_tcp():
    """L'outil tourne sur place autour de son TCP : le TCP reste fixe, la bride décrit un cercle."""
    tool, user = dt.tool_frame_from_pose(OUTIL_CRAYON), dt.user_frame_from_pose(USER_TABLE)
    point = np.array([10.0, 10.0, 40.0])
    sc = _scenario_ligne(tool, user, point, point, rz0=0, rz1=180, n=91)
    assert not np.isnan(sc["angles"]).any()

    derive_tcp = np.linalg.norm(sc["retour"].translation - point, axis=1).max()
    rayon = np.linalg.norm((sc["bride"] - point)[:, :2], axis=1)
    hauteur = (sc["bride"] - point)[:, 2]
    assert derive_tcp < TOL_MM, f"le TCP bouge de {derive_tcp:.2e} mm pendant le pivot"
    assert np.allclose(rayon, OUTIL_CRAYON[0], atol=TOL_MM), "la bride doit rester sur un cercle de rayon 30 mm"
    assert np.allclose(hauteur, -OUTIL_CRAYON[2], atol=TOL_MM), "la bride doit rester à 80 mm au-dessus du TCP"
    print(f"pivot autour du TCP : dérive du TCP = {derive_tcp:.2e} mm, cercle de la bride r = {rayon.mean():.2f} mm")


def test_deplacement_dans_repere_outil():
    """Un mouvement exprimé dans le repère outil (axe X, Y ou Z de l'outil) suit bien cet axe."""
    tool, user = dt.tool_frame_from_pose(OUTIL_CRAYON), dt.user_frame_from_pose(USER_TABLE)
    depart = _poses_cible(user, tool, np.array([[10.0, 10.0, 40.0]]), [20.0])
    rot_tcp, pos_depart = _mat(depart)[:3, :3], _mat(depart)[:3, 3]

    for nom_axe, axe in (("X", [1, 0, 0]), ("Y", [0, 1, 0]), ("Z (axe du crayon)", [0, 0, 1])):
        deplacement_local = 15.0 * np.array(axe)
        hom = np.tile(_mat(depart), (51, 1, 1))
        s = np.linspace(0.0, 1.0, 51)
        hom[:, :3, 3] = pos_depart + s[:, None] * (rot_tcp @ deplacement_local)   # = depart @ translation locale

        cible = RigidTransform.from_homogeneous(hom)
        angles = dt.delta_inverse_with_frames(cible, tool, user)
        assert not np.isnan(angles).any(), f"axe {nom_axe} : pose inatteignable"
        tcp = dt.delta_forward_with_frames(angles, tool, user).translation

        direction = (tcp[-1] - tcp[0]) / np.linalg.norm(tcp[-1] - tcp[0])
        attendue = rot_tcp @ np.array(axe, float)
        ecart_angle = np.degrees(np.arccos(np.clip(direction @ attendue, -1, 1)))
        assert np.isclose(np.linalg.norm(tcp[-1] - tcp[0]), 15.0, atol=TOL_MM), f"axe {nom_axe} : mauvaise distance"
        assert ecart_angle < 1e-5, f"axe {nom_axe} : direction fausse de {ecart_angle:.2e} °"
        assert _ecart_a_la_droite(tcp, tcp[0], tcp[-1]).max() < TOL_MM, f"axe {nom_axe} : trajectoire non droite"

        if nom_axe.startswith("Z"):
            # Le crayon est incliné de 45° : le long de son axe, la bride avance autant à l'horizontale qu'à la verticale.
            bride = dt.delta_forward_with_frames(angles, IDENT, user).translation
            horizontal = np.linalg.norm((bride[-1] - bride[0])[:2])
            vertical = abs((bride[-1] - bride[0])[2])
            assert np.isclose(horizontal, vertical, atol=TOL_MM)
    print("mouvement dans le repère outil : axes X, Y et Z (crayon à 45°) suivis à moins de 1e-5 °")


def test_repere_utilisateur_incline():
    """Plan de travail incliné de 10° : le TCP reste dans le plan ; outil collé au plan -> refusé."""
    tool, user = dt.tool_frame_from_pose(OUTIL_CRAYON), dt.user_frame_from_pose(USER_INCLINE)
    p0, p1 = [-50, -40, 30], [50, 40, 30]
    sc = _scenario_ligne(tool, user, p0, p1)
    ecart = _verifier_ligne(sc, p0, p1, "plan incliné")
    assert np.allclose(sc["retour"].translation[:, 2], 30.0, atol=TOL_MM), "le TCP quitte le plan de travail"

    hom_user = _mat(user)
    tcp_base = sc["retour"].translation @ hom_user[:3, :3].T + hom_user[:3, 3]
    assert np.ptp(tcp_base[:, 2]) > 5.0, "dans la base, la ligne doit monter (plan incliné)"

    # Orientation « outil aligné sur le plan » : la bride devrait s'incliner de 10°, impossible pour un delta.
    cible = RigidTransform.from_pose([0, 0, 30, 0, 0, 0])
    assert np.isnan(dt.delta_inverse_with_frames(cible, IDENT, user)).all()
    accepte = dt.delta_inverse_with_frames(cible, IDENT, user, tol_inclinaison_deg=15.0)
    assert not np.isnan(accepte).any(), "avec une tolérance de 15°, l'inclinaison de 10° doit être ignorée"
    print(f"plan incliné de 10° : TCP dans le plan (écart à la droite {ecart:.2e} mm), "
          f"orientation inclinée refusée (NaN) sauf tolérance élargie")


def test_hors_espace_de_travail():
    """Une ligne qui sort de l'espace de travail donne des NaN, et seulement à partir du point de sortie."""
    tool, user = dt.tool_frame_from_pose(OUTIL_CRAYON), dt.user_frame_from_pose(USER_TABLE)
    positions = np.linspace([0, 0, 30], [900, 0, 30], 91)
    angles = dt.delta_inverse_with_frames(_poses_cible(user, tool, positions, np.zeros(91)), tool, user)
    invalides = np.isnan(angles).any(axis=1)
    assert not invalides[0] and invalides[-1]
    premier = int(np.argmax(invalides))
    assert invalides[premier:].all(), "des poses redeviennent valides après la sortie de l'espace de travail"
    assert np.isnan(angles[invalides]).all(), "une ligne invalide doit être NaN sur les 4 moteurs"

    # Angles impossibles en MGD : NaN sur la pose concernée seulement, le reste du lot reste valide
    mixte = dt.delta_forward_with_frames([[20, 25, 30, 0], [np.nan, 25, 30, 0]], tool, user)
    assert not np.isnan(mixte.translation[0]).any() and np.isnan(mixte.translation[1]).all()

    penche = RigidTransform.from_pose([0, 0, 30, 30, 0, 0])      # bride inclinée de 30°
    assert np.isnan(dt.delta_inverse_with_frames(penche, IDENT, IDENT)).all()
    print(f"hors espace de travail : sortie à {positions[premier, 0]:.0f} mm, NaN ensuite ; bride inclinée refusée")


def test_formes_de_sortie():
    """Entrée (4,) -> sortie (4,) / pose unique ; entrée (n, 4) -> (n, 4) ; n = 1 reste en (1, 4)."""
    tool, user = dt.tool_frame_from_pose(OUTIL_CRAYON), dt.user_frame_from_pose(USER_TABLE)
    q = np.array([20.0, 25.0, 30.0, 40.0])

    seule = dt.delta_forward_with_frames(q, tool, user)
    assert seule.as_pose().shape == (6,)
    assert dt.delta_inverse_with_frames(seule, tool, user).shape == (4,)

    lot = dt.delta_forward_with_frames(np.tile(q, (3, 1)), tool, user)
    assert lot.as_hom_mat().shape == (3, 4, 4)
    assert dt.delta_inverse_with_frames(lot, tool, user).shape == (3, 4)

    un = dt.delta_forward_with_frames(q[None, :], tool, user)
    assert dt.delta_inverse_with_frames(un, tool, user).shape == (1, 4)
    print("formes de sortie : (4,), (n, 4) et (1, 4) respectées")


def test_calibration_repere_utilisateur():
    """calibrate_user_frame retrouve un repère connu, exactement puis avec du bruit de mesure."""
    rng = np.random.default_rng(1)
    rot = Rotation.random(random_state=1).as_matrix()
    origine = np.array([40.0, -60.0, -380.0])
    pts = np.array([origine, origine + 150 * rot[:, 0], origine + 80 * rot[:, 0] + 120 * rot[:, 1]])

    frame = _mat(dt.calibrate_user_frame(pts))
    assert np.allclose(frame[:3, :3], rot, atol=1e-12) and np.allclose(frame[:3, 3], origine, atol=1e-12)

    # Les points enregistrés doivent avoir les bonnes coordonnées dans le repère calibré
    local = (~RigidTransform.from_homogeneous(frame)).transform_points(pts)
    assert np.allclose(local, [[0, 0, 0], [150, 0, 0], [80, 120, 0]], atol=1e-9)

    bruit = pts + rng.normal(0, 0.1, pts.shape)
    frame_bruit = _mat(dt.calibrate_user_frame(bruit))
    erreur_deg = np.degrees(np.linalg.norm(Rotation.from_matrix(frame_bruit[:3, :3] @ rot.T).as_rotvec()))
    assert np.allclose(frame_bruit[:3, :3] @ frame_bruit[:3, :3].T, np.eye(3), atol=1e-12), "repère non orthonormé"
    assert erreur_deg < 0.5, f"bruit de 0.1 mm : erreur d'orientation {erreur_deg:.2f} °"
    print(f"calibration repère utilisateur : exacte ; avec bruit de 0.1 mm, erreur d'orientation = {erreur_deg:.3f} °")


def test_calibration_outil():
    """calibrate_tool_frame retrouve le TCP si les rotations sont variées ; refuse le cas delta (rotation Z seule)."""
    rng = np.random.default_rng(2)
    p_tool = np.array([15.0, -25.0, 120.0])
    point_fixe = np.array([300.0, 100.0, 50.0])

    def poses_bride(angles, bruit=0.0):
        poses = []
        for a in angles:
            rot = Rotation.from_euler("xyz", a, degrees=True).as_matrix()
            poses.append([*(point_fixe - rot @ p_tool + rng.normal(0, bruit, 3)), *a])
        return np.array(poses)

    exact = dt.calibrate_tool_frame(poses_bride(rng.uniform(-60, 60, (8, 3))))
    assert np.allclose(exact.as_pose()[:3], p_tool, atol=1e-6) and np.allclose(exact.as_pose()[3:], 0.0)

    bruite = dt.calibrate_tool_frame(poses_bride(rng.uniform(-60, 60, (12, 3)), bruit=0.05))
    erreur = np.linalg.norm(bruite.as_pose()[:3] - p_tool)
    assert erreur < 0.5, f"bruit de 0.05 mm : TCP faux de {erreur:.2f} mm"

    rotation_z = np.column_stack((np.zeros((5, 2)), [0, 30, 60, 90, 120]))
    try:
        dt.calibrate_tool_frame(poses_bride(rotation_z))
    except ValueError:
        pass
    else:
        raise AssertionError("rotation autour de Z seule : une ValueError était attendue (z inobservable)")
    print(f"calibration outil : exacte ; avec bruit de 0.05 mm, erreur = {erreur:.3f} mm ; rotation Z seule refusée")


def test_poses4_et_repere_identite():
    """Les poses [x, y, z, rz] <-> angles passent par les transformées, sans perdre les rotations > 180°."""
    tool, user = dt.tool_frame_from_pose(OUTIL_CRAYON), dt.user_frame_from_pose(USER_TABLE)
    poses = np.array([[-40.0, -30.0, 20.0, 0.0], [10.0, 20.0, 40.0, 100.0], [60.0, 50.0, 60.0, 270.0]])

    # Orientation du TCP : même résultat que la formule R_user^T Rz(rz) R_tool écrite dans ce fichier de test
    cible = dt.tcp_pose_from_poses4(poses, tool, user)
    attendue = _poses_cible(user, tool, poses[:, :3], poses[:, 3])
    assert np.allclose(cible.as_hom_mat(), attendue.as_hom_mat(), atol=1e-12)

    # Sans repères les poses sont dans la base (z négatif sous les moteurs), pas dans le repère de la table
    poses_base = np.array([[0.0, 0.0, -300.0, 0.0], [100.0, 150.0, -350.0, 100.0], [-80.0, 60.0, -320.0, 270.0]])

    for nom, outil, repere, p in (("repères", tool, user, poses), ("identité (None)", None, None, poses_base)):
        angles = dt.delta_inverse_from_poses4(p, outil, repere)
        assert not np.isnan(angles).any(), f"{nom} : pose inatteignable"
        assert np.isclose(angles[2, 3], 270.0), f"{nom} : la rotation de 270° ne doit pas être repliée à -90°"
        retour = dt.delta_forward_to_poses4(angles, outil, repere)
        assert np.allclose(retour, p, atol=1e-6), f"{nom} : aller-retour poses -> angles -> poses"

    # None doit donner exactement la cinématique brute de la bride dans la base
    assert np.allclose(dt.delta_inverse_from_poses4(poses_base), dt.DeltaInverse4(poses_base), atol=1e-12)
    print("poses [x,y,z,rz] : aller-retour exact avec repères, rz = 270° conservé, None = cinématique brute")


def test_trajectoires_sans_reperes_inchangees():
    """Sans repères (None = identité), les trajectoires de trajectoires4Axes sont celles de la cinématique brute."""
    p0, p1 = [-200, 200, -200, 0], [175, -250, -350, 180]

    traj, duree = tr.cartesian_trapezoidal_trajectory(p0, p1, v_max_mm_s=200.0, a_max_mm_s2=500.0, dt=0.01)
    ident, _ = tr.cartesian_trapezoidal_trajectory(p0, p1, v_max_mm_s=200.0, a_max_mm_s2=500.0, dt=0.01,
                                                   tool_frame=IDENT, user_frame=IDENT)
    assert np.allclose(traj, ident, atol=1e-12), "None et identité explicite doivent donner le même résultat"

    xyz_rz = dt.DeltaForward4(traj)
    ecart = _ecart_a_la_droite(xyz_rz[:, :3], np.array(p0[:3], float), np.array(p1[:3], float)).max()
    assert ecart < 1e-6 and np.allclose(xyz_rz[-1], p1, atol=1e-6), f"ligne droite de la bride : écart {ecart:.2e} mm"

    for nom, (angles, _) in (("jointmotangles", tr.jointmotangles(p0, p1, 0.5, 250)),
                             ("jointmotangles_trapezoidal", tr.jointmotangles_trapezoidal(p0, p1, 30.0, 60.0))):
        assert np.allclose(angles[0], dt.DeltaInverse4(p0)[0], atol=1e-12), f"{nom} : départ différent de la cinématique brute"
        assert np.allclose(angles[-1], dt.DeltaInverse4(p1)[0], atol=1e-12), f"{nom} : arrivée différente de la cinématique brute"
    print(f"trajectoires sans repères : identiques à la cinématique brute (ligne droite à {ecart:.2e} mm, durée {duree:.2f} s)")


def test_trajectoires_avec_reperes():
    """Les trois trajectoires, avec outil excentré et repère utilisateur : le TCP part et arrive aux bonnes poses."""
    tool, user = dt.tool_frame_from_pose(OUTIL_CRAYON), dt.user_frame_from_pose(USER_TABLE)
    depart, arrivee = [-40, -30, 20, 0], [60, 50, 60, 90]
    longueur = np.linalg.norm(np.array(arrivee[:3]) - np.array(depart[:3]))

    # Ligne droite cartésienne : le TCP suit la droite, à la vitesse demandée
    v_max, a_max = 100.0, 300.0
    angles, duree = tr.cartesian_trapezoidal_trajectory(depart, arrivee, v_max_mm_s=v_max, a_max_mm_s2=a_max,
                                                        dt=0.01, tool_frame=tool, user_frame=user)
    poses = dt.delta_forward_to_poses4(angles, tool, user)
    ecart = _ecart_a_la_droite(poses[:, :3], np.array(depart[:3], float), np.array(arrivee[:3], float)).max()
    vitesse_tcp = np.linalg.norm(np.diff(poses[:, :3], axis=0), axis=1) / (duree / (len(poses) - 1))
    assert ecart < 1e-6, f"le TCP quitte la droite de {ecart:.2e} mm"
    assert np.allclose(poses[0], depart, atol=1e-6) and np.allclose(poses[-1], arrivee, atol=1e-6)
    assert vitesse_tcp.max() <= v_max * 1.001, f"vitesse du TCP {vitesse_tcp.max():.1f} mm/s > {v_max} mm/s"
    # Durée attendue : le mouvement le plus contraignant (TCP ou moteur 4) fixe le profil de l'avancement λ
    rotation = abs(arrivee[3] - depart[3]) / abs(dt.RAPPORT_AXE4)
    v_lambda = min(v_max / longueur, tr.V_MAX_AXE4_DEG_S / rotation)
    a_lambda = min(a_max / longueur, tr.A_MAX_AXE4_DEG_S2 / rotation)
    assert abs(duree - (1 / v_lambda + v_lambda / a_lambda)) < 0.02, "durée différente du profil trapézoïdal attendu"
    assert np.all(np.diff(poses[:, 3]) >= -1e-9), "rz doit croître régulièrement"

    # Trajectoires articulaires : seuls les points de départ et d'arrivée sont des poses demandées
    for nom, (q, _) in (("jointmotangles", tr.jointmotangles(depart, arrivee, 0.5, 250, tool_frame=tool, user_frame=user)),
                        ("jointmotangles_trapezoidal", tr.jointmotangles_trapezoidal(depart, arrivee, 30.0, 60.0,
                                                                                    tool_frame=tool, user_frame=user))):
        bouts = dt.delta_forward_to_poses4(q[[0, -1]], tool, user)
        assert np.allclose(bouts, [depart, arrivee], atol=1e-6), f"{nom} : le TCP ne part/n'arrive pas aux poses demandées"

    # Rotation de plus d'un demi-tour : θ4 continu, sans saut de 360°
    angles, _ = tr.cartesian_trapezoidal_trajectory([-20, 0, 30, 0], [20, 0, 30, 270], v_max_mm_s=100.0, a_max_mm_s2=300.0,
                                                    dt=0.01, tool_frame=tool, user_frame=user)
    assert np.isclose(angles[-1, 3], 270.0) and np.abs(np.diff(angles[:, 3])).max() < 5.0, "θ4 discontinu ou replié"
    print(f"trajectoires avec repères : TCP droit à {ecart:.2e} mm, vitesse max {vitesse_tcp.max():.1f} mm/s, "
          f"durée {duree:.2f} s ; rotation de 270° sans saut")


def test_trajectoire_inatteignable_avec_reperes():
    """Une trajectoire dont le TCP sort de l'espace de travail est refusée (ValueError), comme sans repères."""
    tool, user = dt.tool_frame_from_pose(OUTIL_CRAYON), dt.user_frame_from_pose(USER_TABLE)
    try:
        tr.cartesian_trapezoidal_trajectory([0, 0, 30, 0], [900, 0, 30, 0], 100.0, 300.0, dt=0.01,
                                            tool_frame=tool, user_frame=user)
    except ValueError as erreur:
        assert "inatteignable" in str(erreur)
    else:
        raise AssertionError("une ValueError était attendue pour une arrivée hors de l'espace de travail")
    print("trajectoire hors espace de travail avec repères : refusée par une ValueError")


def test_formes_et_entrees_invalides():
    """Les entrées incorrectes lèvent une ValueError au lieu de donner un résultat faux."""
    cas = {
        "outil de shape (3,)": lambda: dt.tool_frame_from_pose([1, 2, 3]),
        "repère utilisateur de shape (7,)": lambda: dt.user_frame_from_pose(np.zeros(7)),
        "3 points alignés": lambda: dt.calibrate_user_frame(np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]], float)),
        "points 0 et 1 confondus": lambda: dt.calibrate_user_frame(np.array([[0, 0, 0], [0, 0, 0], [1, 1, 0]], float)),
        "calibration utilisateur (2, 3)": lambda: dt.calibrate_user_frame(np.zeros((2, 3))),
        "calibration outil avec 3 poses": lambda: dt.calibrate_tool_frame(np.zeros((3, 6))),
        "calibration outil de shape (n, 3)": lambda: dt.calibrate_tool_frame(np.zeros((6, 3))),
        "MGD avec 3 angles": lambda: dt.delta_forward_with_frames([1, 2, 3], IDENT, IDENT),
    }
    for nom, appel in cas.items():
        try:
            appel()
        except ValueError:
            continue
        raise AssertionError(f"{nom} : une ValueError était attendue")
    print(f"entrées invalides : {len(cas)} cas refusés par une ValueError")


def test_entrees_limites_trajectoires():
    """Valeurs absurdes (NaN, infini, pas nul, repère du mauvais type, dt minuscule) : erreur claire.

    Aucun plantage obscur (AttributeError, RuntimeWarning, calcul interminable) ne doit sortir.
    """
    A, B = [0, 0, -300, 0], [50, 0, -300, 90]
    outil = dt.tool_frame_from_pose(OUTIL_CRAYON)
    erreurs_valeur = {
        "pose NaN": lambda: tr.cartesian_trapezoidal_trajectory([np.nan, 0, -300, 0], B, 100, 400),
        "pose infinie": lambda: tr.jointmotangles_trapezoidal([np.inf, 0, -300, 0], B, 60, 120),
        "pose de 5 valeurs": lambda: tr.lineartrajectory([0, 0, -300, 0, 0], B, 1.0),
        "outil NaN": lambda: dt.tool_frame_from_pose([np.nan, 0, 0, 0, 0, 0]),
        "repère utilisateur infini": lambda: dt.user_frame_from_pose([np.inf, 0, 0, 0, 0, 0]),
        "v_max infinie": lambda: tr.cartesian_trapezoidal_trajectory(A, B, np.inf, 400),
        "dt nul": lambda: tr.cartesian_trapezoidal_trajectory(A, B, 100, 400, dt=0),
        "dt minuscule (trop de points)": lambda: tr.cartesian_trapezoidal_trajectory(A, B, 1, 400, dt=1e-7),
        "distance énorme": lambda: tr.lineartrajectory(A, [1e9, 0, -300, 0], 1.0),
        "deg_per_step nul": lambda: tr.jointmotangles(A, B, 0),
        "deg_per_step négatif": lambda: tr.jointmotangles(A, B, -1),
        "steps_per_second nul": lambda: tr.jointmotangles(A, B, 2, steps_per_second=0),
    }
    for nom, appel in erreurs_valeur.items():
        try:
            appel()
        except ValueError:
            continue
        raise AssertionError(f"{nom} : une ValueError était attendue")

    erreurs_type = {
        "outil donné en chaîne": lambda: tr.cartesian_trapezoidal_trajectory(A, B, 100, 400, tool_frame="x"),
        "outil donné en matrice numpy": lambda: tr.jointmotangles(A, B, 2, tool_frame=np.eye(4)),
        "repère utilisateur en liste": lambda: dt.delta_inverse_from_poses4([A], user_frame=[0] * 6),
    }
    for nom, appel in erreurs_type.items():
        try:
            appel()
        except TypeError:
            continue
        raise AssertionError(f"{nom} : une TypeError était attendue")

    # Cas dégénérés valides : aucune erreur, résultat cohérent
    angles, duree = tr.cartesian_trapezoidal_trajectory([0, 0, -380, 0], [0, 0, -380, 0], 100, 400, tool_frame=outil)
    assert angles.shape == (1, 4) and duree == 0.0, "départ = arrivée : une seule pose, durée nulle"
    angles, duree = tr.cartesian_trapezoidal_trajectory([0, 0, -380, 0], [0, 0, -380, 180], 100, 400, tool_frame=outil)
    assert len(angles) > 1 and duree > 0, "rotation seule sur place"
    assert dt.delta_inverse_from_poses4(np.empty((0, 4))).shape == (0, 4), "aucune pose"
    avant_nan = dt.delta_forward_with_frames([np.nan] * 4, outil, None)
    assert np.isnan(avant_nan.translation).all(), "angles NaN -> pose NaN, sans exception"
    print(f"entrées limites : {len(erreurs_valeur)} ValueError, {len(erreurs_type)} TypeError, 4 cas dégénérés valides")


# ============================================================================
#                      GRAPHIQUES
# ============================================================================

def _vecteurs_outil(sc, pas=10):
    """Segments bride -> TCP, un échantillon sur `pas`, pour dessiner l'outil."""
    idx = np.arange(0, len(sc["bride"]), pas)
    return sc["bride"][idx], sc["retour"].translation[idx]


def figure_lignes():
    """Ligne + rotation, pivot, écart à la droite, angles moteurs."""
    couleurs = tr.COULEURS_MOTEURS
    tool, user = dt.tool_frame_from_pose(OUTIL_CRAYON), dt.user_frame_from_pose(USER_TABLE)
    p0, p1 = np.array([-40.0, -30.0, 20.0]), np.array([60.0, 50.0, 60.0])
    ligne = _scenario_ligne(tool, user, p0, p1, rz0=0, rz1=90)
    point = np.array([10.0, 10.0, 40.0])
    pivot = _scenario_ligne(tool, user, point, point, rz0=0, rz1=180, n=91)

    fig, axes = plt.subplots(2, 2, figsize=(13, 10))
    fig.suptitle("Repère outil excentré (crayon à 45°) : le TCP suit la trajectoire demandée", fontweight="bold")

    ax = axes[0, 0]
    bride, tcp = _vecteurs_outil(ligne)
    for b, t in zip(bride, tcp):
        ax.plot([b[0], t[0]], [b[1], t[1]], color="#c3c2b7", lw=1)
    ax.plot(ligne["positions"][:, 0], ligne["positions"][:, 1], "k--", lw=1, label="droite demandée (TCP)")
    ax.plot(*ligne["retour"].translation[:, :2].T, color=couleurs[0], lw=2.5, alpha=0.8, label="TCP obtenu")
    ax.plot(*ligne["bride"][:, :2].T, color=couleurs[1], lw=1.5, label="bride (courbe)")
    ax.set_title("Ligne droite du TCP pendant une rotation de 90°\n(vue de dessus, repère utilisateur)")
    ax.set_xlabel("x utilisateur (mm)"); ax.set_ylabel("y utilisateur (mm)")
    ax.set_aspect("equal"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = axes[0, 1]
    bride, tcp = _vecteurs_outil(pivot, pas=9)
    for b, t in zip(bride, tcp):
        ax.plot([b[0], t[0]], [b[1], t[1]], color="#c3c2b7", lw=1)
    ax.plot(*pivot["bride"][:, :2].T, color=couleurs[1], lw=1.5, label="bride (cercle)")
    ax.plot(*point[:2], "o", color=couleurs[0], ms=9, label="TCP (fixe)")
    ax.set_title("Pivot de 180° autour du TCP\n(vue de dessus, repère utilisateur)")
    ax.set_xlabel("x utilisateur (mm)"); ax.set_ylabel("y utilisateur (mm)")
    ax.set_aspect("equal"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = axes[1, 0]
    ecart_ligne = np.linalg.norm(ligne["retour"].translation - ligne["positions"], axis=1)
    ax.semilogy(np.maximum(ecart_ligne, 1e-12), color=couleurs[0], lw=2, label="avec repère outil (≈ 0, bruit numérique)")
    ax.semilogy(np.maximum(ligne["ecart_naif"], 1e-12), color=couleurs[3], lw=2, label="sans repère outil (bride sur la droite)")
    ax.set_title("Écart du TCP à la pose demandée")
    ax.set_xlabel("échantillon"); ax.set_ylabel("écart (mm, échelle log, plancher 1e-12)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3, which="both")

    ax = axes[1, 1]
    for i in range(4):
        ax.plot(ligne["angles"][:, i], color=couleurs[i], lw=1.8, label=f"θ{i + 1}")
    ax.set_title("Angles moteurs de la ligne + rotation")
    ax.set_xlabel("échantillon"); ax.set_ylabel("angle (°)")
    ax.legend(fontsize=8, ncol=4); ax.grid(alpha=0.3)

    fig.tight_layout()
    return fig


def figure_repere_incline():
    """Vue 3D : plan de travail incliné de 10°, ligne du TCP dans ce plan, crayon et bride."""
    couleurs = tr.COULEURS_MOTEURS
    tool, user = dt.tool_frame_from_pose(OUTIL_CRAYON), dt.user_frame_from_pose(USER_INCLINE)
    p0, p1 = [-50, -40, 30], [50, 40, 30]
    sc = _scenario_ligne(tool, user, p0, p1, n=51)

    hom = _mat(user)
    vers_base = lambda p: np.asarray(p) @ hom[:3, :3].T + hom[:3, 3]
    tcp, bride = vers_base(sc["retour"].translation), vers_base(sc["bride"])

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(projection="3d")
    plan = vers_base([[-80, -70, 0], [80, -70, 0], [80, 70, 0], [-80, 70, 0], [-80, -70, 0]])
    ax.plot(*plan.T, color=tr.AXES, lw=1.5)
    ax.plot_trisurf(plan[:4, 0], plan[:4, 1], plan[:4, 2], color=tr.GRILLE, alpha=0.5)
    origine = vers_base([0, 0, 0])
    for axe, couleur, nom in (([40, 0, 0], "#d6342a", "X"), ([0, 40, 0], "#1baf7a", "Y"), ([0, 0, 40], "#2a78d6", "Z")):
        fin = vers_base(axe)
        ax.plot(*zip(origine, fin), color=couleur, lw=2)
        ax.text(*fin, f"{nom} utilisateur", color=couleur, fontsize=8)
    ax.plot(*tcp.T, color=couleurs[0], lw=3, label="TCP (droite dans le plan)")
    ax.plot(*bride.T, color=couleurs[1], lw=1.5, label="bride")
    for b, t in zip(bride[::10], tcp[::10]):
        ax.plot(*zip(b, t), color="#52514e", lw=1.2)
    ax.set_title("Plan de travail incliné de 10° : ligne du TCP avec un crayon à 45°", fontsize=11)
    ax.set_xlabel("x base (mm)"); ax.set_ylabel("y base (mm)"); ax.set_zlabel("z base (mm)")
    tout = np.vstack((plan, tcp, bride))
    ax.set_box_aspect(np.ptp(tout, axis=0))
    ax.legend(fontsize=8)
    fig.tight_layout()
    return fig


# ============================================================================
#                      LANCEMENT
# ============================================================================

TESTS = [
    test_reperes_identite,
    test_aller_retour_reperes,
    test_ligne_droite_outil_excentre,
    test_ligne_avec_rotation,
    test_pivot_autour_du_tcp,
    test_deplacement_dans_repere_outil,
    test_repere_utilisateur_incline,
    test_hors_espace_de_travail,
    test_formes_de_sortie,
    test_poses4_et_repere_identite,
    test_trajectoires_sans_reperes_inchangees,
    test_trajectoires_avec_reperes,
    test_trajectoire_inatteignable_avec_reperes,
    test_calibration_repere_utilisateur,
    test_calibration_outil,
    test_formes_et_entrees_invalides,
    test_entrees_limites_trajectoires,
]


def test(dossier_sortie=None, afficher=True):
    """Lance tous les tests, enregistre les graphiques et lève AssertionError si un test échoue.

    Args:
        dossier_sortie (str | Path): dossier des images, défaut « Output » à la racine du projet
        afficher (bool): ouvrir les fenêtres matplotlib à la fin
    """
    dossier = Path(dossier_sortie) if dossier_sortie else RACINE_PROJET / "Output"
    dossier.mkdir(parents=True, exist_ok=True)

    echecs = []
    for fonction in TESTS:
        print(f"\n--- {fonction.__name__} ---")
        try:
            fonction()
        except Exception as erreur:       # un test en échec ne doit pas empêcher les suivants
            echecs.append(fonction.__name__)
            print(f"ÉCHEC : {type(erreur).__name__}: {erreur}")

    for nom_fichier, construire in (("test_delta_transform_lignes.png", figure_lignes),
                                    ("test_delta_transform_repere_incline.png", figure_repere_incline)):
        fig = construire()
        fig.savefig(dossier / nom_fichier, dpi=150, bbox_inches="tight")
        print(f"Graphique sauvegardé : {dossier / nom_fichier}")
        if not afficher:
            plt.close(fig)

    print("\n" + "=" * 70)
    if echecs:
        print(f"{len(echecs)} test(s) en échec sur {len(TESTS)} : {', '.join(echecs)}")
        raise AssertionError(f"Tests en échec : {', '.join(echecs)}")
    print(f"Tous les tests passent ({len(TESTS)}/{len(TESTS)})")

    if afficher:
        plt.show()


if __name__ == "__main__":
    test(afficher="--sans-affichage" not in sys.argv)
