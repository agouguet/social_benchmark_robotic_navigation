import math
import random
import numpy as np
from typing import List, Tuple, Optional

LIMIT_DISTANCE_TO_OTHER_AGENT = 0.6

def _to_np_points(pts, dtype=np.float32):
    if not pts:
        return np.empty((0, 2), dtype=dtype)
    arr = np.asarray(pts, dtype=dtype)
    if arr.ndim != 2 or arr.shape[1] != 2:
        raise ValueError("Chaque point doit être de forme (x, y).")
    return arr

def _pairwise_min_dist(actions_np: np.ndarray, points_np: np.ndarray, chunk_size: int = 4096) -> np.ndarray:
    """
    Pour chaque action (N,2), renvoie la distance minimale à l'ensemble points (M,2),
    en traitant par blocs pour limiter l'utilisation mémoire.
    """
    N = actions_np.shape[0]
    out = np.full(N, np.inf, dtype=np.float32)
    if points_np.size == 0:
        return out

    # On découpe les points (M,2) en colonnes de blocs
    M = points_np.shape[0]
    for start in range(0, M, chunk_size):
        end = min(start + chunk_size, M)
        blk = points_np[start:end]  # (B,2)
        # distances pour toutes les actions vers ce bloc: (N,B)
        # ||a - b|| = sqrt(sum((a-b)^2, axis=2))
        d = actions_np[:, None, :] - blk[None, :, :]
        d = np.sqrt(np.sum(d * d, axis=2), dtype=np.float32)
        # min sur l'axe des points du bloc
        out = np.minimum(out, np.min(d, axis=1))
    return out

def _normalize_inverse_distance(d: np.ndarray) -> np.ndarray:
    """
    Transforme une distance par une normalisation min-max inversée:
      score = 1 - (d - dmin) / (dmax - dmin)
    Cas dégénéré (dmax == dmin): on renvoie 1.0.
    """
    if d.size == 0:
        return d
    dmin = d.min()
    dmax = d.max()
    if not np.isfinite(dmin) or not np.isfinite(dmax) or dmax == dmin:
        return np.ones_like(d, dtype=np.float32)
    return 1.0 - (d - dmin) / (dmax - dmin)

def heuristic_score_based(
    robot: Tuple[float, float],
    goal: Tuple[float, float],
    actions: List[Tuple[float, float]],
    previous_path: Optional[List[Tuple[float, float]]] = None,
    humans: Optional[List[Tuple[float, float]]] = None,
    future_humans: Optional[List[List[Tuple[float, float]]]] = None,
    *,
    w1: float = 1.0,    # poids dg (proximité du but)
    w2: float = 3.0,    # poids dnear (distance aux agents)
    w3: float = 0.1,    # poids dm (coût du mouvement)
    w4: float = 0.5,    # poids do (orientation par rapport à l'humain le + proche)
    w5: float = 1.0,    # poids dp (éloignement du chemin précédent)
    limit: float = LIMIT_DISTANCE_TO_OTHER_AGENT,
    k: float = 0.5,     # pénalisation par l'écart-type
    alpha: float = 0.8, # paramètre de l'atténuation dnear
    chunk_size: int = 4096,
    rng: Optional[random.Random] = None,
    return_debug: bool = False,
):
    """
    Version entièrement vectorisée. Retourne la meilleure action (x, y).
    Si return_debug=True, retourne aussi un dict avec tous les vecteurs de scores.
    """
    if rng is None:
        rng = random

    actions_np = _to_np_points(actions)              # (N,2)
    if actions_np.shape[0] == 0:
        return None if not return_debug else (None, {})

    robot_np = np.asarray(robot, dtype=np.float32)   # (2,)
    goal_np  = np.asarray(goal, dtype=np.float32)    # (2,)

    prev_np  = _to_np_points(previous_path or [])
    humans_np = _to_np_points(humans or [])
    # Aplatissement des futures positions des humains
    if future_humans:
        flat_future = [pt for traj in future_humans for pt in traj]
    else:
        flat_future = []
    future_np = _to_np_points(flat_future)

    # 1) Distances de performance (dm: robot->action, dg: goal->action)
    dm = np.linalg.norm(actions_np - robot_np[None, :], axis=1).astype(np.float32)  # (N,)
    dg = np.linalg.norm(actions_np - goal_np[None, :], axis=1).astype(np.float32)   # (N,)
    dm_score = _normalize_inverse_distance(dm)  # plus petit déplacement -> meilleur score
    dg_score = _normalize_inverse_distance(dg)  # plus proche du but -> meilleur score

    # 2) Distance au chemin précédent (dp: éloignement du chemin)
    if prev_np.shape[0] > 1:
        dp_dist = _pairwise_min_dist(actions_np, prev_np, chunk_size=chunk_size)  # (N,)
        dp_score = _normalize_inverse_distance(dp_dist)  # plus loin du chemin -> meilleur score (comme original)
    else:
        dp_score = np.ones(actions_np.shape[0], dtype=np.float32)

    # 3) Proximité aux agents (dnear)
    # Combine humains actuels + futurs
    agents_np = np.vstack([x for x in (humans_np, future_np) if x.size > 0]) if (humans_np.size or future_np.size) else np.empty((0, 2), dtype=np.float32)
    if agents_np.size > 0:
        dist_agents = _pairwise_min_dist(actions_np, agents_np, chunk_size=chunk_size)  # (N,)
        # mapping non linéaire comme dans ton code:
        # dnear = 1 si dist >= limit, sinon dist/limit * exp(-alpha * (limit - dist)^2)
        dnear = np.where(dist_agents >= limit, 1.0, (dist_agents / limit) * np.exp(-alpha * (limit - dist_agents) ** 2)).astype(np.float32)
    else:
        dnear = np.ones(actions_np.shape[0], dtype=np.float32)

    # 4) Orientation par rapport à l'humain le plus proche du robot (do)
    #    On garde la même logique: si pas d'humains -> 0.0
    if humans_np.shape[0] == 0:
        do = np.zeros(actions_np.shape[0], dtype=np.float32)
    else:
        # trouver l'humain le plus proche du robot
        vec_rh = humans_np - robot_np[None, :]           # (H,2)
        dist_rh = np.linalg.norm(vec_rh, axis=1)         # (H,)
        idx_closest = int(np.argmin(dist_rh))
        h = humans_np[idx_closest]                       # (2,)

        # v1 = robot->humain, v2 = robot->action (vectorisé)
        v1 = h - robot_np                                # (2,)
        v2 = actions_np - robot_np[None, :]              # (N,2)

        # produit scalaire -> humain pas derrière ?
        dot = v2 @ v1  # (N,)
        is_not_behind = (dot > 0).astype(np.float32)

        # produit vectoriel 2D (z-component)
        # cross = (hx - rx)*(ay - ry) - (hy - ry)*(ax - rx)
        cross = (h[0] - robot_np[0]) * (v2[:, 1]) - (h[1] - robot_np[1]) * (v2[:, 0])

        do = np.where(
            cross == 0.0,
            0.0,
            np.where(cross > 0.0, 0.9 * is_not_behind, 1.0 * is_not_behind)
        ).astype(np.float32)

    # 5) Agrégation avec pénalisation par l'écart-type
    w = np.array([w1, w2, w3, w4, w5], dtype=np.float32)
    # Empilement des composantes pour calcul par action
    comps = np.vstack([dg_score, dnear, dm_score, do, dp_score]).astype(np.float32)  # shape (5, N)

    # moyennes et variances pondérées par action
    sumw = w.sum()
    mean = (w[:, None] * comps).sum(axis=0) / sumw
    var = (w[:, None] * (comps - mean[None, :]) ** 2).sum(axis=0) / sumw
    std = np.sqrt(var, dtype=np.float32)

    score = mean - k * std

    # 6) Sélection de la/les meilleure(s) action(s)
    max_val = np.max(score)
    idxs = np.flatnonzero(np.isclose(score, max_val))
    idx = int(rng.choice(idxs)) if idxs.size > 1 else int(idxs[0])
    best_action = tuple(map(float, actions_np[idx]))

    if not return_debug:
        return best_action

    debug = {
        "index": idx,
        "action": actions,
        "score_max": float(max_val),
        "scores": score,
        "dg_score": dg_score,
        "dnear": dnear,
        "dm_score": dm_score,
        "do": do,
        "dp_score": dp_score,
    }
    return best_action, debug
