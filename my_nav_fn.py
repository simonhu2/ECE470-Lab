'''import numpy as np

def compute_gradients(state, targets, obstacles, r):
    """
    Compute a velocity field (interpreted by the envelope as -∇U) for each robot.

    Inputs:
      state:    (n, 2) ndarray. Current xy positions for n robots.
      targets:  (n, 2) ndarray. Target xy positions for the same robots.
      obstacles: list of dicts: [{"center": (x, y), "radius": R}, ...]
      r:        float. REQUIRED minimum pairwise distance between any two robots.
                If any pair gets <= r, the simulation declares a collision & fails.
                Use this to shape your inter-robot repulsion term/barriers.

    Returns:
      grads:    (n, 2) ndarray. Velocity vectors for each robot at the current state.
                The envelope will cap the speed and step the system forward.
                IMPORTANT: Must be finite numbers (no NaN/Inf).
    """

    # ============================================================
    # TODO: Replace this naive baseline with YOUR navigation field.
    # ============================================================

    # --- Naive placeholder baseline (will often fail): straight to target.
    V = targets - state
    norms = np.linalg.norm(V, axis=1, keepdims=True) + 1e-12
    return V / norms  # unit direction toward target
'''
import numpy as np

def compute_gradients(state, targets, obstacles, r):
    """
    state:    (n,2) robot positions
    targets:  (n,2) robot goal positions
    obstacles: list of {"center":(x,y), "radius":R}
    r: robot radius
    returns: (n,2) velocity/gradient
    """

    n = state.shape[0]
    eps = 1e-6

    # Tunable weights — change these if you fail some permutations
    k_goal = 1.0
    k_pair = 0.01
    k_obs  = 0.03
    k_wall = 0.02

    # Sideways/tangential strength
    k_tan  = 0.03   # <-- You can tune this too

    # Output
    V = np.zeros_like(state)

    # ------------------------------------------------------------------
    # 1) GOAL TERM (attractive)
    # ------------------------------------------------------------------
    V += -k_goal * (state - targets)

    # ------------------------------------------------------------------
    # 2) PAIRWISE REPULSION
    # ------------------------------------------------------------------
    for i in range(n):
        for j in range(i+1, n):
            diff = state[i] - state[j]
            d = np.linalg.norm(diff) + eps

            min_dist = 2*r   # collision threshold for center distance

            if d < 3*min_dist:        # influence zone
                rep = k_pair * diff / ((d - min_dist + eps)**1.5)

                V[i] += rep
                V[j] -= rep

    # ------------------------------------------------------------------
    # 3) OBSTACLE AVOIDANCE  +  TANGENTIAL COMPONENT (to avoid deadlocks)
    # ------------------------------------------------------------------
    for obs in obstacles:
        c = np.array(obs["center"], dtype=float)
        R = float(obs["radius"])

        diff = state - c
        d = np.linalg.norm(diff, axis=1) + eps
        min_clearance = R + r

        # Influence region
        mask = d < (min_clearance + 0.15)

        if np.any(mask):
            active = diff[mask].astype(float)         # <--- force float
            d_act = d[mask][:, None].astype(float)    # <--- force float

            # 3a) Normal repulsion
            rep = k_obs * active / ((d_act - min_clearance + eps)**2)
            rep = rep.astype(float)                   # <--- ensure float
            V[mask] += rep

            # -----------------------------------------------------------
            # 3b) Tangential component
            # -----------------------------------------------------------
            tangent = np.zeros_like(active, dtype=float)
            tangent[:, 0] = -active[:, 1]
            tangent[:, 1] =  active[:, 0]

            norms = np.linalg.norm(tangent, axis=1, keepdims=True) + eps
            tangent = tangent / norms
            tangent = tangent.astype(float)           # <--- ensure float

            V[mask] += k_tan * tangent
    V = np.asarray(V, dtype=float)  # <--- force final dtype
    V[np.isnan(V)] = 0.0            # no NaNs allowed
    V[np.isinf(V)] = 0.0            # no Infs allowed
    return V

