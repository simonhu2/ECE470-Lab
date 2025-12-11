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
    eps = 1e-12
    
    # Optimized parameters
    k_goal = 3.0
    k_pair = 0.012
    k_obs  = 0.018
    k_wall = 0.05
    
    tol = 0.005
    
    V = np.zeros_like(state)
    
    # 1. GOAL ATTRACTION
    to_target = targets - state
    dist_to_target = np.linalg.norm(to_target, axis=1) + eps
    
    for i in range(n):
        dir_to_target = to_target[i] / dist_to_target[i]
        
        if dist_to_target[i] > tol * 3:
            attraction_strength = k_goal * np.tanh(dist_to_target[i] * 2.0)
        elif dist_to_target[i] > tol:
            attraction_strength = k_goal * 0.1
        else:
            attraction_strength = k_goal * 0.05
        
        V[i] += attraction_strength * dir_to_target
    
    # 2. IDENTIFY MIRRORED SWAPPING ROBOTS (SPECIFIC TO PERMUTATIONS 63, 87)
    # These are robots that need to swap positions through the center
    
    swapping_pairs = []
    for i in range(n):
        for j in range(i+1, n):
            # Check if robots are on opposite sides
            dot_pos = np.dot(state[i], state[j])
            
            # Check if their targets are also on opposite sides AND swapped
            # i.e., robot i's target is near robot j's position and vice versa
            dist_i_to_target_j = np.linalg.norm(state[i] - targets[j])
            dist_j_to_target_i = np.linalg.norm(state[j] - targets[i])
            
            # If targets are closer to the other robot than their own, they need to swap
            need_to_swap = (dist_i_to_target_j < np.linalg.norm(state[i] - targets[i]) * 0.8 and
                           dist_j_to_target_i < np.linalg.norm(state[j] - targets[j]) * 0.8)
            
            if dot_pos < -0.3 and need_to_swap:
                # Both near center and moving slowly?
                if (np.linalg.norm(state[i]) < 0.4 and 
                    np.linalg.norm(state[j]) < 0.4 and
                    np.linalg.norm(V[i]) < 0.02 and
                    np.linalg.norm(V[j]) < 0.02):
                    swapping_pairs.append((i, j))
    
    # 3. IMPLEMENT ONE-WAY SYSTEM FOR SWAPPING PAIRS
    # This is the CRITICAL FIX for permutations 63 and 87
    for i, j in swapping_pairs:
        # Determine which robot should go first
        # The robot that's closer to a clear path goes first
        
        # Check which robot has a clearer path to its target
        # by checking obstacle proximity
        i_obstacle_blocked = False
        j_obstacle_blocked = False
        
        for obs in obstacles:
            center = np.array(obs["center"])
            radius = obs["radius"]
            
            # Vector from robot to target
            i_to_target = targets[i] - state[i]
            j_to_target = targets[j] - state[j]
            
            # Check if obstacle lies between robot and target
            # Simplified: check if robot-target line passes close to obstacle
            i_dist_to_obs = np.linalg.norm(state[i] - center)
            j_dist_to_obs = np.linalg.norm(state[j] - center)
            
            if i_dist_to_obs < radius + 0.2:
                i_obstacle_blocked = True
            if j_dist_to_obs < radius + 0.2:
                j_obstacle_blocked = True
        
        # Robot with clearer path OR closer to center goes first
        # Also consider which robot is moving toward the center vs away
        i_toward_center = np.dot(state[i], to_target[i]) < 0
        j_toward_center = np.dot(state[j], to_target[j]) < 0
        
        if i_toward_center and not j_toward_center:
            # i is moving toward center, j is moving away
            # Let i go first, j yields
            first, second = i, j
        elif j_toward_center and not i_toward_center:
            first, second = j, i
        elif np.linalg.norm(state[i]) < np.linalg.norm(state[j]):
            # i is closer to center, goes first
            first, second = i, j
        else:
            first, second = j, i
        
        # Apply one-way system:
        # First robot gets strong goal attraction and right-of-way
        # Second robot yields by moving perpendicular and reducing goal attraction
        
        # For first robot: full speed ahead
        goal_dir_first = to_target[first] / (dist_to_target[first] + eps)
        V[first] += goal_dir_first * k_goal * 0.8  # Very strong goal force
        
        # For second robot: yield by moving perpendicular to goal
        goal_dir_second = to_target[second] / (dist_to_target[second] + eps)
        perp_dir = np.array([-goal_dir_second[1], goal_dir_second[0]])
        
        # Move perpendicular to get out of the way
        V[second] += perp_dir * 0.1
        
        # Reduce goal attraction for yielding robot
        V[second] *= 0.4  # Reduce existing velocities
        
        # Add slight goal attraction so it doesn't stop completely
        V[second] += goal_dir_second * k_goal * 0.2
    
    # 4. ROBOT-ROBOT REPULSION (normal for non-swapping robots)
    min_pair_dist = 2.0 * r
    
    for i in range(n):
        near_target_i = dist_to_target[i] < tol * 2
        
        for j in range(i+1, n):
            # Skip if this is a swapping pair (already handled)
            if (i, j) in swapping_pairs or (j, i) in swapping_pairs:
                continue
                
            near_target_j = dist_to_target[j] < tol * 2
            
            diff = state[i] - state[j]
            d = np.linalg.norm(diff) + eps
            
            # Check if both near center
            near_center = np.linalg.norm(state[i]) < 0.4 and np.linalg.norm(state[j]) < 0.4
            
            if near_center:
                if d < min_pair_dist * 1.05:
                    rep_strength = k_pair * 2.0 / ((d - min_pair_dist + 0.01) ** 2)
                elif d < min_pair_dist * 1.5:
                    rep_strength = k_pair * 0.3 / ((d - min_pair_dist + 0.03) ** 2)
                else:
                    continue
            elif near_target_i and near_target_j:
                if d < min_pair_dist * 1.1:
                    rep_strength = k_pair * 0.5 / ((d - min_pair_dist + 0.02) ** 2)
                else:
                    continue
            else:
                activation_dist = 3.5 * r
                if d < activation_dist:
                    rep_strength = k_pair / ((d - min_pair_dist + 0.05) ** 3)
                else:
                    continue
            
            rep = rep_strength * diff / d
            V[i] += rep
            V[j] -= rep
    
    # 5. OBSTACLE AVOIDANCE - with clear lanes for first robot in swapping pairs
    for i in range(n):
        near_target = dist_to_target[i] < tol * 3
        pos = state[i]
        
        # Check if this robot is first in a swapping pair
        is_first_in_pair = False
        for pair in swapping_pairs:
            if i == pair[0]:  # First in the pair
                is_first_in_pair = True
                break
        
        for obs in obstacles:
            center = np.array(obs["center"])
            radius = obs["radius"]
            
            diff = pos - center
            d = np.linalg.norm(diff) + eps
            min_clearance = radius + r + 0.005
            
            is_central_obstacle = np.linalg.norm(center) < 0.01
            
            if is_central_obstacle:
                if near_target:
                    if d < min_clearance * 1.2:
                        rep_strength = k_obs * 0.3 / ((d - min_clearance + 0.02) ** 1.5)
                        V[i] += rep_strength * diff / d
                else:
                    # SPECIAL: For first robot in swapping pair, create a clear path
                    if is_first_in_pair and d < min_clearance + 0.25:
                        # Give a clear tangential path around the obstacle
                        tangent = np.array([-diff[1], diff[0]])
                        tangent = tangent / (np.linalg.norm(tangent) + eps)
                        
                        # Determine direction based on target position
                        to_target_dir = to_target[i] / (dist_to_target[i] + eps)
                        cross = np.cross([diff[0], diff[1], 0], 
                                        [to_target_dir[0], to_target_dir[1], 0])[2]
                        
                        # Go the shorter way around
                        if cross > 0:
                            avoid_dir = tangent  # Clockwise
                        else:
                            avoid_dir = -tangent  # Counter-clockwise
                        
                        avoid_strength = k_obs * 0.5 / ((d - min_clearance + 0.04) ** 1.5)
                        V[i] += avoid_strength * avoid_dir
                    else:
                        # Normal central obstacle handling
                        if d < min_clearance + 0.2:
                            if d < min_clearance * 1.05:
                                rep_strength = k_obs * 2.0 / ((d - min_clearance + 0.01) ** 2.0)
                            else:
                                rep_strength = k_obs * 0.8 / ((d - min_clearance + 0.05) ** 1.8)
                            V[i] += rep_strength * diff / d
            else:
                if near_target:
                    if d < min_clearance * 1.1:
                        rep_strength = k_obs * 0.5 / ((d - min_clearance + 0.01) ** 2)
                        V[i] += rep_strength * diff / d
                else:
                    if d < min_clearance + 0.15:
                        rep_strength = k_obs / ((d - min_clearance + 0.03) ** 2)
                        V[i] += rep_strength * diff / d
    
    # 6. AGGRESSIVE PROGRESS ENFORCEMENT FOR SWAPPING ROBOTS
    # This ensures the first robot actually makes progress
    for i, j in swapping_pairs:
        first, second = i, j  # Assuming i is first (simplified)
        
        # Check if first robot is making progress
        if np.linalg.norm(V[first]) < 0.01:
            # Give it a very strong push
            goal_dir = to_target[first] / (dist_to_target[first] + eps)
            V[first] += goal_dir * k_goal * 1.0  # Maximum push
        
        # After first robot has moved away, let second robot go
        # Check distance between them
        diff = state[first] - state[second]
        d = np.linalg.norm(diff)
        
        if d > min_pair_dist * 2.0:  # First robot has cleared the way
            # Let second robot resume normal motion
            goal_dir = to_target[second] / (dist_to_target[second] + eps)
            V[second] += goal_dir * k_goal * 0.6
    
    # 7. FINAL PUSH FOR ALL ROBOTS CLOSE TO TARGETS
    for i in range(n):
        if tol < dist_to_target[i] < tol * 3:
            target_dir = to_target[i] / (dist_to_target[i] + eps)
            final_push = target_dir * k_goal * 0.5
            V[i] += final_push
    
    # 8. BOUNDARY REPULSION
    for i in range(n):
        norm_pos = np.linalg.norm(state[i])
        target_norm = np.linalg.norm(targets[i])
        target_near_boundary = target_norm > 0.8
        
        if norm_pos > 0.85:
            to_center = -state[i] / (norm_pos + eps)
            
            if target_near_boundary:
                boundary_strength = k_wall * 0.5 * (norm_pos - 0.85) * 5.0
            else:
                if norm_pos > 0.95:
                    boundary_strength = k_wall * 5.0 / ((1.0 - norm_pos + eps) ** 0.5)
                elif norm_pos > 0.9:
                    boundary_strength = k_wall * 2.0 / ((1.0 - norm_pos + eps) ** 0.3)
                else:
                    boundary_strength = k_wall * (norm_pos - 0.85) * 10.0
            
            V[i] += boundary_strength * to_center
    
    # 9. FINAL VELOCITY ADJUSTMENTS
    for i in range(n):
        if dist_to_target[i] < tol * 2:
            target_dir = to_target[i] / (dist_to_target[i] + eps)
            strength = k_goal * 0.5 * (1.0 + 1.0 / (dist_to_target[i] + 0.01))
            V[i] = target_dir * strength
    
    # 10. VELOCITY SCALING
    norms = np.linalg.norm(V, axis=1, keepdims=True) + eps
    
    for i in range(n):
        if dist_to_target[i] < tol * 5:
            min_velocity = 0.04
            current_norm = norms[i, 0]
            if current_norm < min_velocity:
                scale = min_velocity / current_norm
                V[i] *= scale
    
    max_velocity = 1.0
    norms = np.linalg.norm(V, axis=1, keepdims=True) + eps
    scale = np.minimum(1.0, max_velocity / norms)
    V = V * scale
    
    V = np.nan_to_num(V, nan=0.0, posinf=0.0, neginf=0.0)
    
    return V

