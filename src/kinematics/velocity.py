import numpy as np
import sympy as sp
# Import the symbolic variables and matrix from your existing file
from .jacobian_symbolic import J_total, theta1, theta2, theta3, theta4, theta5
from .jacobian_symbolic import l1, l2, l3, l4, l5, l6, l7, l8, l9

# --- Constants (Same as in inverse.py) ---
LINK_VALUES = {
    l1: 0.0388353, l2: 0.0624, l3: 0.0303992, l4: 0.0542,
    l5: 0.11257, l6: 0.028, l7: 0.1349, l8: 0.0611, l9: 0.1034
}

# Global cache for the compiled function
_FAST_J_FUNC = None

def get_jacobian_solver():
    """
    Compiles the symbolic Jacobian into a fast NumPy function.
    Uses caching so it only compiles once (which is slow).
    """
    global _FAST_J_FUNC
    if _FAST_J_FUNC is None:
        print("Compiling Jacobian... (This happens once)")
        J_with_constants = J_total.subs(LINK_VALUES)
        _FAST_J_FUNC = sp.lambdify(
            (theta1, theta2, theta3, theta4, theta5),
            J_with_constants, "numpy"
        )
    return _FAST_J_FUNC

def calculate_velocity_step(current_joints_deg, target_pos, current_pos, dt):
    """
    Calculates the required joint velocities (q_dot) to reach target.
    Returns: q_dot in radians/sec
    """
    # 1. Get Solver
    calc_J = get_jacobian_solver()
    
    # 2. Prepare Angles (Degrees -> Radians)
    th_vals = [np.deg2rad(current_joints_deg[k]) for k in 
               ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']]
    
    # 3. Calculate Jacobian
    J = calc_J(*th_vals)
    J_pos = J[0:3, :] # Take top 3 rows (Position only)
    
    # 4. Calculate Error Vector
    error_vec = target_pos - current_pos
    
    # 5. Pseudo-Inverse Control (v = J * q_dot  =>  q_dot = pinv(J) * v)
    # Using the error vector directly as 'desired velocity' for simple P-control
    q_dot_rad = np.linalg.pinv(J_pos, rcond=1e-2) @ error_vec
    
    return q_dot_rad, error_vec