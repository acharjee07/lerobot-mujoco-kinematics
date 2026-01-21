import time
import numpy as np
import mujoco
import mujoco.viewer

# --- PROJECT IMPORTS ---
from src.simulation.sim_utils import set_initial_pose, send_position_command
from src.kinematics import get_forward_kinematics
# Use the cached Jacobian solver from your library
from src.kinematics.velocity import get_jacobian_solver

# ==========================================
# 1. SAFETY CONFIGURATION
# ==========================================
MAX_JOINT_VEL = 1.5         # rad/s (approx 85 deg/s)
MIN_MANIPULABILITY = 0.001  # Singularity threshold
DT = 0.01                   # Time step

# Joint Limits (Degrees)
JOINT_LIMITS = {
    'shoulder_pan':  (-100, 100),
    'shoulder_lift': (-100, 100),
    'elbow_flex':    (-100, 100),
    'wrist_flex':    (-100, 100),
    'wrist_roll':    (-150, 150)
}

# ==========================================
# 2. SAFETY FUNCTION
# ==========================================
def check_safety_and_clip(J_pos, q_dot_raw, current_angles_deg):
    """
    Filters the requested velocity (q_dot_raw) through safety checks.
    Returns: (q_dot_safe, is_safe_to_continue)
    """
    # A. Singularity Check (Manipulability Measure)
    # w = sqrt(det(J * J.T))
    try:
        # We add a tiny epsilon to J*J.T diagonal for stability if needed
        manipulability = np.sqrt(np.linalg.det(J_pos @ J_pos.T))
    except:
        manipulability = 0
        
    if manipulability < MIN_MANIPULABILITY:
        print(f"⚠️ DANGER: Singularity approaching! (w={manipulability:.5f})")
        # In a real robot, you might want to stop or dampen motion here
        return np.zeros_like(q_dot_raw), False 

    # B. Velocity Clamping
    q_dot_safe = np.clip(q_dot_raw, -MAX_JOINT_VEL, MAX_JOINT_VEL)
    
    # C. Joint Limit Prediction
    keys = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
    current_vals = [current_angles_deg[k] for k in keys]
    
    for i, val in enumerate(current_vals):
        # Predict next position
        predicted_val = val + np.degrees(q_dot_safe[i] * DT)
        min_lim, max_lim = JOINT_LIMITS[keys[i]]
        
        # If violating limits, only allow motion that moves AWAY from the limit
        if predicted_val < min_lim:
            if q_dot_safe[i] < 0: # Trying to go more negative
                print(f"⚠️ LIMIT: Joint {keys[i]} hit MIN ({predicted_val:.1f})")
                q_dot_safe[i] = 0
        elif predicted_val > max_lim:
            if q_dot_safe[i] > 0: # Trying to go more positive
                print(f"⚠️ LIMIT: Joint {keys[i]} hit MAX ({predicted_val:.1f})")
                q_dot_safe[i] = 0
            
    return q_dot_safe, True

# ==========================================
# 3. MAIN SIMULATION LOOP
# ==========================================
def main():
    m = mujoco.MjModel.from_xml_path('model/so101_new_calib.xml')
    d = mujoco.MjData(m)

    # Initial Setup
    initial_config = {
        'shoulder_pan': 0.0, 'shoulder_lift': 0, 'elbow_flex': 0,
        'wrist_flex': 0.0, 'wrist_roll': 0.0, 'gripper': 0
    }
    set_initial_pose(d, initial_config)
    send_position_command(d, initial_config)

    # Target & Gains
    target_pos = np.array([0.20, 0.20, 0.10]) 
    Kp = 2.0 
    Kd = 0.1 

    # Get Fast Jacobian Function
    calc_J = get_jacobian_solver()

    with mujoco.viewer.launch_passive(m, d) as viewer:
        # Visualizer: Green Target Sphere
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[0],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=np.array([0.015, 0.015, 0.015]),
            pos=target_pos,
            mat=np.eye(3).flatten(),
            rgba=np.array([0, 1, 0, 0.5], dtype=np.float32)
        )
        viewer.user_scn.ngeom = 1
        
        current_q_dict = initial_config.copy()
        prev_error_vec = np.zeros(3)

        print("Starting Safe Velocity Control Loop...")

        while viewer.is_running():
            # 1. Get State
            curr_pos_vec, _ = get_forward_kinematics(current_q_dict)
            curr_pos_vec = np.array(curr_pos_vec).flatten()
            
            # Prepare angles for Jacobian (Degrees -> Radians)
            th_vals = [np.deg2rad(current_q_dict[k]) for k in 
                       ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']]
            
            J = calc_J(*th_vals)
            
            # 2. PD Control
            error_vec = target_pos - curr_pos_vec
            error_derivative = (error_vec - prev_error_vec) / DT
            v_desired = (Kp * error_vec) + (Kd * error_derivative)
            prev_error_vec = error_vec
            
            dist = np.linalg.norm(error_vec)
            if dist < 0.002:
                print(f"Target Reached! Final Error: {dist:.5f}")
                break
                
            # 3. Inverse Kinematics (Velocity Level)
            J_pos = J[0:3, :] 
            q_dot_raw = np.linalg.pinv(J_pos, rcond=1e-2) @ v_desired
            
            # 4. *** APPLY SAFETY LAYER ***
            q_dot_safe, is_safe = check_safety_and_clip(J_pos, q_dot_raw, current_q_dict)
            
            if not is_safe:
                print("Safety Stop Triggered. Halting.")
                break

            # 5. Integrate & Move
            keys = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
            for i, key in enumerate(keys):
                delta_rad = q_dot_safe[i] * DT
                current_q_dict[key] += np.degrees(delta_rad)
                
            send_position_command(d, current_q_dict)
            mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(DT)

if __name__ == "__main__":
    main()