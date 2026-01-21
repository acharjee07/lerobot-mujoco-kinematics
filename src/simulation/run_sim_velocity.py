import time
import numpy as np
import mujoco
import mujoco.viewer
from src.simulation.sim_utils import set_initial_pose, send_position_command
from src.kinematics import get_forward_kinematics, calculate_velocity_step

def main():
    m = mujoco.MjModel.from_xml_path('model/so101_new_calib.xml')
    d = mujoco.MjData(m)

    # Initial Config
    initial_config = {
        'shoulder_pan': 0.0, 'shoulder_lift': 0, 'elbow_flex': 0,
        'wrist_flex': 0.0, 'wrist_roll': 0.0, 'gripper': 0
    }
    set_initial_pose(d, initial_config)
    
    # Track current state in a dict
    current_q_dict = initial_config.copy()
    
    target_pos = np.array([0.20, 0.20, 0.10])
    dt = 0.01

    with mujoco.viewer.launch_passive(m, d) as viewer:
        # Visualizer Setup
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[0], type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.015, 0.015, 0.015], pos=target_pos,
            mat=np.eye(3).flatten(), rgba=[0, 1, 0, 0.5]
        )
        viewer.user_scn.ngeom = 1

        print("Starting Simulation Loop...")
        while viewer.is_running():
            # 1. Forward Kinematics
            curr_pos, _ = get_forward_kinematics(current_q_dict)
            
            # 2. Velocity Control Calculation (Shared Logic)
            q_dot_rad, error_vec = calculate_velocity_step(
                current_q_dict, target_pos, curr_pos, dt
            )
            
            dist = np.linalg.norm(error_vec)
            if dist < 0.001:
                print(f"Target Reached! Error: {dist:.5f}")
                break

            # 3. Integration (Euler: q_new = q_old + q_dot * dt)
            keys = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
            for i, key in enumerate(keys):
                delta_deg = np.degrees(q_dot_rad[i] * dt)
                current_q_dict[key] += delta_deg

            # 4. Update Sim
            send_position_command(d, current_q_dict)
            mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(dt)

if __name__ == "__main__":
    main()