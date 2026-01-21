import time
import numpy as np
import mujoco
import mujoco.viewer
from src.simulation.sim_utils import set_initial_pose, move_to_pose, hold_position # Note the path
from src.kinematics.inverse import calculate_ik

def show_cube(viewer, position, geom_num=0, halfwidth=0.013):
    """Visualizes the target position."""
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[geom_num],
        type=mujoco.mjtGeom.mjGEOM_BOX, 
        size=[halfwidth, halfwidth, halfwidth],                 
        pos=position,                         
        mat=np.eye(3).flatten(),              
        rgba=[1, 0, 0, 0.2]                   
    )
    viewer.user_scn.ngeom = 1
    viewer.sync()

def main():
    m = mujoco.MjModel.from_xml_path('model/scene.xml') # Update path if needed
    d = mujoco.MjData(m)

    # Initial Setup
    initial_config = {
        'shoulder_pan': 0.0, 'shoulder_lift': 0.0, 'elbow_flex': 0.0,
        'wrist_flex': 0.0, 'wrist_roll': 0.0, 'gripper': 0          
    }
    set_initial_pose(d, initial_config)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        # 1. Define Target
        desired_position = [0.1, -0.2, 0.02]
        show_cube(viewer, desired_position)

        # 2. Calculate IK (The shared logic!)
        print(f"Calculating IK for: {desired_position}")
        joint_configuration = calculate_ik(desired_position)
        print("Solved Joints:", joint_configuration)

        # 3. Move Robot
        move_to_pose(m, d, viewer, joint_configuration, 5)
        hold_position(m, d, viewer, 2000)

if __name__ == "__main__":
    main()