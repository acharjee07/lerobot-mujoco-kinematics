import time
import mujoco
        
def convert_to_dictionary(qpos):
    return {
        'shoulder_pan': qpos[0]*180.0/3.14159,    # convert to degrees
        'shoulder_lift': qpos[1]*180.0/3.14159,   # convert to degrees
        'elbow_flex': qpos[2]*180.0/3.14159,      # convert to degrees
        'wrist_flex': qpos[3]*180.0/3.14159,      # convert to degrees
        'wrist_roll': qpos[4]*180.0/3.14159,      # convert to degrees
        'gripper': qpos[5]*100/3.14159            # convert to 0-100 range
    }
    
def convert_to_list(dictionary):
    return [
        dictionary['shoulder_pan']*3.14159/180.0,
        dictionary['shoulder_lift']*3.14159/180.0,
        dictionary['elbow_flex']*3.14159/180.0,
        dictionary['wrist_flex']*3.14159/180.0,
        dictionary['wrist_roll']*3.14159/180.0,
        dictionary['gripper']*3.14159/100.0
    ]


def set_initial_pose(d, position_dict):
    pos = convert_to_list(position_dict)
    d.qpos = pos


def send_position_command(d, position_dict):
    pos = convert_to_list(position_dict)
    d.ctrl = pos
import mujoco

def move_to_pose(m, d, viewer, desired_position, duration):
    start_time = time.time()
    starting_pose = d.qpos.copy()
    starting_pose = convert_to_dictionary(starting_pose)
    
    while True:
        t = time.time() - start_time
        if t > duration:
            break

        # Interpolation factor [0,1] (make sure it doesn't exceed 1)
        alpha = min(t / duration, 1)

        # Interpolate each joint
        position_dict = {}
        for joint in desired_position:
            p0 = starting_pose[joint]
            pf = desired_position[joint]
            position_dict[joint] = (1 - alpha) * p0 + alpha * pf

        # Send command
        send_position_command(d, position_dict)
        mujoco.mj_step(m, d)
        
        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()
    
def hold_position(m, d, viewer, duration):
    current_pos = d.qpos.copy()
    current_pos_dict = convert_to_dictionary(current_pos)
    
    start_time = time.time()
    while True:
        t = time.time() - start_time
        if t > duration:
            break
        send_position_command(d, current_pos_dict)
        mujoco.mj_step(m, d)
        viewer.sync()
