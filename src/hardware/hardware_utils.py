# so101-utils.py
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from pathlib import Path
import draccus
import time
import os
import json
import os
import json

# --- ADD THIS HELPER CLASS ---
class MotorConfig:
    """
    A simple wrapper to convert dictionary keys into object attributes.
    Example: data['range_min'] becomes data.range_min
    """
    def __init__(self, **entries):
        self.__dict__.update(entries)
    
    def __repr__(self):
        return str(self.__dict__)

def load_calibration(robot_name):
    """
    Loads the robot calibration file and converts it to objects.
    """
    calibration_dir = "calibration_files"
    file_name = f"{robot_name}.json"
    fpath = os.path.join(calibration_dir, file_name)
    
    print(f"Loading calibration from: {fpath}")

    if not os.path.exists(fpath):
        raise FileNotFoundError(f"Missing calibration file: {fpath}")

    with open(fpath, 'r') as f:
        raw_data = json.load(f)
        
    # --- CONVERT DICT TO OBJECTS HERE ---
    # The library expects: calibration['motor_name'].range_min
    calibration = {}
    for motor_name, motor_data in raw_data.items():
        # Wrap the inner dictionary in our MotorConfig class
        calibration[motor_name] = MotorConfig(**motor_data)
        
    return calibration

def setup_motors(calibration, PORT_ID):
    norm_mode_body = MotorNormMode.DEGREES
    bus = FeetechMotorsBus(
                port=PORT_ID,
                motors={
                    "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                    "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                    "elbow_flex": Motor(3, "sts3215", norm_mode_body),
                    "wrist_flex": Motor(4, "sts3215", norm_mode_body),
                    "wrist_roll": Motor(5, "sts3215", norm_mode_body),
                    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
                },
                calibration=calibration,
            )
    bus.connect(True)

    with bus.torque_disabled():
        bus.configure_motors()
        for motor in bus.motors:
            bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            bus.write("P_Coefficient", motor, 16)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            bus.write("I_Coefficient", motor, 0)
            bus.write("D_Coefficient", motor, 32) 
    return bus


def move_to_pose(bus, desired_position, duration):
    start_time = time.time()
    starting_pose = bus.sync_read("Present_Position")
    
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
        bus.sync_write("Goal_Position", position_dict, normalize=True)

        # (Optional) Read back
        present_pos = bus.sync_read("Present_Position")
        print(present_pos)

        time.sleep(0.02)  # 50 Hz loop
    
def hold_position(bus, duration):
    start_time = time.time()
    while True:
        t = time.time() - start_time
        if t > duration:
            break
        time.sleep(0.02)  # 50 Hz loop


import time
        
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


