import time
from src.hardware.hardware_utils import load_calibration, move_to_pose, hold_position, setup_motors
from src.kinematics.inverse import calculate_ik
from src.config import robot_name, port

# Configuration
PORT_ID = port
ROBOT_NAME = robot_name
MOVE_TIME = 2.0
HOLD_TIME = 4.0

def main():
    # 1. Setup Robot
    calibration = load_calibration(ROBOT_NAME)
    bus = setup_motors(calibration, PORT_ID)
    
    # 2. Read Start Pose
    starting_pose = bus.sync_read("Present_Position")
    print("Starting Pose:", starting_pose)

    # 3. Calculate IK
    desired_ee = [0.2, 0.2, 0]
    print(f"Moving to: {desired_ee}")
    
    try:
        desired_configuration = calculate_ik(desired_ee)
        
        # 4. Move Robot
        move_to_pose(bus, desired_configuration, MOVE_TIME)
        hold_position(bus, HOLD_TIME)

        # 5. Return Home
        print("Returning home...")
        move_to_pose(bus, starting_pose, MOVE_TIME)

    finally:
        # Safety: Always disable torque on exit
        bus.disable_torque()
        print("Torque disabled.")

if __name__ == "__main__":
    main()