import time
import numpy as np
from src.hardware.hardware_utils import load_calibration, setup_motors
from src.kinematics import get_forward_kinematics, calculate_velocity_step
from src.config import robot_name, port

# Hardware Settings
PORT_ID = port
ROBOT_NAME = robot_name
TARGET_POS = np.array([0.30, 0.30, 0.30])

# Control Tunings
VELOCITY_SMOOTHING = 0.2
MAX_STEP_SIZE_DEG = 2.0
MIN_STEP_SIZE_DEG = 0.01
SLOW_DOWN_RADIUS = 0.08
MAX_LAG_DEG = 5.0
JOINT_LIMITS = {
    'shoulder_pan': (-120, 120), 'shoulder_lift': (-110, 110),
    'elbow_flex': (-110, 110), 'wrist_flex': (-110, 110), 'wrist_roll': (-150, 150)
}

def main():
    calibration = load_calibration(ROBOT_NAME)
    bus = setup_motors(calibration, PORT_ID)
    
    print("Configuring Motors (Soft & Damped)...")
    for motor in bus.motors:
        bus.write("P_Coefficient", motor, 25)
        bus.write("D_Coefficient", motor, 15)
        bus.write("Torque_Enable", motor, 1)

    # State tracking
    q_actual = bus.sync_read("Present_Position")
    q_virtual = q_actual.copy() # The "Commanded" position
    prev_q_dot_deg = np.zeros(5)

    print(f"\n>>> MOVING TO: {TARGET_POS}")
    time.sleep(1.0)

    try:
        while True:
            loop_start = time.time()
            
            # 1. Read State & FK
            q_actual = bus.sync_read("Present_Position")
            curr_pos, _ = get_forward_kinematics(q_actual)
            
            # 2. Calculate Velocity (Shared Logic)
            # Note: We don't use 'dt' inside the calculator for pure velocity vector,
            # so we handle the step scaling manually below as per your logic.
            q_dot_rad, error_vec = calculate_velocity_step(
                q_actual, TARGET_POS, curr_pos, dt=1.0 
            )
            raw_q_dot_deg = np.degrees(q_dot_rad)
            
            dist = np.linalg.norm(error_vec)
            print(f"Dist: {dist:.4f} m", end='\r')
            
            if dist < 0.05:
                print("\nTarget Reached!")
                break
                
            # 3. Velocity Smoothing (Low Pass Filter)
            filtered_q_dot = (VELOCITY_SMOOTHING * raw_q_dot_deg) + ((1.0 - VELOCITY_SMOOTHING) * prev_q_dot_deg)
            prev_q_dot_deg = filtered_q_dot
            
            # 4. Adaptive Speed Scaling
            speed_ratio = np.clip(dist / SLOW_DOWN_RADIUS, 0.0, 1.0)
            current_limit = MIN_STEP_SIZE_DEG + (MAX_STEP_SIZE_DEG - MIN_STEP_SIZE_DEG) * speed_ratio
            
            max_val = np.max(np.abs(filtered_q_dot))
            if max_val > 1e-6:
                scale = current_limit / max_val if (current_limit / max_val) < 1.0 else 1.0
                q_step_deg = filtered_q_dot * scale
            else:
                q_step_deg = np.zeros(5)

            # 5. Virtual Target Updates (Lag Control)
            keys = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
            for i, key in enumerate(keys):
                proposed = q_virtual[key] + q_step_deg[i]
                
                # Prevent virtual target from running too far ahead of real robot
                lag = proposed - q_actual[key]
                if abs(lag) > MAX_LAG_DEG:
                    q_virtual[key] = q_actual[key] + np.sign(lag) * MAX_LAG_DEG
                else:
                    q_virtual[key] = proposed
                
                # Hard Limits
                min_l, max_l = JOINT_LIMITS[key]
                q_virtual[key] = np.clip(q_virtual[key], min_l, max_l)
            
            # 6. Write Command
            bus.sync_write("Goal_Position", q_virtual, normalize=True)
            
            # 7. Loop Rate
            elapsed = time.time() - loop_start
            if elapsed < 0.02: time.sleep(0.02 - elapsed)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("\nHolding Position...")
        # (Add your hold logic here)
        bus.disable_torque()

if __name__ == "__main__":
    main()