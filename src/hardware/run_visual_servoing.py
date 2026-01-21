import time
import numpy as np
import sympy as sp
import cv2
from src.hardware.hardware_utils import load_calibration, setup_motors
from src.kinematics import get_forward_kinematics
from src.kinematics.jacobian_symbolic import J_total, theta1, theta2, theta3, theta4, theta5
from src.kinematics.jacobian_symbolic import l1, l2, l3, l4, l5, l6, l7, l8, l9 
from src.vision.detector import get_red_object
from src.config import port as PORT_ID, robot_name as ROBOT_NAME, camera_index as CAMERA_INDEX

# ==========================================
# 1. CONFIGURATION
# ==========================================
# --- VISION PARAMETERS ---
OPTICAL_CENTER = (320, 240)  # (cx, cy)

# --- WINDOW / DEADBAND SETTINGS ---
# The robot will STOP if the object is within this many pixels of the center.
# Increase this if it still jitters. Decrease for more precision.
DEADBAND = 50  

# --- DIRECTION TUNING ---
INVERT_X = -1.0  
INVERT_Y = -1.0  

# --- CONTROL GAINS ---
GAIN_X = 0.001   
GAIN_Y = 0.001   

# --- ROBOT LIMITS ---
MAX_VEL_DEG = 10         
JOINT_LIMITS = {
    'shoulder_pan':  (-120, 120),
    'shoulder_lift': (-110, 110), 
    'elbow_flex':    (-110, 110),
    'wrist_flex':    (-110, 110),
    'wrist_roll':    (-150, 150)
}

# --- CAMERA TO END-EFFECTOR TRANSFORM ---
R_CAM_TO_EE = np.array([
    [-1, 0, 0],   # Robot X (Forward): OFF (0)
    [0, 0, 0],   # Robot Y (Left/Right): Controlled by Camera X
    [0, 1, 0]    # Robot Z (Up/Down): Controlled by Camera Y
])

# ==========================================
# 2. JACOBIAN SETUP
# ==========================================
print("Compiling Jacobian...")
link_values = {
    l1: 0.0388353, l2: 0.0624, l3: 0.0303992, l4: 0.0542,
    l5: 0.11257, l6: 0.028, l7: 0.1349, l8: 0.0611, l9: 0.1034
}
J_with_constants = J_total.subs(link_values)
calculate_J_fast = sp.lambdify(
    (theta1, theta2, theta3, theta4, theta5), 
    J_with_constants, "numpy"
)

def get_simple_error(cap):
    """ Returns X and Y pixel error. """
    ret, frame = cap.read()
    if not ret: return None, False, None
    
    frame = cv2.resize(frame, (640, 480))
    cx, cy, area, found, debug_frame = get_red_object(frame, draw_debug=True)
    cx_opt, cy_opt = OPTICAL_CENTER
    
    if not found:
        return 0, 0, False, debug_frame

    error_x = (cx - cx_opt)
    error_y = (cy - cy_opt)
    
    return error_x, error_y, True, debug_frame

def main():
    print("Connecting to Robot...")
    calibration = load_calibration(ROBOT_NAME)
    bus = setup_motors(calibration, PORT_ID)
    
    print(f"Opening Camera Index: {CAMERA_INDEX}")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    for motor in bus.motors:
        bus.write("P_Coefficient", motor, 25)   
        bus.write("D_Coefficient", motor, 15)  
        bus.write("Torque_Enable", motor, 1)

    q_actual = bus.sync_read("Present_Position")
    q_virtual = q_actual.copy()
    
    print("\n>>> TRACKING STARTED with DEADBAND.")
    gui_available = True
    time.sleep(1.0)

    try:
        while True:
            loop_start = time.time()
            
            # 1. GET VISION DATA
            e_x, e_y, found, frame = get_simple_error(cap)
            
            # 2. CALCULATE VELOCITY
            v_cam = np.zeros(3)
            
            if found:
                # --- DEADBAND LOGIC ---
                # Check if object is inside the "Stop Window"
                if abs(e_x) < DEADBAND and abs(e_y) < DEADBAND:
                    v_cam = np.zeros(3) # STOP
                    status_msg = "Target in Window (HOLDING)"
                else:
                    # Move if outside window
                    v_cam[0] = -e_x * GAIN_X * INVERT_X
                    v_cam[1] = -e_y * GAIN_Y * INVERT_Y
                    status_msg = "Target Outside (MOVING)"
                
                v_cam[2] = 0.0 # Depth locked

                # 3. ROBOT KINEMATICS
                q_actual = bus.sync_read("Present_Position")
                curr_pos, R_0_EE = get_forward_kinematics(q_actual) 
                
                v_base = R_0_EE @ (R_CAM_TO_EE @ v_cam)
                
                th_vals = [np.deg2rad(q_actual[k]) for k in ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']]
                J = calculate_J_fast(*th_vals)
                J_pos = J[0:3, :]
                
                q_dot_rad = np.linalg.pinv(J_pos, rcond=1e-2) @ v_base
                raw_q_dot_deg = np.degrees(q_dot_rad)
                
                print(f"Err: {int(e_x)}, {int(e_y)} | {status_msg}".ljust(60), end='\r')
            else:
                raw_q_dot_deg = np.zeros(5)
                if not gui_available: print("Target Lost...".ljust(60), end='\r')

            # 4. GUI VISUALIZATION
            if frame is not None and gui_available:
                cx, cy = OPTICAL_CENTER
                
                # Draw the Deadband Window (Blue Box)
                # If target is inside this box, robot stops.
                top_left = (cx - DEADBAND, cy - DEADBAND)
                bottom_right = (cx + DEADBAND, cy + DEADBAND)
                cv2.rectangle(frame, top_left, bottom_right, (255, 0, 0), 2)
                
                # Center Cross
                cv2.line(frame, (cx-10, cy), (cx+10, cy), (0, 255, 0), 1)
                cv2.line(frame, (cx, cy-10), (cx, cy+10), (0, 255, 0), 1)
                
                if found:
                    # Draw Arrow only if moving
                    if np.linalg.norm(v_cam) > 0:
                        scale = 2000
                        start = (cx, cy)
                        end = (int(cx + v_cam[0]*scale), int(cy + v_cam[1]*scale))
                        cv2.arrowedLine(frame, start, end, (0, 255, 255), 4, tipLength=0.3)

                try:
                    cv2.imshow("Deadband Tracker", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'): break
                except: gui_available = False

            # 5. SEND TO MOTORS
            safe_vel = np.clip(raw_q_dot_deg, -MAX_VEL_DEG, MAX_VEL_DEG)
            keys = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
            for i, key in enumerate(keys):
                q_virtual[key] += safe_vel[i] * 0.05 
                min_l, max_l = JOINT_LIMITS[key]
                q_virtual[key] = np.clip(q_virtual[key], min_l, max_l)
            
            bus.sync_write("Goal_Position", q_virtual, normalize=True)
            
            elapsed = time.time() - loop_start
            if elapsed < 0.03: time.sleep(0.03 - elapsed)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        for motor in bus.motors: bus.write("Torque_Enable", motor, 0)

if __name__ == "__main__":
    main()