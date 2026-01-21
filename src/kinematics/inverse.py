import numpy as np
# Import functions from the forward kinematics module in the same package
from .forward import get_g_w1, get_g_12, get_g_23, get_g_34

# --- Constants (Link Lengths) ---
L1 = 0.0388353
L2 = 0.0624
L3 = 0.0303992
L4 = 0.0542
L5 = 0.11257
L6 = 0.028
L7 = 0.1349
L8 = 0.0611
L9 = 0.1034

def calculate_ik(target_position):
    """
    Geometric Inverse Kinematics for SO-100/101 Arm.
    
    Args:
        target_position (list or np.array): [x, y, z] coordinates
        
    Returns:
        dict: Joint angles in degrees
    """
    X, Y, Z = target_position

    # --- 1. Base Rotation (Theta 1) ---
    theta1 = -np.arctan2(Y, X - L1)

    # --- 2. Geometric Projection for Arm Plane ---
    X_d = X - L1 - L3 * np.cos(-theta1)
    Y_d = Y - L3 * np.sin(-theta1)
    Z_d = Z - L2 - L4 + L8 + L9

    # --- 3. Elbow Geometry (Theta 2 & 3) ---
    l_a = np.sqrt(L5**2 + L6**2)
    l_b = L7
    beta = np.arctan2(L5, L6)
    
    P = np.sqrt(X_d**2 + Y_d**2 + Z_d**2)
    
    # Law of Cosines for Theta 3
    D = (P**2 - l_a**2 - l_b**2) / (2 * l_a * l_b)
    # Clamp D to [-1, 1] to avoid numerical errors
    D = np.clip(D, -1.0, 1.0) 
    
    theta3 = np.arctan2(np.sqrt(1 - D**2), D) - beta

    # Law of Cosines for Theta 2
    D2 = (l_a**2 + P**2 - l_b**2) / (2 * l_a * P)
    D2 = np.clip(D2, -1.0, 1.0)
    
    theta2 = beta - np.arctan2(np.sqrt(1 - D2**2), D2) - np.arctan2(Z_d, np.sqrt(X_d**2 + Y_d**2))

    # --- 4. Wrist Orientation (Theta 4) ---
    # Convert calculated angles to degrees for FK function calls
    th1_deg = np.degrees(theta1)
    th2_deg = np.degrees(theta2)
    th3_deg = np.degrees(theta3)

    # Use Forward Kinematics to find the orientation of the frame before the wrist
    gw1 = get_g_w1()
    g12 = get_g_12(th1_deg)
    g23 = get_g_23(th2_deg)
    g34 = get_g_34(th3_deg)

    g_w4 = gw1 @ g12 @ g23 @ g34
    
    # Extract rotation component to solve for wrist flex
    R11_component = g_w4[1, 1]
    R11_component = np.clip(R11_component, -1.0, 1.0)
    theta4 = np.pi - np.arccos(R11_component)

    # --- 5. Pack Results ---
    joint_config = {
        'shoulder_pan':  th1_deg,
        'shoulder_lift': th2_deg,
        'elbow_flex':    th3_deg,
        'wrist_flex':    np.degrees(theta4),
        'wrist_roll':    th1_deg, # Keeping your logic that roll follows pan
        'gripper':       0.0
    }

    return joint_config