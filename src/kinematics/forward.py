import numpy as np

# --- 1. Robot Physical Parameters (The DH Table) ---
# Defining these at the top makes it easy to tweak calibration later
# specific to your hardware.
DH_PARAMS = {
    'w1': {'d': 0.0624,  'a': 0.0388353, 'alpha': -np.pi},
    '12': {'d': -0.0542, 'a': 0.0303992, 'alpha': np.pi / 2},
    '23': {'d': 0.0,     'a': -np.sqrt(0.11257**2 + 0.028**2), 'alpha': 0.0},
    '34': {'d': 0.0,     'a': -0.1349,   'alpha': 0.0},
    '45': {'d': 0.0,     'a': 0.0,       'alpha': np.pi / 2},
    '5t': {'d': -0.1645, 'a': 0.0,       'alpha': -np.pi / 2}, # Combined d values (-0.1034 - .0611)
}

# Calculated Offsets
THETA2_OFFSET = np.arctan2(0.11257, 0.028)
THETA3_OFFSET = np.arctan2(0.11257, 0.028)


# --- 2. Core Math Helper Functions ---

def dh_matrix(theta_rad, d, a, alpha_rad):
    """
    Calculates the standard Denavit-Hartenberg (DH) transformation matrix.
    Transformation: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    """
    c_th = np.cos(theta_rad)
    s_th = np.sin(theta_rad)
    c_al = np.cos(alpha_rad)
    s_al = np.sin(alpha_rad)

    return np.array([
        [c_th, -s_th * c_al,  s_th * s_al, a * c_th],
        [s_th,  c_th * c_al, -c_th * s_al, a * s_th],
        [0,     s_al,         c_al,        d       ],
        [0,     0,            0,           1       ]
    ])


# --- 3. Joint Transformations ---

def get_g_w1():
    # World -> Joint 1
    p = DH_PARAMS['w1']
    return dh_matrix(0, p['d'], p['a'], p['alpha'])

def get_g_12(theta_deg):
    # Joint 1 -> Joint 2
    p = DH_PARAMS['12']
    return dh_matrix(np.deg2rad(theta_deg), p['d'], p['a'], p['alpha'])

def get_g_23(theta_deg):
    # Joint 2 -> Joint 3
    p = DH_PARAMS['23']
    theta_rad = np.pi - THETA2_OFFSET + np.deg2rad(theta_deg)
    return dh_matrix(theta_rad, p['d'], p['a'], p['alpha'])

def get_g_34(theta_deg):
    # Joint 3 -> Joint 4
    p = DH_PARAMS['34']
    theta_rad = THETA3_OFFSET + np.deg2rad(theta_deg)
    return dh_matrix(theta_rad, p['d'], p['a'], p['alpha'])

def get_g_45(theta_deg):
    # Joint 4 -> Joint 5
    p = DH_PARAMS['45']
    theta_rad = (np.pi / 2) + np.deg2rad(theta_deg)
    return dh_matrix(theta_rad, p['d'], p['a'], p['alpha'])

def get_g_5t(theta_deg):
    # Joint 5 -> Tool Tip
    p = DH_PARAMS['5t']
    theta_rad = (np.pi / 2) - np.deg2rad(theta_deg)
    return dh_matrix(theta_rad, p['d'], p['a'], p['alpha'])


# --- 4. Main Solver ---

def calculate_fk(position_dict):
    """
    Calculates the Forward Kinematics.
    
    Args:
        position_dict (dict): Keys must match: 
                              ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 
                               'wrist_flex', 'wrist_roll']
    
    Returns:
        tuple: (position [x,y,z], rotation [3x3 matrix])
    """
    # 1. Compute individual transforms
    gw1 = get_g_w1()
    g12 = get_g_12(position_dict['shoulder_pan'])
    g23 = get_g_23(position_dict['shoulder_lift'])
    g34 = get_g_34(position_dict['elbow_flex'])
    g45 = get_g_45(position_dict['wrist_flex'])
    g5t = get_g_5t(position_dict['wrist_roll'])

    # 2. Chain them: T_world_tool = T_w1 * T_12 * ... * T_5t
    gwt = gw1 @ g12 @ g23 @ g34 @ g45 @ g5t
    
    position = gwt[0:3, 3]
    rotation = gwt[0:3, 0:3]
    
    return position, rotation