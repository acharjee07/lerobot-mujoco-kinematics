import sympy as sp

# --- 1. Define Symbolic Variables ---

# Joint Angles (The variables we differentiate with respect to)
theta1, theta2, theta3, theta4, theta5 = sp.symbols('theta1 theta2 theta3 theta4 theta5')

# Link Lengths (Constants we will substitute later)
l1, l2, l3, l4, l5, l6, l7, l8, l9 = sp.symbols('l1 l2 l3 l4 l5 l6 l7 l8 l9')

# --- 2. Helper Functions ---

def dh_matrix_sym(theta, d, a, alpha):
    """
    Creates the symbolic Denavit-Hartenberg transformation matrix.
    Standard DH: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    """
    c_th = sp.cos(theta)
    s_th = sp.sin(theta)
    c_al = sp.cos(alpha)
    s_al = sp.sin(alpha)

    return sp.Matrix([
        [c_th, -s_th * c_al,  s_th * s_al, a * c_th],
        [s_th,  c_th * c_al, -c_th * s_al, a * s_th],
        [0,     s_al,         c_al,        d       ],
        [0,     0,            0,           1       ]
    ])

# --- 3. Construct Transformations (Matching forward.py) ---

# Offsets derived from geometry (e.g., bent elbow structure)
# Corresponds to: np.arctan2(0.11257, 0.028)
angle_offset = sp.atan2(l5, l6) 

# Transformation 1: World -> Joint 1
# Params: d=l2, a=l1, alpha=-pi
T_w1 = dh_matrix_sym(0, l2, l1, -sp.pi)

# Transformation 2: Joint 1 -> Joint 2
# Params: d=-l4, a=l3, alpha=pi/2
# Theta: theta1
T_12 = dh_matrix_sym(theta1, -l4, l3, sp.pi/2)

# Transformation 3: Joint 2 -> Joint 3
# Params: d=0, a=-sqrt(l5^2 + l6^2), alpha=0
# Theta: theta2 (with offset logic matching forward.py: pi - offset + theta)
a_23 = -sp.sqrt(l5**2 + l6**2)
th_23 = sp.pi - angle_offset + theta2
T_23 = dh_matrix_sym(th_23, 0, a_23, 0)

# Transformation 4: Joint 3 -> Joint 4
# Params: d=0, a=-l7, alpha=0
# Theta: theta3 (with offset logic: offset + theta)
th_34 = angle_offset + theta3
T_34 = dh_matrix_sym(th_34, 0, -l7, 0)

# Transformation 5: Joint 4 -> Joint 5 (Wrist Flex)
# Params: d=0, a=0, alpha=pi/2
# Theta: theta4 (logic: pi/2 + theta)
th_45 = (sp.pi/2) + theta4
T_45 = dh_matrix_sym(th_45, 0, 0, sp.pi/2)

# Transformation 6: Joint 5 -> Tool Tip (Wrist Roll)
# Params: d=-(l8+l9), a=0, alpha=-pi/2
# Theta: theta5 (logic: pi/2 - theta)
th_5t = (sp.pi/2) - theta5
d_5t = -(l8 + l9)
T_5t = dh_matrix_sym(th_5t, d_5t, 0, -sp.pi/2)

# --- 4. Compute Forward Kinematics Chain ---

# Full transformation from World to Tool
T_total = T_w1 * T_12 * T_23 * T_34 * T_45 * T_5t

# Extract the Position Vector (x, y, z)
# This is the last column, first 3 rows
pos_vector = T_total[0:3, 3]

# --- 5. Compute the Jacobian ---

print("Deriving Symbolic Jacobian... (This runs on import, but is fast for small chains)")

# The Jacobian is the partial derivative of the Position Vector 
# with respect to each joint angle variable.
# Size: 3x5 (3 spatial coords, 5 joints)
J_total = pos_vector.jacobian([theta1, theta2, theta3, theta4, theta5])

# This J_total is what we import in velocity.py