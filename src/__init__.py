# src/__init__.py

from .kinematics.forward import calculate_fk
from .kinematics.inverse import calculate_ik

# Expose helper functions if needed
from .kinematics.forward import (
    get_g_w1, 
    get_g_12, 
    get_g_23, 
    get_g_34, 
    get_g_45, 
    get_g_5t
)