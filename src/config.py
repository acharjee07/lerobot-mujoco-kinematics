import yaml
import os

def load_config():
    """Load configuration from configs/env_config.yaml"""
    config_path = os.path.join(os.path.dirname(__file__), '..', 'configs', 'env_config.yaml')
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

# Load config once
config = load_config()

# Extract values for easy importing
robot_name = config['robot_name']
port = config['port']
camera_index = config['camera_index']
resolution_width = config['resolution_width']
resolution_height = config['resolution_height']
deadband = config['deadband']
gain_x = config['gain_x']
gain_y = config['gain_y']