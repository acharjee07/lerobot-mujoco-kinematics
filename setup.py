from setuptools import setup, find_packages

setup(
    name="lerobot_kinematics",
    version="0.1.0",
    description="A Sim2Real Kinematics library for SO-100 robots using MuJoCo and LeRobot",
    author="Sanjay Acharjee",
    packages=find_packages(),  # This automatically finds the 'src' folder
    python_requires=">=3.10",
    install_requires=[
        "numpy",
        "sympy",
        "mujoco",
        "opencv-python",
        "pyyaml",
        # We don't list 'lerobot' here because it's usually installed from source/git
    ],
)