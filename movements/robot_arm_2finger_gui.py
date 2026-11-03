"""Launch the kinematics GUI with a symmetric two-finger gripper."""

import tkinter as tk

import robot_arm as ra
from robot_arm_gui import RobotArmApp


def main() -> None:
    root = tk.Tk()
    RobotArmApp(root, config=ra.RobotConfig(gripper_fingers=2))
    root.mainloop()


if __name__ == "__main__":
    main()
