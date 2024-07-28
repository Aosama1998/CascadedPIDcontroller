#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pynput import keyboard

# Joint names as defined in your configuration
joint_names = [
    'J_1', 'J_2', 'J_4', 'J_6', 'J_7',
    'J_10', 'J_12', 'J_14', 'J_15', 'J_16'
]

# Initialize joint positions
joint_positions = [0.0] * len(joint_names)

# Define key bindings for each joint
key_bindings = {
    'q': (0, 0.01), 'a': (0, -0.01),  # Small increment for prismatic joint J_1
    'w': (1, 0.01), 's': (1, -0.01),  # Small increment for prismatic joint J_2
    'e': (2, 0.1), 'd': (2, -0.1),    # Increment for revolute joint J_4
    'r': (3, 0.1), 'f': (3, -0.1),    # Increment for revolute joint J_6
    't': (4, 0.1), 'g': (4, -0.1),    # Increment for revolute joint J_7
    'y': (5, 0.1), 'h': (5, -0.1),    # Increment for revolute joint J_10
    'u': (6, 0.1), 'j': (6, -0.1),    # Increment for revolute joint J_12
    'i': (7, 0.1), 'k': (7, -0.1),    # Increment for revolute joint J_14
    'o': (8, 0.1), 'l': (8, -0.1),    # Increment for revolute joint J_15
    'p': (9, 0.1), ';': (9, -0.1)     # Increment for revolute joint J_16
}

def send_joint_trajectory():
    # Create a JointTrajectory message
    traj = JointTrajectory()
    traj.joint_names = joint_names

    # Create a JointTrajectoryPoint message
    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.time_from_start = rospy.Duration(1.0)

    traj.points.append(point)
    pub.publish(traj)

def on_press(key):
    try:
        char = key.char
        if char in key_bindings:
            joint, change = key_bindings[char]
            joint_positions[joint] += change
            print(f"Joint {joint_names[joint]}: {joint_positions[joint]:.2f}")
            send_joint_trajectory()
    except AttributeError:
        pass

def main():
    global pub
    rospy.init_node('keyboard_joint_control')

    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

    print("Use the following keys to control the robot's joints:")
    for key, (joint, change) in key_bindings.items():
        print(f"{key}: Joint {joint_names[joint]} {'+' if change > 0 else '-'} {abs(change)}")

    with keyboard.Listener(on_press=on_press) as listener:
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

