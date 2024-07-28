#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pynput import keyboard

# Joint names as defined in your configuration
joint_names = [
    'R_thumb_proximal_yaw_joint', 'R_thumb_proximal_pitch_joint', 
    'R_thumb_intermediate_joint', 'R_thumb_distal_joint', 
    'R_index_proximal_joint', 'R_index_intermediate_joint', 
    'R_middle_proximal_joint', 'R_middle_intermediate_joint', 
    'R_ring_proximal_joint', 'R_ring_intermediate_joint', 
    'R_pinky_proximal_joint', 'R_pinky_intermediate_joint'
]

# Initialize joint positions
joint_positions = [0.0] * len(joint_names)

# Define key bindings for each joint
key_bindings = {
    'q': (0, 0.01), 'a': (0, -0.01),  # Increment for R_thumb_proximal_yaw_joint
    'w': (1, 0.01), 's': (1, -0.01),  # Increment for R_thumb_proximal_pitch_joint
    'e': (2, 0.1), 'd': (2, -0.1),    # Increment for R_thumb_intermediate_joint
    'r': (3, 0.1), 'f': (3, -0.1),    # Increment for R_thumb_distal_joint
    't': (4, 0.1), 'g': (4, -0.1),    # Increment for R_index_proximal_joint
    'y': (5, 0.1), 'h': (5, -0.1),    # Increment for R_index_intermediate_joint
    'u': (6, 0.1), 'j': (6, -0.1),    # Increment for R_middle_proximal_joint
    'i': (7, 0.1), 'k': (7, -0.1),    # Increment for R_middle_intermediate_joint
    'o': (8, 0.1), 'l': (8, -0.1),    # Increment for R_ring_proximal_joint
    'p': (9, 0.1), ';': (9, -0.1),    # Increment for R_ring_intermediate_joint
    'z': (10, 0.1), 'x': (10, -0.1),  # Increment for R_pinky_proximal_joint
    'c': (11, 0.1), 'v': (11, -0.1)   # Increment for R_pinky_intermediate_joint
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

