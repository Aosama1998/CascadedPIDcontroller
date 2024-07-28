#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class JointMover:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('joint_mover', anonymous=True)
        
        # Publisher for the prismatic joint position
        self.pub = rospy.Publisher('/joint1_position_controller/command', Float64, queue_size=10)
        
        # Subscriber to the joint states
        self.sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # Variables to store the position and effort of J_1
        self.position = None
        self.effort = None

    def move_joint(self, position):
        # Publish the desired position for J_1
        rospy.sleep(1)  # Wait for the publisher to establish connection
        self.pub.publish(Float64(position))
        rospy.loginfo(f'Moving J_1 to position {position}')
        
    def joint_state_callback(self, msg):
        # Check if J_1 is in the joint names list
        if 'J_1' in msg.name:
            index = msg.name.index('J_1')
            self.position = msg.position[index]
            self.effort = msg.effort[index]
            rospy.loginfo(f'Position of J_1: {self.position}')
            rospy.loginfo(f'Effort of J_1: {self.effort}')
            
            # Log the position and effort to a file
            with open('/tmp/joint1_state.log', 'a') as log_file:
                log_file.write(f'Position of J_1: {self.position}, Effort of J_1: {self.effort}\n')
                
    def run(self):
        # Move the joint to the desired position
        self.move_joint(0.05)
        
        # Keep the node alive to listen to joint states
        rospy.spin()

if __name__ == '__main__':
    try:
        joint_mover = JointMover()
        joint_mover.run()
    except rospy.ROSInterruptException:
        pass

