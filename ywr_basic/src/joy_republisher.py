#!/usr/bin/env python
import rospy, math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

#flag used to ensure home goal is only sent once
vel = Twist()

def joyCallback(data):
    global vel
    vel.linear.y = data.axes[1]
    vel.linear.x = data.axes[0]

# Function to send the robot the origin as a goal when exploration is complete
def joyRepublisher():
	global vel
    # Initialize ros subscriber to move_base/cancel
	rospy.init_node('joy_republisher', anonymous=True)
    
	# Create a subscriber to move_base/cancel topic
	rospy.Subscriber('joy', Joy, joyCallback)
    
	# Initialize ROS publisher to move_base_simple/goal
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=20)
    
	rate = rospy.Rate(20)
    
	# Loop to keep the nodes going
	while not rospy.is_shutdown():
		pub.publish(vel) 	
		rate.sleep()

if __name__ == '__main__':
    try:
        joyRepublisher()
    except rospy.ROSInterruptException:
        pass