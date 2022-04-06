#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8
import tf
import numpy as np

#flag used to ensure home goal is only sent once
cornerPoints = np.array([[0,0],[0,0],[0,0],[0,0]], dtype = np.float) #np.array([[0,0],[0,0],[0,0],[0,0]], dtype = np.float)
cornerPointsOG = np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]], dtype = np.float)
pointsCounter = 0
oGrid = OccupancyGrid()

def gridCallback(data):
		global cornerPoints, cornerPointsOG, pointsCounter, oGrid

		oGrid.header.stamp = rospy.get_rostime()
		oGrid.header.frame_id='map'
		oGrid.info.resolution = data.info.resolution
		oGrid.info.origin.position.x = data.info.origin.position.x
		oGrid.info.origin.position.y = data.info.origin.position.y
		oGrid.info.width = data.info.width
		oGrid.info.height = data.info.height
		oGrid.info.origin.orientation = data.info.origin.orientation
		oGrid.data = np.zeros(oGrid.info.width*oGrid.info.height, dtype='int8')
		for i in range(0, oGrid.info.width*oGrid.info.height):
			oGrid.data[i] = data.data[i]

		if (pointsCounter >= 4):
			for i in range(0,4):
				cornerPointsOG[i,0] = round((cornerPoints[i,0]-oGrid.info.origin.position.x)/oGrid.info.resolution)
				cornerPointsOG[i,1] = round((cornerPoints[i,1]-oGrid.info.origin.position.y)/oGrid.info.resolution)
			for i in range(0,3):
				cornerPointsOG[i,2] = (cornerPointsOG[i+1,0]-cornerPointsOG[i,0])/abs(cornerPointsOG[i+1,1]-cornerPointsOG[i,1])
				cornerPointsOG[i,3] = (cornerPointsOG[i+1,1]-cornerPointsOG[i,1])/abs(cornerPointsOG[i+1,0]-cornerPointsOG[i,0])
			# cornerPointsOG[3,2] = (cornerPointsOG[0,0]-cornerPointsOG[3,0])/abs(cornerPointsOG[0,1]-cornerPointsOG[3,1])
			# cornerPointsOG[3,3] = (cornerPointsOG[0,1]-cornerPointsOG[3,1])/abs(cornerPointsOG[0,0]-cornerPointsOG[3,0])

			for i in range(0,int(cornerPointsOG[1,1]-cornerPointsOG[0,1])+1):
				xInc = round(float(i)*cornerPointsOG[0,2])
				index = int(oGrid.info.width*(cornerPointsOG[0,1]+i))+int(cornerPointsOG[0,0]+xInc)
				oGrid.data[index] = 100

			for i in range(0,int(cornerPointsOG[2,0]-cornerPointsOG[1,0])+1):
				yInc = round(float(i)*cornerPointsOG[1,3])
				index = int(oGrid.info.width*(cornerPointsOG[1,1]+yInc))+int(cornerPointsOG[1,0]+i)
				oGrid.data[index] = 100

			for i in range(0,int(cornerPointsOG[3,1]-cornerPointsOG[2,1]),-1):
				xInc = round(abs(float(i))*cornerPointsOG[2,2])
				index = int(oGrid.info.width*(cornerPointsOG[2,1]+i))+int(cornerPointsOG[2,0]+xInc)
				oGrid.data[index] = 100

			# for i in range(0,int(cornerPointsOG[0,0]-cornerPointsOG[3,0]),-1):
			# 	yInc = round(abs(float(i))*cornerPointsOG[3,3])
			# 	index = int(round(oGrid.info.width*(cornerPointsOG[3,1]+yInc)+cornerPointsOG[3,0]+i))
			# 	oGrid.data[index] = 100


def pointsCallback(data):
	global xPoint, yPoint, pointsCounter, cornerPoints
	if(data.data==pointsCounter-1):
		cornerPoints[pointsCounter,0] = xPoint
		cornerPoints[pointsCounter,1] = yPoint
		pointsCounter+=1

# Function to send the robot the origin as a goal when exploration is complete
def ogUpdater():
	global oGrid, xPoint, yPoint
    # Initialize ros subscriber to move_base/cancel
	rospy.init_node('occupancy_grid_updater', anonymous=True)
	listener = tf.TransformListener()
	# Create a subscriber to move_base/cancel topic
	rospy.Subscriber('rtabmap/grid_map', OccupancyGrid, gridCallback)
	rospy.Subscriber('dwCorners', Int8, pointsCallback)
    
	# Initialize ROS publisher to move_base_simple/goal
	pub = rospy.Publisher('final_grid_map', OccupancyGrid, queue_size=10)
    
	rate = rospy.Rate(1)
    
	# Loop to keep the nodes going
	while not rospy.is_shutdown():
		try:
			(trans, rot) = listener.lookupTransform('/camera_link', '/map', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		xPoint = trans[0]
		yPoint = trans[1]
		print(xPoint)
		print(yPoint)
		pub.publish(oGrid) 	
		rate.sleep()

if __name__ == '__main__':
    try:
        ogUpdater()
    except rospy.ROSInterruptException:
        pass
