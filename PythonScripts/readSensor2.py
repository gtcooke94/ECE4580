#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class DisplayData():
	def __init__(self):
		# initiliaze
		rospy.init_node('DisplayData', anonymous=False)
		# What function to call when you ctrl + c    
		rospy.on_shutdown(self.shutdown)    
		self.bridge = CvBridge()
		rospy.loginfo('flag1')
		rospy.Subscriber("/camera/depth/image/", Image, self.depthCallback)    
		rospy.loginfo('flag2')		
		#rospy.spin() tells the program to not exit until you press ctrl + c.  If this wasn't there... it'd subscribe to /laptop_charge/ then immediatly exit (therefore stop "listening" to the thread).
		rospy.spin();


	def depthCallback(self, data):
		rospy.loginfo('flagCallback')
		cv_image = self.bridge.imgmsg_to_cv2(data)
		dst = cv_image
		cv_image = cv2.normalize(cv_image, dst, 0, 1, cv2.NORM_MINMAX)
		cv2.imshow("Image Window 2", cv_image)
		cv2.waitKey(3)

        
	def shutdown(self):
    	# stop turtlebot
		rospy.loginfo("Stop TurtleBot")
		rospy.loginfo("Stopping code -- Connnor")
		# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
		#self.cmd_vel.publish(Twist())
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
		rospy.sleep(1)
 
if __name__ == '__main__':
	try:
		DisplayData()
	except:
		rospy.loginfo("DisplayData node terminated.")

