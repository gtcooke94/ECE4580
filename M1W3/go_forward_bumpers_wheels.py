
#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution & use in source & binary forms, with | without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions & the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions & the following disclaimer in the documentation &/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse | promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS & CONTRIBUTORS "AS IS" & ANY EXPRESS | IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY & FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER | CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, | CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS | SERVICES; LOSS OF USE, DATA, | PROFITS; | BUSINESS INTERRUPTION) HOWEVER CAUSED & ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, | TORT (INCLUDING NEGLIGENCE | OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent

class GoForward():
	def __init__(self):
		# initiliaze
		rospy.init_node('GoForward', anonymous=False)
		rospy.loginfo("Line 33")

		# What function to call when you ctrl + c	
		rospy.on_shutdown(self.shutdown)
		rospy.loginfo("Line 37")
		
		# Create a publisher which can "talk" to TurtleBot & tell it to move
		# This will tell it to move slowly then stop when there is a bumper | wheel event
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		rospy.loginfo("Line 42")

	 	
		rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.BumperEventCallback)
		rospy.Subscriber("/mobile_base/events/wheel_drop", WheelDropEvent, self.WheelDropEventCallback)
		rospy.loginfo("Line 46")
		# Define the states for the state machine

		# bhit is bumper hit. The most significant bit is the left bumper, the next is the middle bumper, the next is the right bumper, the next is the left wheel, & the least significant bit is the right wheel. 1 Means it's been pushed | the wheels are dropped, 0 means not hit & wheels are not dropped
		self.bhit = 0b00000
		# self.bhit will be made up of self.bumpers & self.wheels
		self.bumpers = 0b000
		self.wheels = 0b00
		rospy.loginfo("Line 48")		

		# Safety states are 0, 1, & 2.
		# 0 means that the robot is moving forward
		# 1 means that the robot bumper has been triggered
		# If bumpers cease to be hit in state 1, then state 2 tirggers which waits two seconds
		# Getting through 2 seconds of state 2 moves to state 0
		self.safety = 0b00000

		#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ

		# Twist is a datatype for velocity
		move_cmd = Twist()
		# let's go forward at 0.2 m/s
		move_cmd.linear.x = 0.2
		# let's turn at 0 radians/s
		move_cmd.angular.z = 0

		
			
			

		# as long as you haven't ctrl + c keeping doing...
		while not rospy.is_shutdown():
			# Make the state machine here

			if (self.safety == 0):
				rospy.loginfo("In state 0")
				if (self.bhit == 0):
					self.cmd_vel.publish(move_cmd)
				if (self.bhit > 0):
					self.safety = 1
			if (self.safety == 1):
				rospy.loginfo("In state 1")
				rospy.loginfo("%s"%(bin(self.bhit)))
				self.cmd_vel.publish(Twist())
				if (self.bhit == 0):
					self.safety = 2
			if (self.safety == 2):
				rospy.loginfo("In state 2")
				rospy.sleep(2.)
				self.safety = 0

				

		# Is this necessary?
		rospy.spin()
						
		
	def BumperEventCallback(self, data):
		rospy.loginfo("Bumper Event Callback")
		# Print out what happened & change the state variable based on what happened
		if (data.state == BumperEvent.PRESSED) :
			state = "pressed"
			if (data.bumper == BumperEvent.LEFT) :
				bumper = "left"
				self.bhit = self.bhit | 0b10000
			elif (data.bumper == BumperEvent.RIGHT) :
				bumper = "right"
				self.bhit = self.bhit | 0b00100
			else:
				bumper = "center"
				self.bhit = self.bhit | 0b01000
		else:
			state = "released"
			if (data.bumper == BumperEvent.LEFT) :
				bumper = "left"
				self.bhit = self.bhit & 0b01111
			elif (data.bumper == BumperEvent.RIGHT) :
				bumper = "right"
				self.bhit = self.bhit & 0b11011
			else:
				bumper = "center"
				self.bhit = self.bhit & 0b10111
		rospy.loginfo("Bumper %s was %s."%(bumper, state))	
		
	
	def WheelDropEventCallback(self, data):
		rospy.loginfo("Wheeldrop Event Callback")
		if (data.state == WheelDropEvent.RAISED):
			state = "raised"
			if (data.wheel == WheelDropEvent.LEFT):
				self.bhit= self.bhit & 0b11101
				wheel = "left"
			else:
				self.bhit= self.bhit & 0b11110
				wheel = "right"
		else:
			state = "dropped"
			if (data.wheel == WheelDropEvent.LEFT):
				wheel = "left"
				self.bhit= self.bhit | 0b00010
			else:
				wheel = "right"
				self.bhit= self.bhit | 0b00001
		rospy.loginfo("The %s wheel was %s."%(wheel, state))

	def shutdown(self):
		# stop turtlebot
		rospy.loginfo("Stop TurtleBot")
		# a default Twist has linear.x of 0 & angular.z of 0.  So it'll stop TurtleBot
		self.cmd_vel.publish(Twist())
		# sleep just makes sure TurtleBot receives the stop comm& prior to shutting down the script
		rospy.sleep(1)
 
if __name__ == '__main__':
	try:
		GoForward()
	except:
		rospy.loginfo("GoForward node terminated.")

