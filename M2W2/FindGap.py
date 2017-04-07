import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from math import radians

# Right now we find the largest gap
# Need: Calculate the gap center angle AND the final heading angle.
# then it says to implement the algorithm so I think this means we need to put it into our movement state machine

class FindGap():
    def __init__(self):
        # initiliaze
        self.gapPublisher = rospy.Publisher('/gapscan', LaserScan)
        rospy.init_node('FindGap', anonymous=False)

        # What function to call when you ctrl + c   
        rospy.on_shutdown(self.shutdown)
        
        # Laserscan Subscribers
        rospy.Subscriber("/scan", LaserScan, self.LaserScanCallback)
        rospy.Subscriber("/odom", Odometry, self.OdometryCallback)
        self.gapArray = []
        self.gapToPublish = []
        self.startIndex = []
        self.endIndex = []

        
        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            pass
        # Is this necessary?
        rospy.spin()
                        
    def OdometryCallback(self, odom):
    	#rospy.loginfo(str(odom.pose.pose))
    	self.worldGoalAngle = math.atan((odom.pose.pose.position.y)/(20 - odom.pose.pose.position.x))
    	self.turtlebotGoalAngle = self.worldGoalAngle + odom.pose.pose.orientation.z
    	self.turtlebotangle = self.turtlebotGoalAngle + self.gapTheta


    def LaserScanCallback(self, laserscan):
        self.gapArray = []
        self.gapToPublish = []
        self.deltaTheta = 0
        self.MakeGapArray(laserscan)
        [self.startIndex, self.endIndex, self.gapTheta] = self.GapFinder()

        #Publish to the gapscan topic.
        self.MakeGapToPublish()
        gapMessage = laserscan
        gapMessage.ranges = self.gapToPublish
        self.gapPublisher.publish(gapMessage)
        rospy.loginfo('Odom Angle: ' + str(self.turtlebotangle) + '   Gap Angle: ' + str(self.gapTheta) + '   Goal Angle: ' + str(self.turtlebotangle)

    def MakeGapToPublish(self):
    	entries = len(self.gapArray)
    	for entry in range(0, entries):
    		if self.gapArray[entry] == 0:
    			self.gapToPublish.append(1)
    		else:
    			self.gapToPublish.append(0)

    def MakeGapArray(self, laserscan):
        entries = len(laserscan.ranges)
        self.deltaTheta = laserscan.angle_increment
        for entry in range(0, entries):
            #Deal with NaNs better
            if math.isnan(laserscan.ranges[entry]):
                if (0 < entry) & (entry < entries-1): # all middle values
                    if (math.isnan(laserscan.ranges[entry+1])):
                        self.gapArray.append(0)
                    elif (math.isnan(laserscan.ranges[entry-1])):
                        self.gapArray.append(0)
                    else:
                        value = (laserscan.ranges[entry-1] + laserscan.ranges[entry+1])/2
                        self.gapArray.append(value if value < 3 else 0)  
                elif (entry == 0): #first value
                    if (math.isnan(laserscan.ranges[entry+1])):
                        self.gapArray.append(0)
                    else:
                        self.gapArray.append(laserscan.ranges[entry+1])
                else: #last value
                    if (math.isnan(laserscan.ranges[entry-1])):
                        self.gapArray.append(0)
                    else:
                        self.gapArray.append(laserscan.ranges[entry-1])

            elif (laserscan.ranges[entry] > 3):
                self.gapArray.append(0)
            else:
                self.gapArray.append(laserscan.ranges[entry])

    def GapFinder(self):
        startIndex = 0
        endIndex = 0
        prevIndex = 0
        maxZeroLength = 0
        curZeroLength = 0
        for curIndex in range(0, len(self.gapArray)):
            if self.gapArray[curIndex] == 0:
                curZeroLength += 1
                prevIndex = curIndex
                if curIndex == (len(self.gapArray) - 1):
                    if (curZeroLength > maxZeroLength):
                        maxZeroLength = curZeroLength
                        endIndex = curIndex
                        curZeroLegnth = 0
            if self.gapArray[curIndex] is not 0:
                if (curZeroLength > maxZeroLength):
                    maxZeroLength = curZeroLength
                    endIndex = prevIndex
                    curZeroLength = 0
        # Right here we can have this return the angle of the middle of the gap
        # middle of the gap is at index
        midGapIndex = round((((endIndex - (maxZeroLength)) + (endIndex)) / 2))
        zeroDegInd = round(len(self.gapArray)/2) - 1
        self.gapTheta = (midGapIndex - zeroDegInd) * self.deltaTheta
        #rospy.loginfo(str(self.gapTheta) + ' ' + str(midGapIndex) + ' ' + str(zeroDegInd) +' ' + str(self.deltaTheta))
        return [(endIndex - maxZeroLength + 1), endIndex, self.gapTheta]




    def movement(self):
        '''Uses the information known about the obstacles to move robot.

        Parameters are class variables and are used to assign a value to
        variable sect and then  set the appropriate angular and linear 
        velocities, and log messages.
        These are published and the sect variables are reset.'''
        averages = [self.average3,self.average4,self.average2,self.average5,self.average1]
        rospy.loginfo(averages)
        #for average in averages:
        #    rospy.loginfo("1: " + str(average))
        maxSector = averages.index(max(averages)) + 1
        # I think this logic is wrong cause it defaults left.
        # One way to combat this would make "1" go straight in our archetecture, and then have 2 and 3
        # be the veer states, and then 4 and 5 be the turn states
        self.mMsg.angular.z = self.ang[maxSector]
        self.mMsg.linear.x = self.fwd[maxSector]
        rospy.loginfo(self.dbgmsg[maxSector])



    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 & angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop comm& prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    #try:
    FindGap()
    #except:
     #   rospy.loginfo("GoForward node terminated.")
