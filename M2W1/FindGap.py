import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent
from sensor_msgs.msg import LaserScan
import math
from math import radians

class FindGap():
    def __init__(self):
        # initiliaze
        self.gapPublisher = rospy.Publisher('/gapscan', LaserScan)
        rospy.init_node('FindGap', anonymous=False)

        # What function to call when you ctrl + c   
        rospy.on_shutdown(self.shutdown)
        
        # Laserscan Subscribers
        rospy.Subscriber("/scan", LaserScan, self.LaserScanCallback)
        self.gapArray = []
        self.startIndex = []
        self.endIndex = []

        
        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            pass
        # Is this necessary?
        rospy.spin()
                        
        
    def LaserScanCallback(self, laserscan):
        self.gapArray = []
        self.MakeGapArray(laserscan)
        [self.startIndex, self.endIndex] = self.GapFinder()

        #Publish to the gapscan topic.
        gapMessage = laserscan
        gapMessage.ranges = self.gapArray
        self.gapPublisher.publish(gapMessage)

    def MakeGapArray(self, laserscan):
        entries = len(laserscan.ranges)

        for entry in range(0, entries):
            #Deal with NaNs better
            if math.isnan(laserscan.ranges[entry]):
                self.gapArray.append(0)
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
                if curIndex == (len(self.gapArray) - 1):
                    if curZeroLength > maxZeroLength:
                        maxZeroLength = curZeroLength
                        endIndex = curIndex
                        curZeroLegnth = 0
            if self.gapArray[curIndex] is not 0:
                if curZeroLength > maxZeroLength:
                    maxZeroLength = curZeroLength
                    endIndex = prevIndex
                    curZeroLegnth = 0
        return [(endIndex - maxZeroLength + 1), endIndex]




    def movement(self):
        '''Uses the information known about the obstacles to move robot.

        Parameters are class variables and are used to assign a value to
        variable sect and then  set the appropriate angular and linear 
        velocities, and log messages.
        These are published and the sect variables are reset.'''
        averages = [self.average3,self.average4,self.average2,self.average5,self.average1]
        #averages = [self.average5,self.average4,self.average3,self.average2,self.average1]
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
        self.reset_averages()

    def reset_averages(self):
        '''Resets the below variables before each new scan message is read'''
        self.average1 = 0
        self.average2 = 0
        self.average3 = 0
        self.average4 = 0
        self.average5 = 0



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
