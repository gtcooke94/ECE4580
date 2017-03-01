#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent
from math import radians
import time

class Scan_msg:

    
    def __init__(self):
        '''Initializes an object of this class.

        The constructor creates a publisher, a twist message.
        3 integer variables are created to keep track of where obstacles exist.
        3 dictionaries are to keep track of the movement and log messages.'''
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist)
        self.msg = Twist()
        self.average1 = 0
        self.average2 = 0
        self.average3 = 0
        self.average4 = 0
        self.average5 = 0

        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0
        self.sect_4 = 0
        self.sect_5 = 0
        self.ang = {1:.6, 2:.6, 3:0, 4:-.6, 5:-.6}
        self.fwd = {1:.05, 2:.2, 3:.3, 4:.2, 5:.05}
        self.dbgmsg = {1:'Turn Left', 2:'Veer Left', 3:'Move Straight', 4:'Veer Right', 5:'Turn Right'}

                
    def reset_averages(self):
        '''Resets the below variables before each new scan message is read'''
        self.average1 = 0
        self.average2 = 0
        self.average3 = 0
        self.average4 = 0
        self.average5 = 0

    def sort(self, laserscan):
        '''Goes through 'ranges' array in laserscan message and determines 
        where obstacles are located. The class variables sect_1, sect_2, 
        and sect_3 are updated as either '0' (no obstacles within 0.7 m)
        or '1' (obstacles within 0.7 m)

        Parameter laserscan is a laserscan message.'''
        entries = len(laserscan.ranges)
        maxValue = laserscan.range_max
        minValue = laserscan.range_min
        rospy.loginfo(str(maxValue) + "    " + str(minValue))

        totalEntries1 = 0
        totalEntries2 = 0
        totalEntries3 = 0
        totalEntries4 = 0
        totalEntries5 = 0
        toSubtract1 = 0
        toSubtract2 = 0
        toSubtract3 = 0
        toSubtract4 = 0
        toSubtract5 = 0
        for entry in range(0, entries/5):
            if not (math.isnan(laserscan.ranges[entry])):
                totalEntries1 += laserscan.ranges[entry]
            else:
                toSubtract1 += 1 
        self.average1 = totalEntries1/(entries/5 - toSubtract1 + 1)

        for entry in range(entries/5, 2*entries/5):
            if not (math.isnan(laserscan.ranges[entry])):
                totalEntries2 += laserscan.ranges[entry]
            else:
                toSubtract2 += 1     
        self.average2 = totalEntries2/(entries/5 - toSubtract2 + 1)

        for entry in range(2*entries/5, 3*entries/5):
            if not (math.isnan(laserscan.ranges[entry])):
                totalEntries3 += laserscan.ranges[entry]
            else:
                toSubtract3 += 1  
        self.average3 = totalEntries3/(entries/5 - toSubtract3 + 1)

        for entry in range(3*entries/5, 4*entries/5):
            if not (math.isnan(laserscan.ranges[entry])):
                totalEntries4 += laserscan.ranges[entry]
            else:
                toSubtract4 += 1 
        self.average4 = totalEntries4/(entries/5 - toSubtract4 + 1)

        for entry in range(4*entries/5, entries):
            if not (math.isnan(laserscan.ranges[entry])):
                totalEntries5 += laserscan.ranges[entry]
            else:
                toSubtract5 += 1 
        self.average5 = totalEntries5/(entries/5 - toSubtract5 + 1)

        rospy.loginfo('End of finding averages')

    def movement(self):
        '''Uses the information known about the obstacles to move robot.

        Parameters are class variables and are used to assign a value to
        variable sect and then  set the appropriate angular and linear 
        velocities, and log messages.
        These are published and the sect variables are reset.'''
        averages = [self.average1,self.average2,self.average3,self.average4,self.average5]
        #for average in averages:
        #    rospy.loginfo("1: " + str(average))
        rospy.loginfo(averages)
        maxSector = averages.index(max(averages)) + 1
        self.msg.angular.z = self.ang[maxSector]
        self.msg.linear.x = self.fwd[maxSector]
        rospy.loginfo(self.dbgmsg[maxSector])
        self.pub.publish(self.msg)

        self.reset_averages()
 
    def for_callback(self,laserscan):
        '''Passes laserscan onto function sort which gives the sect 
        variables the proper values.  Then the movement function is run 
        with the class sect variables as parameters.

        Parameter laserscan is received from callback function.'''
        self.sort(laserscan)
        self.movement()
    

def call_back(scanmsg):
    '''Passes laser scan message to for_callback function of sub_obj.

    Parameter scanmsg is laserscan message.'''
    sub_obj.for_callback(scanmsg)

def listener():
    '''Initializes node, creates subscriber, and states callback 
    function.'''
    rospy.init_node('navigation_sensors')
    rospy.loginfo("Subscriber Starting")
    pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist)

    #rospy.Subscriber("/mobile_base/events/wheel_drop", WheelDropEvent, self.WheelDropEventCallback)
    r = rospy.Rate(5)

    rightTurn = Twist()
    rightTurn.angular.z = -radians(60)
    leftTurn = Twist()
    leftTurn.angular.z = radians(60)
    centerBumper = Twist()
    centerBumper.linear.x = -.2
    sub = rospy.Subscriber('/scan', LaserScan, call_back)
    subBumper = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.BumperEventCallback)
    rospy.spin()

def BumperEventCallback(self, data):
    rospy.loginfo("In Bumper Callback")
    if (data.state == BumperEvent.PRESSED):
        if (data.bumper == BumperEvent.LEFT):
            for i in range(0, 10):
                pub.publish(self.rightTurn)
                self.r.sleep()
            #rospy.sleep(1.)
            #time.sleep(1)
        elif (data.bumper == BumperEvent.RIGHT):
            for i in range(0, 10):
                self.pub.publish(self.leftTurn)
                self.r.sleep()
            #rospy.sleep(1.)
            #time.sleep(1)
        else:
            for i in range(0, 10):
                self.pub.publish(self.centerBumper)
                self.r.sleep()
            #rospy.sleep(1.)
            #time.sleep(1)

if __name__ == "__main__":
    '''A Scan_msg class object called sub_obj is created and listener
    function is run''' 
    sub_obj = Scan_msg()
    listener()
