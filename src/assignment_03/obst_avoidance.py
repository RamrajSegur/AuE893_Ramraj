#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import *

class Scan_msg:

    
    def __init__(self):
	'''Initializes an object of this class.
	The constructor creates a publisher, a twist message.
	3 integer variables are created to keep track of where obstacles exist.
	3 dictionaries are to keep track of the movement and log messages.'''
	self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist)
	self.msg = Twist()
	self.sect_1 = 0
	self.sect_2 = 0
	self.sect_3 = 0
	self.ang = {001:-0.2,10:-0.2,11:-0.2,100:0.5,101:0.5,110:0.5,111:0.6}
	self.fwd = {1:0,10:0,11:0,100:1.0,101:0,110:0,111:0}
	self.dbgmsg = {1:'Veer right',10:'Veer right',11:'Veer right',100:'Veer left',101:'Veer left',110:'Veer left',111:'Veer right'}
        self.x=0
        self.y=0
        self.z=0

        self.w=0
        self.ox=0
        self.oy=0
        self.oz=0 


    def reset_sect(self):
	'''Resets the below variables before each new scan message is read'''
	self.sect_1 = 0
	self.sect_2 = 0
	self.sect_3 = 0

    def sort(self, laserscan):
	'''Goes through 'ranges' array in laserscan message and determines 
	where obstacles are located. The class variables sect_1, sect_2, 
	and sect_3 are updated as either '0' (no obstacles within 0.7 m)
	or '1' (obstacles within 0.7 m)
	Parameter laserscan is a laserscan message.'''
	entries = len(laserscan.ranges)
	for entry in range(0,entries):
	    if 0.4 < laserscan.ranges[entry] < 0.75:
		self.sect_1 = 1 if (0 < entry < entries/3) else 0 
		self.sect_2 = 1 if (entries/3 < entry < entries/2) else 0
		self.sect_3 = 1 if (entries/2 < entry < entries) else 0
	#rospy.loginfo("sort complete,sect_1: " + str(self.sect_1) + " sect_2: " + str(self.sect_2) + " sect_3: " + str(self.sect_3))

    def movement(self, sect1, sect2, sect3):
	'''Uses the information known about the obstacles to move robot.
	Parameters are class variables and are used to assign a value to
	variable sect and then	set the appropriate angular and linear 
	velocities, and log messages.
	These are published and the sect variables are reset.'''
	sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3))
	#rospy.loginfo("Sect = " + str(sect)) 
	
       	self.msg.angular.z = self.ang[sect]
	self.msg.linear.x = self.fwd[sect]
	rospy.loginfo(self.dbgmsg[sect])
	self.pub.publish(self.msg)

	self.reset_sect()
 
    def for_callback(self,laserscan):
	'''Passes laserscan onto function sort which gives the sect 
	variables the proper values.  Then the movement function is run 
	with the class sect variables as parameters.
	Parameter laserscan is received from callback function.'''
	self.sort(laserscan)
	#self.movement(self.sect_1, self.sect_2, self.sect_3)
	
    
    def pose_Callback(self,pose_message):

        self.x=pose_message.pose.pose.position.x
        self.y=pose_message.pose.pose.position.y
        self.z=pose_message.pose.pose.position.z

        self.w=pose_message.pose.pose.orientation.w
        self.ox=pose_message.pose.pose.orientation.x
        self.oy=pose_message.pose.pose.orientation.y
        self.oz=pose_message.pose.pose.orientation.z 

    def move_goal(self, x, y, speed, isForward):
	r=rospy.Rate(1)
	rospy.loginfo("I have executed this function")	
	print 'I have executed the move goal'
        #declare a Twist message to send velocity commands
        #VelocityMessage = Twist()
        #Distance between the goal and current pose
        #set the linear velocity to a positive value if isFoward is True
	#print self.x
	#dx=self.x-x
	#dy=self.y-y
	#angle = 2.0 * (acos(self.w))
	#rotation_diff=atan(dy/dx)-angle
	#self.msg.angular.z=rotation_diff
	condition = abs(self.x-x)>1# and self.y!=y	
	print condition       
	while abs(self.x-x)>0.5 and abs(self.y-y)>0.5:
	    #print 'I am in the while loop'
	    if self.sect_1==0 and self.sect_2==0 and self.sect_3==0: 
		dx=x-self.x
		dy=y-self.y
		print 'This is dx  ', dx
		print 'This is dy  ', dy
		d=sqrt((dx)**2+(dy)**2)
		kp=0.04
		print 'this is distance ', d
		#print d
		vel=kp*d
		t3 = 2.0 * (self.w*self.oz+ self.ox *self.oy)
		t4 = 1.0 - 2.0 * (self.oy*self.oy + self.oz * self.oz)
		yaw=2.0*asin(self.oz)
		#print "yaw is: ",yaw
		#print "atan is: ",atan2(dy,dx)
		
		angle=(atan2(dy,dx)-yaw)
		#print "angle diff is: ",angle
		kp1=0.4
		deg=kp1*angle
		#print deg
		#if deg>0.1:
	           #deg=0.1 
		self.msg.angular.z=deg
		#print vel
		if vel>0.5:
		   vel=0.5
                #print "velocity is: ",vel
		self.msg.linear.x = vel
		self.pub.publish(self.msg)
                #starthere
		#self.msg.angular.z = 4*(atan(self.y-y, self.x-x)-self.orientationz);
		#if (angle-atan(dy/dx)>0.1):		
		#	angle = 2.0 * (acos(self.w))
		#	rotation_diff=atan(dy/dx)-angle
		#	self.msg.angular.z=rotation_diff
	    else:
		self.movement(self.sect_1, self.sect_2, self.sect_3)  
		r.sleep()	
    '''if (isForward):
          VelocityMessage.linear.x =abs(speed)
       else: #else set the velocity to negative value to move backward
          VelocityMessage.linear.x =-abs(speed)

       # all velocities of other axes must be zero.
       VelocityMessage.linear.y =0
       VelocityMessage.linear.z =0
       #The angular velocity of all axes must be zero because we want  a straight motion
       VelocityMessage.angular.x = 0
       VelocityMessage.angular.y = 0
       VelocityMessage.angular.z =0

       distance_moved = 0.0
       loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)'''  


def call_back(scanmsg):
   
    sub_obj.for_callback(scanmsg)

def listener(sub_obj):
    '''Initializes node, creates subscriber, and states callback 
    function.'''
    rospy.init_node('navigation_sensors')
    rospy.loginfo("Subscriber Starting")
    sub = rospy.Subscriber('/scan', LaserScan, call_back)
    sub1 = rospy.Subscriber('/odom', Odometry, poseCallback)
    print "I'm about to executre move goal"
    sub_obj.move_goal(-6,-3,1.0,True)
    rospy.spin()

def poseCallback(pose_message):
    sub_obj.pose_Callback(pose_message)
    	

if __name__ == "__main__":
    '''A Scan_msg class object called sub_obj is created and listener
    function is run''' 
    sub_obj = Scan_msg()
    listener(sub_obj)
