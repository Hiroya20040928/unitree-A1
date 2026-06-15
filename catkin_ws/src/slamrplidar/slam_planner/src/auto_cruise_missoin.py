#!/usr/bin/env python  
# coding: utf-8
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt  
import collections as cll
  
class MultiNav():  
    def __init__(self):  
        rospy.init_node('MultiNav', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  
  
        # How long in seconds should the robot pause at each location?  
        self.rest_time = rospy.get_param("~rest_time", 10)  
  
        # Are we running in the fake simulator?  
        self.fake_test = rospy.get_param("~fake_test", False)  
  
        # Goal state return values  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED','SUCCEEDED',  
                       'ABORTED', 'REJECTED','PREEMPTING', 'RECALLING',   
                       'RECALLED','LOST']  
  
        # Set up the goal locations. Poses are defined in the map frame.  
        # An easy way to find the pose coordinates is to point-and-click  
        # Nav Goals in RViz when running in the simulator.  
        # Pose coordinates are then displayed in the terminal  
        # that was used to launch RViz.  
        locations = dict()  
    
        # we give the several points at here
        locations['home_0'] = Pose(Point(2.19139647484, -2.31213498116, 0.00), Quaternion(0.000, 0.000, -0.35013148553, 0.936700561994))  
        locations['home_1'] = Pose(Point(4.8243560791, -2.13051557541, 0.00), Quaternion(0.000, 0.000, 0.596221592164, 0.802819913205))  
        locations['home_2'] = Pose(Point(2.97264814377, 2.50924801826, 0.00), Quaternion(0.000, 0.000, 0.981138247899, 0.193307367964))  
        locations['home_3'] = Pose(Point(0.412056326866, 0.00907477736473, 0.00), Quaternion(0.000, 0.000, 0.985232193867, -0.171223608674)) 
        locations['home_4_0'] = Pose(Point(2.19803142548, -2.60839629173, 0.00), Quaternion(0.000, 0.000, -0.484941997516, 0.874546316123))
        

        dd = ['home_0', 'home_1', 'home_2', 'home_3', 'home_4_0']

        # d = cll.OrderedDict([('home_0', 1), ('home_1', 2), ('home_2', 3), ('home_3', 4), ('home_4_0', 5)])

        # locations[0] = Pose(Point(1.37165856361, -0.920648574829, 0.00), Quaternion(0.000, 0.000, -0.597629161309, 0.801772652036))  
        # locations[1] = Pose(Point(3.60029625893, 0.0308146104217, 0.00), Quaternion(0.000, 0.000, 0.635143321447, 0.77239430424))  
        # locations[2] = Pose(Point(1.68819582462, 4.14018297195, 0.00), Quaternion(0.000, 0.000, 0.983479112078, 0.181021645406))  
        # locations[3] = Pose(Point(-0.0941917300224, 2.0376625061, 0.00), Quaternion(0.000, 0.000, 0.97619304141, -0.216903540549)) 
        # locations[4] = Pose(Point(1.34527516365, -0.737474322319, 0.00), Quaternion(0.000, 0.000, -0.274246139658, 0.961659531686))
        #locations['home_balcony'] = Pose(Point(2.86, -0.574, 0.00), Quaternion(0.000, 0.000, 0.739, 0.674)) 

        
        # Publisher to manually control the robot (e.g. to stop it)  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)  
  
        # Subscribe to the move_base action server  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
        rospy.loginfo("Waiting for move_base action server...")  
  
        # Wait 60 seconds for the action server to become available  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  
          
        # A variable to hold the initial pose of the robot to be set by the user in RViz  
        initial_pose = PoseWithCovarianceStamped()  
        # Variables to keep track of success rate, running time, and distance traveled  
        n_locations = len(locations)  
        n_goals = 0  
        n_successes = 0  
        i = n_locations  
        distance_traveled = 0  
        start_time = rospy.Time.now()  
        running_time = 0  
        location = ""  
        last_location = ""  
        # Get the initial pose from the user  
        rospy.loginfo("Click on the map in RViz to set the intial pose...")  
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)  
        self.last_location = Pose()  
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)  
        # Make sure we have the initial pose  
        while initial_pose.header.stamp == "":  
            rospy.sleep(1)  
        rospy.loginfo("Starting navigation test")  
  
        # Begin the main loop and run through a sequence of locations  
        while not rospy.is_shutdown():  
  
        # If we've gone through the current sequence, start with a new random sequence  
            for j in range(5):
                
                # sequence[v] = v
                # i = 0  
                # sequence = sample(locations, n_locations)  
                # #sequence = locations[i] 
                # # Skip over first location if it is the same as the last location  
                # if sequence[0] == last_location:  
                #     i = 1  
  
            # Get the next location in the current sequence  
                # location = v  
    
                # Keep track of the distance traveled.  
                # Use updated initial pose if available.  
                # if initial_pose.header.stamp == "":  
                #     distance = sqrt(pow(locations[dd[j]].position.x  
                #             - locations[last_location].position.x, 2) +  
                #             pow(locations[location].position.y -  
                #             locations[last_location].position.y, 2))  
                # else:  
                #     rospy.loginfo("Updating current pose.")  
                #     distance = sqrt(pow(locations[location].position.x  
                #             - initial_pose.pose.pose.position.x, 2) +  
                #             pow(locations[location].position.y -  
                #             initial_pose.pose.pose.position.y, 2))  
                #     initial_pose.header.stamp = ""  
    
                # Store the last location for distance calculations  
                last_location = location  
    
                # Increment the counters  
                i += 1  
                n_goals += 1  
    
                # Set up the next goal location  
                self.goal = MoveBaseGoal()  
                self.goal.target_pose.pose = locations[dd[j]]  
                self.goal.target_pose.header.frame_id = 'map'  
                self.goal.target_pose.header.stamp = rospy.Time.now()  
    
                # Let the user know where the robot is going next  
                rospy.loginfo("Going to: " + str(dd[j]))  
                # Start the robot toward the next location  
                self.move_base.send_goal(self.goal)  
    
                # Allow 5 minutes to get there  
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))  
    
                # Check for success or failure  
                if not finished_within_time:  
                    self.move_base.cancel_goal()  
                    rospy.loginfo("Timed out achieving goal")  
                else:  
                    state = self.move_base.get_state()  
                    if state == GoalStatus.SUCCEEDED:  
                        rospy.loginfo("Goal succeeded!")  
                        n_successes += 1  
                        #distance_traveled += distance  
                    else:  
                        rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  
    
                # How long have we been running?  
                running_time = rospy.Time.now() - start_time  
                running_time = running_time.secs / 60.0  
    
                # Print a summary success/failure, distance traveled and time elapsed  
                #rospy.loginfo("Success so far: " + str(n_successes) + "/" +  
                #            str(n_goals) + " = " + str(100 * n_successes/n_goals) + "%")  
                #rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +  
                          #  " min Distance: " + str(trunc(distance_traveled, 1)) + " m")  
                #rospy.sleep(self.rest_time)  
  
    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose  
  
    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  
def trunc(f, n):  
  
    # Truncates/pads a float f to n decimal places without rounding  
    slen = len('%.*f' % (n, f))  
    return float(str(f)[:slen])  
  
if __name__ == '__main__':  
    try:  
        MultiNav()  
        rospy.spin()  
    except rospy.ROSInterruptException:  
        rospy.loginfo("AMCL navigation test finished.")  
