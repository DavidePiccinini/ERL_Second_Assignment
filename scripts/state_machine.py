#!/usr/bin/env python

## @package state_machine
# Defines the different robot behaviours and the transitions between them.
# Available states are NORMAL, SLEEP and PLAY.

import roslib
import rospy
import smach
import smach_ros
import time
import random
import actionlib
import cv2
import imutils
import numpy as np
from scipy.ndimage import filters
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import erl_second_assignment.msg

## Action client
actC = None

## Goal pose
pos = PoseStamped()

## Counter
sleepCounter = 0

## Flag to notify that the robot has seen the ball
ballFound = False

##
# 
def checkForBall(ros_data):
    global ballFound

    # Direct conversion to CV2
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

    greenLower = (50, 50, 20)
    greenUpper = (70, 255, 255)

    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # Only proceed if at least one contour was found
    if len(cnts) > 0:
        # Set the flag to True
        ballFound = True

## 
# 
def trackBall(ros_data):
    global ballFound
    ballFound = False
    #

##
# Define Normal state
class Normal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sleep','play'])

        # Threshold
        # self.sleepThreshold = random.randint(5, 10)
        self.sleepThreshold = 3

    def execute(self, userdata):
        print('State machine: Executing state NORMAL.\n')

        global sleepCounter
        global ballFound
        global actC
        global pos

        imageSub = rospy.Subscriber("robot/camera1/image_raw/compressed", CompressedImage, checkForBall,  queue_size=1)

        while sleepCounter < self.sleepThreshold:
            # Get a random location on the plane
            x = random.randint(-7, 7)
            y = random.randint(-7, 7)

            pos.pose.position.x = x
            pos.pose.position.y = y
            pos.pose.orientation.w = 0

            # Create the goal
            goal = erl_second_assignment.msg.PlanningGoal(target_pose=pos)

            # Send the goal
            actC.send_goal(goal)

            while actC.get_state() != GoalStatus.SUCCEEDED:
                if ballFound == True:
                    # Set the flag back to False
                    ballFound = False

                    # Preempt the current goal
                    actC.cancel_goal()

                    # Unsubscribe to the image topic
                    imageSub.unregister()

                    # Go into the PLAY state
                    print('NORMAL state: The robot saw the ball and wants to play.\n')
                    return 'play'

            #     #time.sleep(2)

            # Increment the counter
            sleepCounter += 1

        # Go into the SLEEP state
        print('NORMAL state: The robot is sleepy.\n')
        return 'sleep'

##
# Define Sleep state
class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wakeup'])

    def execute(self, userdata):
        print('State machine: Executing state SLEEP.\n')

        global sleepCounter
        global actC
        global pos

        # Get the home location on the plane
        x = -5
        y = 7

        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.orientation.w = 0

        # Create the goal
        goal = erl_second_assignment.msg.PlanningGoal(target_pose=pos)

        # Send the goal
        actC.send_goal(goal)

        # Wait until the robot has reached the destination
        actC.wait_for_result()

        print('SLEEP state: The robot has arrived home.\n')

        # Sleep for a random amount of seconds
        #time.sleep(random.randint(10, 15))
        time.sleep(5)

        # Go back to the NORMAL state
        sleepCounter = 0
        print('SLEEP state: The robot woke up.\n')
        return 'wakeup'

##
# Define Play state
class Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stopplaying'])

        # Threshold
        # self.timeThreshold = random.randint(30, 40)
        self.timeThreshold = 5

    def execute(self, userdata):
        print('State machine: Executing state PLAY.\n')

        global sleepCounter
        global ballFound
        # global actC

        imageSub = rospy.Subscriber("robot/camera1/image_raw/compressed", CompressedImage, trackBall,  queue_size=1)

        timePassed = 0

        # Keep incrementing the time counter if the robot doesn't see the ball
        while timePassed < self.timeThreshold:
            # Check if the velocity of the robot is 0: if so, move the head

            if ballFound == False:
                time.sleep(1)
                timePassed += 1
        
        # Go back to the NORMAL state
        imageSub.unregister()
        print("PLAY state: The robot hasn't seen the ball for a while, so it stops playing.\n")
        return 'stopplaying'

##
# State machine initialization
def main():
    rospy.init_node('robot_behaviour', anonymous=True)

    global actC

    # Create the action client and wait for the server
    actC = actionlib.SimpleActionClient("robot/reaching_goal", erl_second_assignment.msg.PlanningAction)
    actC.wait_for_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(),
                                transitions={'sleep': 'SLEEP',
                                             'play': 'PLAY'})

        smach.StateMachine.add('SLEEP', Sleep(),
                                transitions={'wakeup': 'NORMAL'})

        smach.StateMachine.add('PLAY', Play(),
                                transitions={'stopplaying': 'NORMAL'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()