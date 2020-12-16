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
import math
import numpy as np
from scipy.ndimage import filters
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float64
from sensor_msgs.msg import CompressedImage
import erl_second_assignment.msg

## Action client
actC = None

## Publishers
velPub = None
headJointPub = None

## Subscriber
imageSub = None

## Goal pose
pos = PoseStamped()

## Counter
sleepCounter = 0

## Flag to notify that the robot has seen the ball
ballFound = False

## Flag to notify that the robot has stopped
robotStopped = False

##
# Use OpenCV to check if the robot camera has detected a ball.
# @param ros_data The compressed image picked up by the camera
def checkForBall(ros_data):
    global ballFound

    np_arr = np.frombuffer(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    greenLower = (50, 50, 20)
    greenUpper = (70, 255, 255)

    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    if len(cnts) > 0:
        # The robot has seen the ball
        ballFound = True

## 
# Use OpenCV to check if the robot camera has detected a ball: if so,
# move the robot until the ball is at the center of the image and has
# a certain radius. When the robot stops, raise a flag.
# @param ros_data The compressed image picked up by the camera
def trackBall(ros_data):
    global ballFound
    global robotStopped
    global velPub

    # If the robot is rotating the camera then simply return
    if robotStopped == True:
        return
    
    np_arr = np.frombuffer(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    greenLower = (50, 50, 20)
    greenUpper = (70, 255, 255)

    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    if len(cnts) > 0:
        # The robot has seen the ball
        ballFound = True

        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if radius > 10:
            vel = Twist()
            vel.angular.z = 0.002*(center[0]-400)
            vel.linear.x = -0.01*(radius-100)

            # Saturation
            if vel.linear.x > 0.6:
                vel.linear.x = 0.6
            elif vel.linear.x < -0.6:
                vel.linear.x = -0.6

            if (abs(vel.linear.x) <= 0.1) and (abs(vel.angular.z) <= 0.1):
                # The robot has stopped
                robotStopped = True
                return

            velPub.publish(vel)

    else:
        # The robot doesn't see the ball
        ballFound = False


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
        global imageSub
        global pos

        # Subscribe to the image topic to check if the robot sees the ball
        imageSub = rospy.Subscriber("robot/camera1/image_raw/compressed", CompressedImage, checkForBall,  queue_size=1)

        while sleepCounter < self.sleepThreshold:
            # Get a random location on the plane
            x = random.randint(-7, 0)
            y = random.randint(0, 7)

            pos.pose.position.x = x
            pos.pose.position.y = y

            # Create the goal
            goal = erl_second_assignment.msg.PlanningGoal(target_pose=pos)

            print("NORMAL state: the robot is moving to position [%d, %d].\n" %(x, y))

            # Send the goal
            actC.send_goal(goal)

            # If the ball is detected while the robot is moving towards 
            # the goal, preempt it and change the state to PLAY
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

            # Increment the counter
            sleepCounter += 1

        # Unsubscribe to the image topic
        imageSub.unregister()

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

        # Create the goal
        goal = erl_second_assignment.msg.PlanningGoal(target_pose=pos)

        # Send the goal
        actC.send_goal(goal)

        # Wait until the robot has reached the destination
        actC.wait_for_result()

        print('SLEEP state: The robot has arrived home.\n')

        # Sleep for a random amount of seconds
        #time.sleep(random.randint(10, 15))
        time.sleep(10)

        # Go back to the NORMAL state
        sleepCounter = 0
        print("SLEEP state: The robot woke up.\n")
        return 'wakeup'

##
# Define Play state
class Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stopplaying'])

    def execute(self, userdata):
        print('State machine: Executing state PLAY.\n')

        global sleepCounter
        global ballFound
        global robotStopped
        global imageSub
        global headJointPub
        global velPub

        # Subscribe to the image topic to track the ball
        imageSub = rospy.Subscriber("robot/camera1/image_raw/compressed", CompressedImage, trackBall)

        while True:
            # Check if the velocity of the robot is 0: if so, move the head
            if robotStopped == True:
                # Keep the robot still
                vel = Twist()
                vel.angular.z = 0
                vel.linear.x = 0
                velPub.publish(vel)

                angle = Float64()
                angle.data = 0.0

                print("PLAY state: rotating the camera to the left.\n")

                # Move the head to the left and stay there for some seconds
                while angle.data < (math.pi/4):
                    angle.data = angle.data + 0.1
                    headJointPub.publish(angle)
                    time.sleep(1)
                time.sleep(2)

                print("PLAY state: rotating the camera to the right.\n")

                # Move the head to the right and stay there for some seconds
                while angle.data > -(math.pi/4):
                    angle.data = angle.data - 0.1
                    headJointPub.publish(angle)
                    time.sleep(1)
                time.sleep(2)

                print("PLAY state: rotating the camera to the center.\n")

                # Move the head to the center
                while angle.data < 0:
                    angle.data = angle.data + 0.1
                    headJointPub.publish(angle)
                    time.sleep(1)

                print("PLAY state: finished rotating the camera.\n")

                robotStopped = False
                sleepCounter += 1

            # If the robot doesn't see the ball for a certain period of time then go back to the NORMAL state
            if ballFound == False:
                startingTime = time.time()

            while ballFound == False:
                if (time.time() - startingTime) >= 10:
                    robotStopped = False

                    # Unsubscribe to the image topic
                    imageSub.unregister()

                    # Go back to the NORMAL state
                    print("PLAY state: The robot hasn't seen the ball for a while, so it stops playing.\n")
                    return 'stopplaying'
                    
        time.sleep(1)
        
        

##
# State machine initialization
def main():
    rospy.init_node('robot_behaviour', anonymous=True)

    global actC
    global headJointPub
    global velPub

    # Create the action client and wait for the server
    actC = actionlib.SimpleActionClient("robot/reaching_goal", erl_second_assignment.msg.PlanningAction)
    actC.wait_for_server()

    # Initialize the two publishers
    headJointPub = rospy.Publisher("robot/joint1_position_controller/command", Float64, queue_size=1)
    velPub = rospy.Publisher("robot/cmd_vel", Twist, queue_size=1)

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