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

## Flag for notifying 
# playState = False

## Flag to notify that the robot has seen the ball
ballFound = False

##
# Calls the "robot_control" service
# @param x The x position of the destination
# @param y The y position of the destination
# def robotControlCall(x, y):

#     rospy.wait_for_service('robot_control')
#     try:

#         robot_control = rospy.ServiceProxy('robot_control', MoveRobot)
#         response = robot_control(x, y)
#         return response.goalReached

#     except rospy.ServiceException as e:

#         print("Service call failed %s.\n"%e)

## 
# Callback for the 'voice_command' topic
# @param data The voice command
# def receivedVoiceCommand(data):

#     global playState

#     # Change the robot state flag
#     playState = True

## 
# Callback for the 'pointing_gesture' topic
# @param data The pointed location
# def receivedPointingGesture(data):

#     global sleepCounter
    
#     # Go to the pointed location
#     goalReached = robotControlCall(data.x, data.y)
#     if goalReached == True:

#         # Return to the person's position
#         robotControlCall(personx, persony)
#         print('PLAY state: The robot reached the person.\n')

#         # Increment the counter
#         sleepCounter+=1

##
# Define Normal state
class Normal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sleep','play'])

        # Threshold
        self.sleepThreshold = random.randint(5, 10)

    def execute(self, userdata):
        
        print('State machine: Executing state NORMAL.\n')

        global sleepCounter
        global ballFound

        rospy.Subscriber("robot/output/image_raw/compressed", CompressedImage, queue_size=1)

        while sleepCounter < self.sleepThreshold:

            # Get a random location on the plane
            x = random.randint(-7, 7)
            y = random.randint(-7, 7)

            pos.pose.position.x = x
            pos.pose.position.y = y
            pos.pose.position.z = 0.05

            # Create the goal
            goal = erl_second_assignment.msg.PlanningGoal(target_pose=pos)

            # Send the goal
            actC.send_goal(goal)

            while actC.getState() != GoalStatus.SUCCEEDED:
                if ballFound:
                    actC.cancel_goal()

        # time.sleep(5 / timeScale)

        # # Move randomly until the robot is sleepy
        # while sleepCounter < self.sleepThreshold:

        #     if playState == True:
        #         # Go into the PLAY state
        #         print('NORMAL state: The robot is eager to play.\n')
        #         return 'play'

        #     x = random.randint(0, mapx)
        #     y = random.randint(0, mapy)

        #     goalReached = robotControlCall(x, y)
        #     if goalReached == True:

        #         sleepCounter+=1
        
        # # Go into the SLEEP state
        # print('NORMAL state: The robot is sleepy.\n')
        # return 'sleep'

##
# Define Sleep state
class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wakeup'])

    def execute(self, userdata):
        
        # rospy.set_param("robot/state", "sleep")
        # print('State machine: Executing state SLEEP.\n')

        # global sleepCounter

        # time.sleep(5 / timeScale)

        # # Call robot control service to go "home"
        # robotControlCall(homex, homey)
        # print('SLEEP state: The robot has arrived home.\n')

        # # Sleep for a random amount of seconds
        # time.sleep(random.randint(10, 15) / timeScale)

        # # Go back to the NORMAL state
        # sleepCounter = 0
        # print('SLEEP state: The robot woke up.\n')
        # return 'wakeup'

##
# Define Play state
class Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stopplaying'])
        
        # rospy.Subscriber('pointing_gesture', Location, receivedPointingGesture)

        # Threshold
        # self.timeThreshold = random.randint(30, 100)

    def execute(self, userdata):

        # rospy.set_param("robot/state", "play")
        # print('State machine: Executing state PLAY\n')

        # global sleepCounter

        # timePassed = 0

        # time.sleep(5 / timeScale)

        # # Call robot control service to go to the person's position
        # robotControlCall(personx, persony)
        # print('PLAY state: The robot reached the person.\n')
        # sleepCounter+=1

        # # Keep incrementing the time counter if the person doesn't point locations
        # while timePassed < self.timeThreshold:

        #     timePassed+=1
        #     time.sleep(1 / timeScale)
        
        # # Go back to the NORMAL state
        # print("PLAY state: The robot doesn't want to play anymore.\n")
        # return 'stopplaying'

##
# State machine initialization
def main():
    rospy.init_node('robot_behaviour', anonymous=True)

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