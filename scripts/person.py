#!/usr/bin/env python

## @package person
# Mimics the behaviour of a person controlling the robot.
# The person can move the ball to a certain goal position or can make 
# it disappear.

import rospy
import time
import random
import actionlib
from geometry_msgs.msg import PoseStamped
import erl_second_assignment.msg

# Action client
actC = None

# Goal pose
pos = PoseStamped()

##
# Sends a goal to the ball's action server to move it to a random 
# position. After the ball has reached the destination the person
# wait some time before issuing another command.
def moveBall():
    # Get a random location on the plane
    x = random.randint(-7, 0)
    y = random.randint(0, 7)

    pos.pose.position.x = x
    pos.pose.position.y = y
    pos.pose.position.z = 0.25

    # Create the goal
    goal = erl_second_assignment.msg.PlanningGoal(target_pose=pos)

    # Print a feedback message
    print("\nPerson: moving the ball to position [%d, %d].\n" %(x, y))

    # Send the goal
    actC.send_goal(goal)

    # Wait until the ball has reached the destination
    actC.wait_for_result()

    time.sleep(10)

##
# Makes the ball disappear by making it go under the plane.
def disappearBall():
    # Make the ball go under the plane
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = -3

    # Create the goal
    goal = erl_second_assignment.msg.PlanningGoal(target_pose=pos)

    # Print a feedback message
    print("\nPerson: making the ball disappear.\n")

    # Send the goal
    actC.send_goal(goal)

    # Wait until the ball has reached the destination
    actC.wait_for_result()

    time.sleep(20)

##
# Randomly perfoms one of the two available commands.
def person():

    # Send commands to the ball sporadically
    while not rospy.is_shutdown():

        action = random.randint(0, 1)

        if action == 0:
            moveBall()
        else:
            disappearBall()


if __name__ == "__main__":
    try:
        # Initialize the node
        rospy.init_node('person')

        # Create the action client and wait for the server
        actC = actionlib.SimpleActionClient("ball/reaching_goal", erl_second_assignment.msg.PlanningAction)
        actC.wait_for_server()

        person()
        
    except rospy.ROSInterruptException:
        pass