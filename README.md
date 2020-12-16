# Experimental Robotics Laboratory Second Assignment
The second assignment builds on top of the first one: we students are now asked to implement a similar robot behaviour on a gazebo-simulated pet robot. The robot is akin to a dog, it's a wheeled robot with a neck and a camera-equipped head that can rotate: it can move randomly, track a ball, move its head and go to sleep.

---

## System Architecture

### Component Diagram

<p align="center"> 
<img src="">
</p>

The software architecture is based on **four main components**:

- **Person module**

    This module mimics the behaviour of a person that can either move the ball to a random valid location or make it disappear under the plane.

    It implements an action client which randomly sends one of the two types of goals to the *"go_to_point_ball.py"* action server: the module then waits for the ball to reach its destination and sleeps for some time before sending another goal.

- **Finite state machine**

    This component implements a finite state machine using "Smach". It features an action client, two publishers and one subscriber.

    The three states, together with the transitions between them, will be further explained in the following paragraph.

- **Robot action server**

    This component implements an action server to control the robot. Since it was provided by the professor and I only modified some gains and some print functions, I'm not going to further explain how it works in detail.

- **Ball action server**

    This component implements an action server to control the ball. As for the robot action server, I'm not going over it in detail. 

### State Diagram

<p align="center"> 
<img src="">
</p>

This is the state diagram that shows how the finite state machine works:

When the robot is in **Normal** state it moves randomly in the plane by sending a goal to the robot action server: if at anytime the robot sees the ball, then it switches to the *Play* state. If instead the robot has performed enough actions (an action is either reaching a location or performing the sequence of camera movements as described in the *Play* state) then it switches to the *Sleep* state.

When the robot is in **Sleep** state, it first reaches the predefined "Home" location (-5, 7), then stays there for some time and finally wakes up, transitioning back to the *Normal* state.

When the robot is in **Play** state, it tracks the ball until it gets close enough: when this happens, the robot stays still and rotates its head 45 degrees on the left, then does the same for the right and finally goes back to the center. This behaviour goes on until the ball can't be found for a certain period of time, after which the robot switches back to the *Normal* state.

### rqt_graph

<p align="center"> 
<img src="">
</p>

---

## ROS Messages and Parameters

Custom ROS **messages** are:

- **Planning.action**

    ```
    geometry_msgs/PoseStamped target_pose
    ---
    ---
    string stat
    geometry_msgs/Pose position 
    ```

    Describes the action that will be used both for the robot action server and the ball one: `target_pose` represents the "goal", `stat` and `position` are the "feedback": there's no "result" field.

I haven't defined any custom **ROS parameter**.

---

## Packages and File List

Going in alphabetical order:

- **action**

    - `Planning.action`
    
        Action file described above.

- **diagrams**

    - `Component_Diagram.png`, `State_Diagram.png` and `rosgraph.png`

        The three diagrams shown in this README file.

- **documentation**

    - **html** and **latex**

        Contain the output of *Doxygen*.

    - `assignment.conf`

        Configuration file used to run *Doxygen*.

- **launch**

    - `gazebo_world.launch`

        The launchfile which runs the gazebo server and client, the action servers, the finite state machine and spawns the ball, robot and human models.

- **scripts**

    - `go_to_point_ball.py`, `go_to_point_robot.py`, `person.py` and `state_machine.py`

        The modules of the architecture, as described in the *System Architecture* section.

- **urdf**

    - `ball.gazebo`, `ball.xacro`, `human.urdf`, `robot.gazebo` and `robot.xacro`

        The files which describe the models used for the ball, robot and human.

- **worlds**

    - `world_assignment.world`

    The world file in which the simulation environment is defined.

- `CMakeLists.txt` and `package.xml`

    Necessary files to compile and run the system nodes.

---

## Installation and Running Procedure

First of all, clone this repository inside your ROS workspace's *"/src"* folder .

Then, navigate to the *"/scripts"* folder and make the Python scripts executable with:
```
$ chmod +x go_to_point_ball.py
$ chmod +x go_to_point_robot.py
$ chmod +x person.py
$ chmod +x state_machine.py
```

Go back to the root folder of your ROS workspace and execute:
```
$ catkin_make
$ catkin_make install
```

In a separate terminal run:
```
$ roscore
```

Finally, run the launchfile with this command:
```
$ roslaunch erl_second_assignment gazebo_world.launch
```
After a brief setup period in which gazebo is launched and the models are spawned, on the console you will see the transitions between states, when the robot and the ball reach their destinations, and other feedbacks from the system.

---

## Working Hypothesis and Environment

The **working hypotheses** are: 

<!-- - The robot has no modules dedicated to sensing, which is considered implicit, thus the commands sent by the user are understood perfectly.

- The person can, at any time, aknowledge exactly what the robot state is and thus sends commands accordingly; e.g. if the robot is in normal state he/she won't point a location.

- The person only sends "play" voice commands or pointing gestures.

- The robot knows the environment, i.e. the map's boundaries, the key locations position and the person position.

- The robot can't switch states when it's moving, reaching a location is considered an atomic action.

- The robot can go anywhere on the map.

- The robot has infinite battery life, it never has to recharge. -->

The **enviroment hypotheses** are:

<!-- - The map is described by two values, `xmax` and `ymax`: it's assumed to be a rectangle starting from the origin O = (0, 0) and having as sides 2D vectors **OX** and **OY**, with X = (xmax, 0) and Y = (0, ymax).

- The environment is free from obstacles.

- The only key location the robot knows is "Home".

- The person position is constant throughout the entire robot operation. -->

---

## System's Features

<!-- The system satisfies all the requirements: the robot can transition corretly between states and executes the expected behaviour. Print functions placed in the components allow the user to monitor the robot and what is the person doing. The robot control also checks if the requested location is consistent with respect to the map's boundaries: if so, the robot moves, otherwise it doesnt.
Having no sensing makes the system fast and reactive to commands.
The robot control service can accomodate requests from one or more clients. -->

---

## System's Limitations

<!-- The system is dependent on a person which sends fixed commands according to the robot state, but in principle we would like to have a flexible system that can receive a wide range of commands and then reasons about what to do.
Implementing the robot control as a service has its pros, but a future development may want to make the robot able to transition between states while it's reaching a destination: doing so would require modifying both the finite state machine and the robot control component.
The code is almost entirely *ad hoc* for the use case considered: the components aren't modular enough to be re-usable for other applications, they would need almost certainly additional work to fit in. -->

---

## Possible Technical Improvements

<!-- A possible improvement would be removing the person component and instead develop two sensing components, one for voice commands and one for pointing gestures, which process respective user inputs and sends them to a reasoning component. The reasoning component would first check if the command is valid, then, depending on the robot's state, send data to the finite state machine. This would improve modularity and re-usability.
Another improvement would be supporting other types of commands, for example the user could say the name of a key location, e.g. "Home", and the robot would go there. This is related to the reasoning component just mentioned and would require the user to add key locations via the ROS parameter server, which is already possible. -->

---

## Authors and Contacts

Davide Piccinini matricola S4404040
Emails: 
- piccio98dp@gmail.com
- 4404040@studenti.unige.it

---

## Doxygen Documentation

<!-- [Doxygen pdf documentation](https://github.com/DavidePiccinini/ERL_First_Assignment/tree/master/documentation/latex/refman.pdf) -->
