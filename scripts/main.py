#!/usr/bin/env python3
import roslib
import rospy
import smach
import smach_ros
from subprocess import call, Popen
import actionlib
from move_base_msgs.msg import *

# define state Follow
class LineBot(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'],
                             )
        
    def execute(self, userdata):
        rospy.loginfo('LineBot')
        Popen("rosrun competition_pkg app.py", shell=True)
        rospy.sleep(3.0)
        call("rosnode kill /flask_server", shell=True)
        return "outcome"
        
        
class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome"])

    def execute(self, userdata):
        rospy.loginfo("Follow")
        Popen("rosrun competition_pkg follow.py", shell=True)
        rospy.sleep(30.0)
        call("rosnode kill /follow", shell=True)
        return "outcome"

# define state Navigation
class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome"])

    def execute(self, userdata):
        rospy.loginfo("Navigation")
        call("rosrun competition_pkg turtlebot_stop.py", shell=True)
        rospy.sleep(1)
        call("rosnode kill /turtlebot_controller", shell=True)
        return "outcome"


def main():
    rospy.init_node('sm_main')
    rospy.sleep(5.)
    input("PLEASE ENTER TO START>> ")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['EXIT'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('LineBot',    LineBot(),    transitions={'outcome':'Follow'})
        smach.StateMachine.add('Follow',     Follow(),     transitions={'outcome':'Navigation'})
        smach.StateMachine.add('Stop', Stop(), transitions={'outcome':'EXIT'}      )

    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer("sm_server", sm, "/ROOT")
    outcome = sm.execute()
    sis.start()
    outcome = sm.execute()
    sis.stop()



if __name__ == '__main__':
    main()
