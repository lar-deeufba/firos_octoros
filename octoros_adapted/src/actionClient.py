#! /usr/bin/env python
import rospy
import actionlib
from octo_ros.msg import PrintPartGoal, PrintPartAction

if __name__ == '__main__':
    rospy.init_node('print_part_client')
    client = actionlib.SimpleActionClient('printer_3D_server', PrintPartAction)
    client.wait_for_server()

    goal = PrintPartGoal
    goal.file_to_print = "testfil"
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(3.0))