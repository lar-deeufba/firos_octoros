#!/usr/bin/env python

""" octoROS - integrating octoprint with ROS, so it's possible to control your 3D printer from your robot
This library is based in the Octoprint API, more info can be found here:
http://docs.octoprint.org/en/master/api/index.html
"""

import os
import sys
import datetime
import rospy
import actionlib
import requests
import messenger
import signal
from std_msgs.msg import Bool, String
from octo_ros.msg import PrinterState, PrintPartAction


counter = 0

class RosInterface(object):
    def __init__(self):
        rospy.init_node('printerWatcher', anonymous=True, disable_signals=True)
        rospy.loginfo("---- OctoROS Initialized! ----")
        self.print_pub = rospy.Publisher('printer3d1/printer3d', PrinterState, queue_size=100)
        self.printFinished_pub = rospy.Publisher('printer3d1/printer3d/finishedPrinting', Bool, queue_size=10)
        # Starts/sets up the Action server for our 3D Printer
        self.printerServer = actionlib.SimpleActionServer('printer_3D_server', PrintPartAction, self.getFileCallback, False)
        self.printerServer.start()
        self.rate = rospy.Rate(0.1)  # 0.1 hz

    def getFileCallback(self, goal):
        """ Waits for the file name from the action client and then calls the addExtension function """
        # Extracts just the content from the goal
        fileToPrint = goal.file_to_print
        self.printerServer.set_succeeded()
        self.addExtension(fileToPrint)

    def addExtension(self, fileToPrint):
        """ Adds .gcode extension to the wished file, because on the Action Client one should provide only the file name without the extension """
        self.extension = ".gcode"
        fileToPrint = fileToPrint + self.extension
        self.searchFile(fileToPrint)

    def searchFile(self, fileToPrint):
        """ Searches for files with extension .gcode.
        NOTE: This works only when using roslaunch because roslaunch points to the /src directory directly, while running with
        rosrun runs the script locally. Because of the cwd="node" on the launch file """

        files = os.listdir(".")
        gcode_files = []
        rospy.loginfo("---- Searching for requested file ----")
        for file in files:
            if file == fileToPrint:
                rospy.loginfo("---- Sending requested file to OctoPrint ----")
                self.printPartAndGetStatus(fileToPrint)
                # If the file has been found we exit this function
                return 0
            elif file.endswith(self.extension):
                gcode_files.append(file)
        if file != fileToPrint:
            notFoundString = "Requested file: %s not found in the package /src directory" %fileToPrint
            rospy.logwarn(notFoundString)
            rospy.logwarn("Files with the extension .gcode that have been found: ")
            for i in range(len(gcode_files)):
                print(gcode_files[i])

    def getDateTime(self):
        """ Get the actual date and time. """
        date_time = datetime.datetime.now()
        date_time = str(date_time)
        return date_time

    def countTimeStamp(self):
        """ Currently implementing our own time stamp to count the messages that are being sent.
        TODO: implement it using Header() and seq"""
        global counter
        counter += 1
        return int(counter)

    def countTime(self, ts):
        """ Takes the timestamp and 'converts' it to count the time since the printer has started to print.  """
        global counter
        if counter >= 2:
            # Transform to the actual time
            ts = ts - 1
            timeInSeconds = ts*10
            return int(timeInSeconds)

    def printPartAndGetStatus(self, fileToPrint):
        """ Sends command to print the wished part and sends all the data retrieved from the printer to ROS """
        connectedToServer = False
        try:
            printing = messenger.printModel(fileToPrint)
            connectedToServer = True
            #print("esperamos que ele nao printa")

        except(AttributeError, requests.exceptions.RequestException, requests.exceptions.ConnectionError, requests.exceptions.HTTPError):
            rospy.loginfo("Printer not connected to the OctoPrint server.")
            sys.exit()

        if connectedToServer:
                bootString = "Starting to print model %s" %fileToPrint
                rospy.loginfo(bootString)

                # Just retrieving progress
                progress, _, _, _ = messenger.printingProgressTracking()
                rospy.loginfo("Started retrieving data from 3D Printer. Hear to the topic if you want to see the streamed data.")

                try:
                    while progress < 100 and not rospy.is_shutdown():
                        # Update progress and get all the remaining data
                        progress, printingTimeLeft, fileName, fileSize = messenger.printingProgressTracking()

                        try:
                            # Retrieving all data
                            bedTempA, bedTempT, tool0TempA, tool0TempT, state, tool1TempA, tool1TempT = messenger.getprinterInfo()
                            date_time = self.getDateTime()
                            ts = self.countTimeStamp()
                            timeElapsed = self.countTime(ts)
                            if timeElapsed == None:
                                timeElapsed = 0

                            # Change made to work with the Graph fom Grafana, since the division by 0 is invalid
                            elif tool0TempT == 0.0:
                                tool0TempT = 1.0
                            elif bedTempT == 0.0:
                                bedTempT = 1.0

                            # Check if this is really needed
                            #elif tool1TempT == 0.0:
                            #    tool1TempT = 1.0
                            #elif tool1TempA == 0.0:
                            #    tool1TempA = 1.0

                            # Encapsulate all the data to send to the publisher
                            pstate = PrinterState()
                            pstate.timestamp = ts
                            pstate.date_time = date_time
                            pstate.temp_tool1_actual = tool0TempA
                            pstate.temp_tool2_actual = tool1TempA
                            pstate.temp_bed_actual = bedTempA
                            pstate.file_name = fileName
                            pstate.file_size = fileSize
                            pstate.printer3d_state = state
                            pstate.progress = progress
                            pstate.time_elapsed = timeElapsed
                            pstate.time_left = printingTimeLeft
                            pstate.temp_tool1_goal = tool0TempT
                            pstate.temp_tool2_goal = tool1TempT
                            pstate.temp_bed_goal = bedTempT
                            self.print_pub.publish(pstate)
                            self.rate.sleep()
                        except(ValueError,UnboundLocalError):
                            rospy.logerr("Not connected to the printer, check if the USB cable is connected and if the port is enabled.")
                            sys.exit()
                except(KeyboardInterrupt):
                    #Keyboardinterrupt
                    rospy.logwarn("Printing preempted.")

        else:
            return None

        rospy.loginfo("Sucessful printing.")
        self.printFinished_pub.publish(True)
        self.printerServer.set_succeeded()


def signal_handler(signal, frame):
    """ Function to handle the printing cancellation, too much overheading and the printing wasn't been cancelled. """
    sys.exit(0)

if __name__ == '__main__':
    interf = RosInterface()
    rospy.spin()
    signal.signal(signal.SIGINT, signal_handler)
    print('Stop printing forced, Sucessful printing shoudl not be printed')
    messenger.cancelPrinting()
