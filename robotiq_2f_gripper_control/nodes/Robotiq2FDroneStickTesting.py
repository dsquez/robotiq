#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a 2F gripper.

This serves as an example for publishing messages on the 'Robotiq2FGripperRobotOutput' 
topic using the 'Robotiq2FGripper_robot_output' msg type for sending commands to a 2F gripper.
"""

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep
import swarmlib
import numpy as np


def genCommand(char, command):
    """Update the command according to the character entered by the user."""    
        
    if char == 'a':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 255
        command.rFR  = 150

    if char == 'r':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 0

    if char == 'c':
        command.rPR = 255

    if char == 'o':
        command.rPR = 0   

    #If the command entered is a int, assign this value to rPRA
    try: 
        command.rPR = int(char)
        if command.rPR > 255:
            command.rPR = 255
        if command.rPR < 0:
            command.rPR = 0
    except ValueError:
        pass                    
        
    if char == 'f':
        command.rSP += 25
        if command.rSP > 255:
            command.rSP = 255
            
    if char == 'l':
        command.rSP -= 25
        if command.rSP < 0:
            command.rSP = 0

            
    if char == 'i':
        command.rFR += 25
        if command.rFR > 255:
            command.rFR = 255
            
    if char == 'd':
        command.rFR -= 25
        if command.rFR < 0:
            command.rFR = 0

    # New char for commanding with dronestick for opening
    # and closing
    if char == 'q':
        if command.rPR == 0:
            command.rPR = 90

        elif command.rPR == 90:
            command.rPR = 0
        else:
            command.rPR = 0

    return command
        

def askForCommand(command):
    """Ask the user for a command to send to the gripper."""    

    currentCommand  = 'Simple 2F Gripper Controller\n-----\nCurrent command:'
    currentCommand += '  rACT = '  + str(command.rACT)
    currentCommand += ', rGTO = '  + str(command.rGTO)
    currentCommand += ', rATR = '  + str(command.rATR)
    currentCommand += ', rPR = '   + str(command.rPR )
    currentCommand += ', rSP = '   + str(command.rSP )
    currentCommand += ', rFR = '   + str(command.rFR )


    print currentCommand

    strAskForCommand  = '-----\nAvailable commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += '(0-255): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'
    
    strAskForCommand += '-->'

    return raw_input(strAskForCommand)

def publisher():
    """Main loop which requests new commands and publish them on the 
    Robotiq2FGripperRobotOutput topic."""
    rospy.init_node('Robotiq2FGripperSimpleController')
    
    # Number of values that will be averaged to determine
    # when a drop has occurred
    numvals = 10

    # Initialize a counter for the while loop
    counter = 0

    # Initialize dropcounter. This variable ensures that
    # a command from the dronestick is not sent twice
    dropcounter = 0;

    # Initialize array of previous positions. The length
    # of this array is determined by the variable numvals
    pastPositions = np.zeros((1,numvals),float)

    # Identify drone that is the dronestick
    drone2 = swarmlib.Drone('cf3')

    # Get initial position of drone as a 3 dimensional vector
    pos = drone2.position()

    # Isolate vertical position of drone
    zmax = pos[2]

    # The next two lines were already here. They initialize
    # the publisher and the command variable 
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)

    command = outputMsg.Robotiq2FGripper_robot_output();


    print "Initialized variable"
    print command
    # Automate the initialization of the gripper
    print "Resetting gripper"
    command = genCommand('r', command)

    print "After reset"
    print command
    pub.publish(command)

    rospy.sleep(0.5)
    print "Gripper Reset"

    print "Activating Gripper"
    command = genCommand('a', command)

    print "After activate"
    print command
    pub.publish(command)

    rospy.sleep(2.0)
    print "Gripper Activated"    

    while not rospy.is_shutdown():

        # increment the counter and drop counter
        counter += 1
        dropcounter += 1

        # print for debugging
        #print drone2.position()

        # Get current position and isolate vertical component
        pos = drone2.position()
        posz = pos[2]

        # Update the previous position values
        for i in range(numvals-1):
            pastPositions[0,i] = pastPositions[0,i+1]


        # Add newest position value to the end of the array    
        pastPositions[0,numvals-1] = posz

        # Do not do anything before the vector of previous 
        # positions is filled.
        if counter > numvals:

            # If the current position of the dronestick is
            # below the average of the previous positions,
            # then the dronestick has been pulled down by 
            # the operator.
            if posz < np.mean(pastPositions) - .03 and dropcounter > 5:
                # Reset dropcounter so the command is not sent twice
                dropcounter = 0

                # print for debugging
                print "Something happened"

                # Generate a new command using the new 'q'
                # char as defined in the askforCommand
                # function
                if posz > .1:
                	command = genCommand('q', command) 
                	#print command
                	
                else:
                	command = genCommand('o', command)
                	print "Dronestick below threshold. Opening gripper"

                pub.publish(command)

        
        rospy.sleep(0.1)
                        

if __name__ == '__main__':
    publisher()
