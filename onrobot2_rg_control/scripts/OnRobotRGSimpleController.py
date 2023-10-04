#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot2_rg_msgs.msg import OnRobotRGOutput


def genCommand(char, command):
    """ Updates the command according to the input character.

        Args:
            char (str): set command service request message
            command (OnRobotRGOutput): command to be sent

        Returns:
            command: command message with parameters set
    """

    if gtype == 'rg2':
        max_force = 400
        max_width = 1100
    elif gtype == 'rg6':
        max_force = 1200
        max_width = 1600
    else:
        rclpy.shutdown()

    if char == 'c':
        command.rgfr = 400
        command.rgwd = 0
        command.rctr = 16
    elif char == 'o':
        command.rgfr = 400
        command.rgwd = max_width
        command.rctr = 16
    elif char == 'i':
        command.rgfr += 25
        command.rgfr = min(max_force, command.rgfr)
        command.rctr = 16
    elif char == 'd':
        command.rgfr -= 25
        command.rgfr = max(0, command.rgfr)
        command.rctr = 16
    else:
        # If the command entered is a int, assign this value to rgwd
        try:
            command.rgfr = 400
            command.rgwd = min(max_width, int(char))
            command.rctr = 16
        except ValueError:
            pass

    return command


def askForCommand(command):
    """ Asks the user for a command to send to the gripper.

        Args:
            command (OnRobotRGOutput): command to be sent

        Returns:
            input(strAskForCommand) (str): input command strings
    """

    currentCommand = 'Simple OnRobot RG Controller\n-----\nCurrent command:'
    currentCommand += ' rgfr = ' + str(command.rgfr)
    currentCommand += ', rgwd = ' + str(command.rgwd)
    currentCommand += ', rctr = ' + str(command.rctr)

    node.get_logger().info(currentCommand)

    strAskForCommand = '-----\nAvailable commands\n\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += '(0 - max width): Go to that position\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'

    strAskForCommand += '-->'

    return input(strAskForCommand)


def publisher():
    """ Main loop which requests new commands and
        publish them on the OnRobotRGOutput topic.
    """
    global node
    rclpy.init()
    node = Node('OnRobotRGSimpleController')
    pub = node.create_publisher(
        OnRobotRGOutput, 'OnRobotRGOutput', 1)
    node.declare_parameter('/onrobot/gripper', 'rg2')
    global gtype 
    gtype = node.get_parameter('/onrobot/gripper').get_parameter_value().string_value
    command = OnRobotRGOutput()
    rate = node.create_rate(10)

    while rclpy.ok():
        command = genCommand(askForCommand(command), command)
        pub.publish(command)
        rate.sleep()


if __name__ == '__main__':
    publisher()
