#!/usr/bin/env python3

import rclpy
from onrobot2_rg_msgs.msg import OnRobotRGInput


class onrobotbaseRG:
    """ onrobotbaseRG sends commands and receives the status of RG gripper.

        Attributes:
            gtype (str): gripper type 'rg2' or 'rg6'
            message (list[int]): message including commands to be sent

            verifyCommand:
                Verifies that the value of each variable satisfy its limits.
            refreshCommand:
                Updates the command sent during the next sendCommand() call.
    """

    def __init__(self, gtype):
        # Initiating output message as an empty list
        self.gtype = gtype
        self.message = []

        # Note: after the instantiation,
        # a ".client" member must be added to the object

    def verifyCommand(self, command):
        """ Verifies that the value of each variable satisfy its limits.

            Args:
                command (OnRobotRGOutput): command message to be verified

            Returns:
                command (OnRobotRGOutput): verified command message
        """

        # Verifying that each variable is in its correct range
        if self.gtype == 'rg2':
            max_force = 400
            max_width = 1100
        elif self.gtype == 'rg6':
            max_force = 1200
            max_width = 1600
        else:
            rclpy.logging.get_logger('node_logger').info(": Select the gripper type from rg2 or rg6.")
            rclpy.shutdown()

        command.rgfr = max(0, command.rgfr)
        command.rgfr = min(max_force, command.rgfr)
        command.rgwd = max(0, command.rgwd)
        command.rgwd = min(max_width, command.rgwd)

        # Verifying that the selected mode number is available
        if command.rctr not in [1, 8, 16]:
            rclpy.logging.get_logger('node_logger').info(": Select the mode number from" +
                "1 (grip), 8 (stop), or 16 (grip_w_offset).")
            rclpy.shutdown()

        # Returning the modified command
        return command

    def refreshCommand(self, command):
        """ Updates the command sent during the next sendCommand() call.

            Args:
                command (OnRobotRGOutput): command to be refreshed
        """

        # Limiting the value of each variable
        command = self.verifyCommand(command)

        # Initiating command as an empty list
        self.message = []

        # Building the command with each output variable
        self.message.append(command.rgfr)
        self.message.append(command.rgwd)
        self.message.append(command.rctr)

    def sendCommand(self):
        """ Sends the command to the Gripper. """

        self.client.sendCommand(self.message)

    def restartPowerCycle(self):
        """ Restarts the power cycle of the Gripper. """

        self.client.restartPowerCycle()

    def getStatus(self):
        """ Requests the gripper status and return OnRobotRGInput message.

            Returns:
                message (list[int]): message including commands to be sent
        """

        # Acquiring status from the Gripper
        status = self.client.getStatus()

        # Messaging to output
        message = OnRobotRGInput()

        # Assignning the values to their respective variables
        message.gfof = status[0]
        message.ggwd = status[9]
        message.gsta = status[10]
        message.gwdf = status[17]

        return message
