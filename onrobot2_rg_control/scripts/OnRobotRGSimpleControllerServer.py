#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot2_rg_msgs.msg import OnRobotRGOutput
from onrobot2_rg_msgs.srv import SetCommand
import time

class OnRobotRGNode(Node):

    def __init__(self):
        super().__init__('OnRobotRGStatusListener')
        self.pub = self.create_publisher(
            OnRobotRGOutput, 'OnRobotRGOutput', 1)
        self.command = OnRobotRGOutput()
        self.set_command_srv = self.create_service(
            SetCommand,
            "/onrobot_rg/set_command",
            self.handleSettingCommand)
        self.declare_parameter('/onrobot/gripper', 'rg2')

    def handleSettingCommand(self, req, response):
        """ Handles sending commands via socket connection. """

        rclpy.logging.get_logger('node_logger').info(str(req.command))
        self.command = self.genCommand(str(req.command), self.command)
        self.pub.publish(self.command)
        time.sleep(1)
        response.success=None
        response.message=None
        return reponse

    def genCommand(self, char, command):
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
            command.rgfr = max_force
            command.rgwd = 0
            command.rctr = 16
        elif char == 'o':
            command.rgfr = max_force
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
                command.rgfr = max_force
                command.rgwd = min(max_width, int(char))
                command.rctr = 16
            except ValueError:
                pass

        return command


if __name__ == '__main__':
    rclpy.init()
    node = OnRobotRGNode()
    gtype = node.get_parameter('/onrobot/gripper').get_parameter_value().string_value
    rclpy.spin(node)

