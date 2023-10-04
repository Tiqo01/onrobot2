#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot2_rg_msgs.msg import OnRobotRGOutput
from onrobot2_rg_msgs.srv import SetCommand
import time

class OnRobotRGDualNode(Node):
    
    def __init__(self):
        super().__init__('OnRobotDualRGStatusListener')
        self.pub_primary_gripper = self.create_publisher(
             OnRobotRGOutput, 'OnRobotRGOutput_A' , 1)
        self.pub_secondary_gripper = self.create_publisher(
            OnRobotRGOutput, 'OnRobotRGOutput_B', 1)

        self.commandA = OnRobotRGOutput()
        self.commandB = OnRobotRGOutput()
        
        self.declare_parameter('/onrobot/gripper_primary', 'rg2')
        self.declare_parameter('/onrobot/gripper_secondary', 'rg2')

        self.set_command_srv_A = self.create_service(
            SetCommand,
            "/onrobot_rg/set_command_A",
            self.handleCommandA)

        self.set_command_srv_B = self.create_service(
            SetCommand,
            "/onrobot_rg/set_command_B",
            self.handleCommandB)

    def handleCommandA(self, req, res):
        """ Handles sending commands for the gripper A. """

        rclpy.get_logger().info(str(req.command))
        self.command = self.genCommand(
            str(req.command), self.commandA, gtype=gtype_A)
        self.pub_primary_gripper.publish(self.command)
        time.sleep(1)
        res.success=None
        res.message=None
        return res

    def handleCommandB(self, req, res):
        """ Handles sending commands for the gripper B. """

        rclpy.get_logger().info(str(req.command))
        self.command = self.genCommand(
            str(req.command), self.commandB, gtype=gtype_B)
        self.pub_secondary_gripper.publish(self.command)
        time.sleep(1)
        res.success=None
        res.message=None
        return res

    def genCommand(self, char, command, gtype):
        """ Updates the command according to the input character.

            Args:
                char (str): set command service request message
                command (OnRobotRGOutput): command to be sent
                gtype (str): gripper type 'RG2' or 'RG6'

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
    node = OnRobotRGDualNode()
    gtype_A = node.get_parameter(
        '/onrobot/gripper_primary').get_parameter_value().string_value
    gtype_B = node.get_parameter(
        '/onrobot/gripper_secondary').get_parameter_value().string_value
    rclpy.spin(node)
