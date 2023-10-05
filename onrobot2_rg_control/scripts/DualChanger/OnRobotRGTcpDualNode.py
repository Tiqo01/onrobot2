#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import onrobot2_rg_control.baseOnRobotRG
import onrobot2_rg_modbus_tcp.comModbusTcp
from std_srvs.srv import Trigger
from onrobot2_rg_msgs.msg import OnRobotRGInput
from onrobot2_rg_msgs.msg import OnRobotRGOutput
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time


class OnRobotDualRGTcp(Node):

    def __init__(self):
        super().__init__('OnRobotDualRGTcp')
        # Dual quicker changer addresses for both sides using ModBus/TCP
        primary_address = 66  # 0x42
        secondary_address = 67  # 0x43

        self.declare_parameter('/onrobot/ip', '192.168.1.1')
        self.declare_parameter('/onrobot/port', 502)
        self.declare_parameter('/onrobot/gripper_primary', 'rg2')
        self.declare_parameter('/onrobot/gripper_secondary', 'rg2')
        self.declare_parameter('/onrobot/dummy', False)
       
        ip = self.get_parameter('/onrobot/ip').get_parameter_value().string_value
        port = self.get_parameter('/onrobot/port').get_parameter_value().integer_value 
        gtype_prime = self.get_parameter('/onrobot/gripper_primary').get_parameter_value().string_value
        gtype_second = self.get_parameter('/onrobot/gripper_secondary').get_parameter_value().string_value
        dummy = self.get_parameter('/onrobot/dummy').get_parameter_value().bool_value 


        # Primary side Gripper on Dual Changer Connection
        self.gripper_primary = \
            onrobot2_rg_control.baseOnRobotRG.onrobotbaseRG(gtype_prime)
        self.gripper_primary.client = \
            onrobot2_rg_modbus_tcp.comModbusTcp.communication(dummy)
        self.gripper_primary.client.connectToDevice(
            ip, port, primary_address)

        # Secondary side Gripper on Dual changer Connection
        self.gripper_secondary = \
            onrobot2_rg_control.baseOnRobotRG.onrobotbaseRG(gtype_second)
        self.gripper_secondary.client = \
            onrobot2_rg_modbus_tcp.comModbusTcp.communication(dummy)
        self.gripper_secondary.client.connectToDevice(
            ip, port, secondary_address)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        # Grippers Status publish
        self.pub_primary_gripper = self.create_publisher(
            OnRobotRGInput, 'OnRobotRGInput_A', 1)
        self.pub_secondary_gripper = self.create_publisher(
            OnRobotRGInput,'OnRobotRGInput_B', 1)

        # Gripper Commandds reception
        self.create_subscription(OnRobotRGOutput,
                         'OnRobotRGOutput_A',
                         self.gripper_primary.refreshCommand, qos_profile)
        self.create_subscription(OnRobotRGOutput,
                         'OnRobotRGOutput_B',
                         self.gripper_secondary.refreshCommand, qos_profile)

        # The restarting service
        self.create_service(
            Trigger,
            "/onrobot_rg/restart_power",
            self.restartPowerCycle)

        self.mainLoop()

    def restartPowerCycle(self, request, response):
        """ Restarts the power cycle of the gripper. """
        rclpy.logging.get_logger('node_logger').info("Restart the power cycle of all grippers connected.")
        self.gripper_primary.restartPowerCycle()
        time.sleep(1)
        response.success=None
        response.message=None
        return response

    def mainLoop(self):
        """ Loops the sending status and command, and receiving message. """

        prev_msg_prime = []
        prev_msg_second = []
        while not rclpy.ok():
            # Getting grippers status
            status_primary = self.gripper_primary.getStatus()
            status_secondary = self.gripper_secondary.getStatus()
            # Publishing status
            self.pub_primary_gripper.publish(status_primary)
            self.pub_secondary_gripper.publish(status_secondary)

            time.sleep(0.05)
            # Updating command of primary side
            if not int(format(status_primary.gsta, '016b')[-1]):  # not busy
                # Getting new messages
                if not prev_msg_prime == self.gripper_primary.message:
                    rclpy.logging.get_logger('node_logger').info(
                        rclpy.get_name()+": Sending Message A Side")
                    self.gripper_primary.sendCommand()
                prev_msg_prime = self.gripper_primary.message
            # Updating command of secondary side
            if not int(format(status_secondary.gsta, '016b')[-1]):  # not busy
                # Getting new messages
                if not prev_msg_second == self.gripper_secondary.message:
                    rclpy.logging.get_logger('node_logger').info(
                        rclpy.get_name()+": Sending Message B Side")
                    self.gripper_secondary.sendCommand()
                prev_msg_second = self.gripper_secondary.message
            time.sleep(0.05)


if __name__ == '__main__':
    rclpy.init()
    node = OnRobotDualRGTcp()
    rclpy.spin(node)
