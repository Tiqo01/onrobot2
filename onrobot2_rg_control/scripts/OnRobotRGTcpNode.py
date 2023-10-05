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

class OnRobotRGTcp(Node):
    
    def __init__(self):
        super().__init__('OnRobotRGTcpNode')
        self.declare_parameter('/onrobot/ip', '192.168.1.1')
        self.declare_parameter('/onrobot/port', 502)
        self.declare_parameter('/onrobot/gripper', 'rg6')
        self.declare_parameter('/onrobot/changer_addr', 65)
        self.declare_parameter('/onrobot/dummy', False)
        
        ip = self.get_parameter('/onrobot/ip').get_parameter_value().string_value
        port = self.get_parameter('/onrobot/port').get_parameter_value().integer_value
        gtype = self.get_parameter('/onrobot/gripper').get_parameter_value().string_value 
        changer_addr = self.get_parameter('/onrobot/changer_addr').get_parameter_value().integer_value 
        dummy = self.get_parameter('/onrobot/dummy').get_parameter_value().bool_value 
        # Gripper is a RG gripper with a Modbus/TCP connection
        self.gripper = \
            onrobot2_rg_control.baseOnRobotRG.onrobotbaseRG(gtype)
        self.gripper.client = \
            onrobot2_rg_modbus_tcp.comModbusTcp.communication(dummy)

        # Connecting to the ip address received as an argument
        self.gripper.client.connectToDevice(ip, port, changer_addr)

        # The Gripper status is published on the topic 'OnRobotRGInput'
        self.pub = self.create_publisher(
            OnRobotRGInput,'OnRobotRGInput', 1)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        # The Gripper command is received from the topic 'OnRobotRGOutput'
        self.create_subscription(OnRobotRGOutput,
                         'OnRobotRGOutput',
                         self.gripper.refreshCommand,qos_profile)

        # The restarting service
        self.create_service(
            Trigger,
            "/onrobot_rg/restart_power",
            self.restartPowerCycle)

        self.mainLoop()

    def restartPowerCycle(self, request, response):
        """ Restarts the power cycle of the gripper. """
        rclpy.logging.get_logger('node_logger').info("Restarting the power cycle of all grippers connected.")
        self.gripper.restartPowerCycle()
        time.sleep(1)
        response.success=None
        response.message=None        
        return response

    def mainLoop(self):
        """ Loops the sending status and command, and receiving message. """

        prev_msg = []
        while not rclpy.ok():
            # Getting and publish the Gripper status
            status = self.gripper.getStatus()
            self.pub.publish(status)

            time.sleep(0.05)
            # Sending the most recent command
            if not int(format(status.gsta, '016b')[-1]):  # not busy
                if not prev_msg == self.gripper.message:  # find new message
                    rclpy.logging.get_logger('node_logger').info(rclpy.get_name()+": Sending message.")
                    self.gripper.sendCommand()
            prev_msg = self.gripper.message
            time.sleep(0.05)


if __name__ == '__main__':
    rclpy.init()
    node = OnRobotRGTcp()
    rclpy.spin(node)

