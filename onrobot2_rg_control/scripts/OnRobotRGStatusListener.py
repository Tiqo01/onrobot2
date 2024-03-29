#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from onrobot2_rg_msgs.msg import OnRobotRGInput


def printStatus(status):
    """ Prints the status string generated by the statusInterpreter. """
    rclpy.logging.get_logger('node_logger').info(statusInterpreter(status))


def OnRobotRGStatusListener():
    """ Initializes the node and subscribe to the OnRobotRGInput topic. """
    global node
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=1
    )
    rclpy.init()
    node = Node('OnRobotRGStatusListener')
    node.create_subscription(OnRobotRGInput, "OnRobotRGInput", printStatus,qos_profile)
    rclpy.spin(node)


def statusInterpreter(status):
    """ Generates a string according to the current status. """

    output = '\n-----\nOnRobot RG status interpreter\n-----\n'

    # gFOF
    output += 'gFOF = ' + str(status.gfof) + ': '
    output += 'Current fingertip offset: ' + str(status.gfof / 10.0) + ' mm\n'

    # gGWD
    output += 'gGWD = ' + str(status.ggwd) + ': '
    output += 'Current width between the gripper fingers (w/o offset): ' + \
              str(status.ggwd / 10.0) + ' mm\n'

    # gSTA
    output += 'gSTA = ' + str(status.gsta) + ': '
    gSTA16bit = format(status.gsta, '016b')
    output += '(gSTA (16 bit) = ' + gSTA16bit + '), Currtent states: '
    if int(gSTA16bit[-1]):
        output += ' A motion is ongoing so new commands are not accepted.'
    if int(gSTA16bit[-2]):
        output += ' An internal- or external grip is detected.'
    if int(gSTA16bit[-3]):
        output += ' Safety switch 1 is pushed.'
    if int(gSTA16bit[-4]):
        output += ' Safety circuit 1 is activated so the gripper cannot move.'
    if int(gSTA16bit[-5]):
        output += ' Safety switch 2 is pushed.'
    if int(gSTA16bit[-6]):
        output += ' Safety circuit 2 is activated so the gripper cannot move.'
    if int(gSTA16bit[-7]):
        output += ' Any of the safety switch is pushed.'

    # gWDF
    output += '\ngWDF = ' + str(status.gwdf) + ': '
    output += 'Current width between the gripper fingers (w offset): ' + \
              str(status.gwdf / 10.0) + ' mm\n'

    return output


if __name__ == '__main__':
    OnRobotRGStatusListener()
