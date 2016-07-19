#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyRequest
from pr2_power_board.srv import PowerBoardCommand2, PowerBoardCommand2Request


class CalibrationWarning(object):
    def __init__(self):
        self.init_successful = True
        try:
            rospy.wait_for_service('pr2_etherCAT/halt_motors', 60)
            self.halt_motors_client = rospy.ServiceProxy('pr2_etherCAT/halt_motors', Empty)
            rospy.loginfo("[%s] Found halt motors service" % rospy.get_name())
        except:
            rospy.logerr("[%s] Cannot find halt motors service" % rospy.get_name())
            self.init_successful = False

        try:
            rospy.wait_for_service('pr2_etherCAT/reset_motors', 60)
            self.reset_motors_client = rospy.ServiceProxy('pr2_etherCAT/reset_motors', Empty)
            rospy.loginfo("[%s] Found reset motors service" % rospy.get_name())
        except:
            rospy.logerr("[%s] Cannot find halt motors service" % rospy.get_name())
            self.init_successful = False

        try:
            rospy.wait_for_service('power_board/control2', 60)
            self.power_board_client = rospy.ServiceProxy('power_board/control2', PowerBoardCommand2)
            rospy.loginfo("[%s] Found power_board/control2 service" % rospy.get_name())
        except:
            rospy.logerr("[%s] Cannot find power_board/control2 service" % rospy.get_name())
            self.init_successful = False

    def start(self):
        """Reset power board, reset motors.  Un-does 'run_stop'."""
        success = [False, False, False]
        for circuit in CIRCUITS:
            success[circuit] = self.reset_power(circuit)
        if success[0] and success[1] and success[2]:
            rospy.sleep(2.0)
            self.reset_motors_client(EmptyRequest())
            return True
        else:
            return False

    def standby_power(self, circuit):
        """Place PR2 power board into standby"""
        stdby_cmd = PowerBoardCommand2Request()
        stdby_cmd.circuit = circuit
        stdby_cmd.command = "stop"
        return self.power_board_client(stdby_cmd)

    def reset_power(self,circuit):
        """Reset PR2 power board to active from standby"""
        reset_cmd = PowerBoardCommand2Request()
        reset_cmd.circuit = circuit
        reset_cmd.command = "start"
        return self.power_board_client(reset_cmd)

def main():
    calibration_warning = CalibrationWarning()
    rospy.spin()
