#!/usr/bin/env python

import sys

import rospy
from std_srvs.srv import Empty, EmptyRequest
from pr2_power_board.srv import PowerBoardCommand2, PowerBoardCommand2Request

from hrl_pr2_upstart.srv import SetRunStop


class RunStop(object):

    CIRCUITS = [0, 1, 2]  # Base, Right arm, Left Arm circuits

    def __init__(self):
        self.init_successful = False
        count_found_services = 0
        try:
            rospy.loginfo("[%s] Waiting for halt motors service...", rospy.get_name())
            rospy.wait_for_service('pr2_ethercat/halt_motors', 5)
            self.halt_motors_client = rospy.ServiceProxy('pr2_ethercat/halt_motors', Empty)
            rospy.loginfo("[%s] ...halt motors service found", rospy.get_name())
            count_found_services += 1
        except:
            rospy.logerr("[%s] Cannot find halt motors service", rospy.get_name())

        try:
            rospy.loginfo("[%s] Waiting for reset motors service...", rospy.get_name())
            rospy.wait_for_service('pr2_ethercat/reset_motors', 5)
            self.reset_motors_client = rospy.ServiceProxy('pr2_ethercat/reset_motors', Empty)
            rospy.loginfo("[%s] ...reset motors service found", rospy.get_name())
            count_found_services += 1
        except:
            rospy.logerr("[%s] Cannot find halt motors service", rospy.get_name())

        try:
            rospy.loginfo("[%s] Waiting for power_board/control2 service...", rospy.get_name())
            rospy.wait_for_service('power_board/control2', 5)
            self.power_board_client = rospy.ServiceProxy('power_board/control2', PowerBoardCommand2)
            rospy.loginfo("[%s] ...power_board/control2 service found", rospy.get_name())
            count_found_services += 1
        except:
            rospy.logerr("[%s] Cannot find power_board/control2 service", rospy.get_name())
        # Set the flag for successful setup
        self.init_successful = True if count_found_services == 3 else False

    def stop(self):
        """Halt motors, place power board into standboy. Stops robot."""
        self.halt_motors_client(EmptyRequest())  # Halt motors immediately
        rospy.loginfo("[%s] RunStopEmulator HALTING MOTORS", rospy.get_name())
        success = [False, False, False]
        for circuit in self.CIRCUITS:
            success[circuit] = self.standby_power(circuit)
        if success[0] and success[1] and success[2]:
            return True
        else:
            return False

    def start(self):
        """Reset power board, reset motors.  Un-does 'run_stop'."""
        success = [False, False, False]
        for circuit in self.CIRCUITS:
            success[circuit] = self.reset_power(circuit)
        if success[0] and success[1] and success[2]:
            rospy.sleep(2.0)
            rospy.loginfo("[%s] RunStopEmulator RESETTING MOTORS", rospy.get_name())
            self.reset_motors_client(EmptyRequest())
            return True
        else:
            return False

    def standby_power(self, circuit):
        """Place PR2 power board into standby"""
        rospy.loginfo("[%s] RunStopEmulator placing circuit %d into STANDBY", rospy.get_name(), circuit)
        stdby_cmd = PowerBoardCommand2Request()
        stdby_cmd.circuit = circuit
        stdby_cmd.command = "stop"
        return self.power_board_client(stdby_cmd)

    def reset_power(self, circuit):
        """Reset PR2 power board to active from standby"""
        rospy.loginfo("[%s] RunStopEmulator placing circuit %d into ACTIVE", rospy.get_name(), circuit)
        reset_cmd = PowerBoardCommand2Request()
        reset_cmd.circuit = circuit
        reset_cmd.command = "start"
        return self.power_board_client(reset_cmd)


class RunStopServer(object):
    def __init__(self):
        """Provide dead-man-switch like server for handling wouse run-stops."""
        self.run_stop = RunStop()
        if not self.run_stop.init_successful:
            rospy.logerr("[%s] RunStopEmulator failed to initialize properly! SHUTTING DOWN!", rospy.get_name())
            sys.exit(1)
        rospy.Service("emulate_runstop", SetRunStop, self.service_cb)
        rospy.loginfo("[%s] Run Stop Emulator Ready.", rospy.get_name())

    def service_cb(self, req):
        """Handle service requests to start/stop run-stop.  Used to reset."""
        if req.stop:
            return self.run_stop.stop()
        elif req.start:
            return self.run_stop.start()
        else:
            return True


def main():
    rospy.init_node('run_stop_emulator')
    rs_server = RunStopServer()
    rospy.spin()
