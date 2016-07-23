#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from pr2_msgs.msg import PowerBoardState


class RunStopAsBool(object):
    def __init__(self):
        self.current_status = None
        self.power_state_sub = rospy.Subscriber('/power_board/state', PowerBoardState, self.state_update_cb)
        self.runstop_status_pub = rospy.Publisher('/runstop_status', Bool, latch=True, queue_size=5)

    def state_update_cb(self, pbs_msg):
        if self.current_status is None:
            self.runstop_status_pub.publish(pbs_msg.run_stop)
        elif (pbs_msg.run_stop and not self.current_status):
            self.runstop_status_pub.publish(True)
        elif (not pbs_msg.run_stop and self.current_status):
            self.runstop_status_pub.publish(False)
        self.current_status = pbs_msg.run_stop


def main():
    rospy.init_node('runstop_status_pub')
    converter = RunStopAsBool()
    rospy.spin()


if __name__ == '__main__':
    main()
