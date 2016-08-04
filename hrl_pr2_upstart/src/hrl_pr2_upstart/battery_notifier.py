#!/usr/bin/env python

import rospy

from pr2_msgs.msg import BatteryServer2
from sound_play.msg import SoundRequest

BATTERY_COUNT = 4


class BatteryNotifier(object):
    def __init__(self):
        self.low_battery_warn_pcts = []

        self.sound_request_pub = rospy.Publisher('/robotsound', SoundRequest, queue=3)
        self.charge_pct = [None]*BATTERY_COUNT
        self.time_left = [None]*BATTERY_COUNT
        self.last_update = [None]*BATTERY_COUNT
        self.battery_state_sub = rospy.Subscriber('/battery/server2', BatteryServer2, self.battery_state_cb)
        while any([state is None for state in self.charge_pct]):
            rospy.sleep(4)
            rospy.logwarn("[%s] Waiting for data on battery server(s)", rospy.get_name())
        rospy.loginfo("[%s] Battery Notifier ready", rospy.get_name())

    def battery_state_cb(self, msg):
        if self.charge_pct[msg.id] is None:
            rospy.loginfo("[%s] Received data on battery %d", rospy.get_name(), msg.id)
        self.charge_pct[msg.id] = msg.average_charge
        self.time_left[msg.id] = msg.time_left
        self.last_update[msg.id] = msg.last_system_update

    def check_status(self):
        self.average_pct = np.mean(self.charge_pct)
        self.average_time = np.mean(self.charge_pct)

        if self.average_pct <





def main():
    rospy.init_node('battery_state_notifier')
    notifier = BatteryNotifier()
    rospy.spin()


if __name__=='__main__':
    main()
