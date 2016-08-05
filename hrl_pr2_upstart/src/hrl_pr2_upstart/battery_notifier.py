#!/usr/bin/env python

import subprocess
import numpy as np

import rospy

from pr2_msgs.msg import BatteryServer2
from sound_play.msg import SoundRequest

BATTERY_COUNT = 4
STALE_TIMEOUT = 300 # 300s = 5 minutes


class BatteryNotifier(object):
    def __init__(self):
        self.low_battery_warn_pcts = [100, 75, 50, 20, 10, 5]
        self.low_battery_warn_pcts.sort()
        self.low_battery_warn_pcts.reverse()  # Keep in consistent high-to-low order
        self.low_battery_level_warned = None

        self.sound_request_pub = rospy.Publisher('/robotsound', SoundRequest, queue_size=3)
        self.charge_pct = [None]*BATTERY_COUNT
        self.time_left = [None]*BATTERY_COUNT
        self.last_update = [None]*BATTERY_COUNT
        self.power_present_list = [None]*BATTERY_COUNT*4
        self.power_present = None
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

        # Warn for low battery percentages
        avg_pct = np.mean(self.charge_pct)
        if avg_pct in self.low_battery_warn_pcts
        for i, level in enumerate(self.low_battery_warn_pcts):
            if avg_pct < level:
                warn_level  = level_idx

        # Keep track of plug state, beep for power added or removed
        self.power_present_list[msg.id*4:msg.id*4+4] = [batt.power_present for batt in msg.battery]
        power_present_now = bool(sum([x for x in self.power_present_list if x is not None]) > len(self.power_present_list)/2)
        if self.power_present and not power_present_now:
            subprocess.call(['beep', '-l', '300', '-f', '1000', '-n', '-l', '500', '-f', '600'])
        elif not self.power_present and power_present_now:
            subprocess.call(['beep', '-l', '300', '-f', '600', '-n', '-l', '500', '-f', '1000'])
        self.power_present = power_present_now

    def check_status(self):
        self.average_pct = np.mean(self.charge_pct)
        self.average_time = np.mean(self.charge_pct)


def main():
    rospy.init_node('battery_state_notifier')
    notifier = BatteryNotifier()
    rospy.spin()


if __name__ == '__main__':
    main()
