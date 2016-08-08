#!/usr/bin/env python

import subprocess
import numpy as np

import rospy

from pr2_msgs.msg import BatteryServer2
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest

BATTERY_COUNT = 4
STALE_TIMEOUT = 300  # 300s = 5 minutes


class BatteryNotifier(object):
    def __init__(self):
        self.sound_client = SoundClient()
        self.battery_notify_pcts = [99, 90, 75, 50, 30, 20, 10, 5, 3]
        self.battery_notify_pcts.sort()
        self.battery_notify_pcts.reverse()  # Keep in consistent high-to-low order
        self.draining_notified_pcts = []
        self.charging_notified_pcts = []

        self.sound_request_pub = rospy.Publisher('/robotsound', SoundRequest, queue_size=3)
        self.charge_pct_list = [None]*BATTERY_COUNT
        self.charge_pct = None
        self.last_update = [None]*BATTERY_COUNT
        self.power_present_list = [None]*BATTERY_COUNT*4
        self.power_present = None
        self.battery_state_sub = rospy.Subscriber('/battery/server2', BatteryServer2, self.battery_state_cb)
        while None in self.charge_pct_list:
            rospy.sleep(4)
            rospy.logwarn("[%s] Waiting for data on battery server(s)", rospy.get_name())
        rospy.loginfo("[%s] Battery Notifier ready", rospy.get_name())

    def battery_state_cb(self, msg):
        self.charge_pct_list[msg.id] = msg.average_charge

        # Keep track of plug state, beep for power added or removed
        self.power_present_list[msg.id*4:msg.id*4+4] = [batt.power_present for batt in msg.battery]
        power_present_now = bool(sum([x for x in self.power_present_list if x is not None]) > len(self.power_present_list)/2)
        if self.power_present and not power_present_now:
            self.notify_power_disconnected()
        elif not self.power_present and power_present_now:
            self.notify_power_connected()

    def check_status(self):
        pct_now = int(round(np.mean(self.charge_pct_list)))
        if self.charge_pct is None:
            self.charge_pct = pct_now
        charging = "charging" if self.power_present else "draining"
        rospy.loginfo("[%s] Battery %s: %s %%", rospy.get_name(), charging,  pct_now)
        # Warn for low battery percentages
        if pct_now in self.battery_notify_pcts:
            if self.power_present and pct_now not in self.charging_notified_pcts:
                self.notify_pcts(pct_now, charging=True)
            if not self.power_present and pct_now not in self.draining_notified_pcts:
                self.notify_pcts(pct_now, charging=False)

    def notify_power_connected(self):
        self.draining_notified_pcts = []
        subprocess.call(['beep', '-l', '300', '-f', '600', '-n', '-l', '500', '-f', '1000'])
        self.power_present = True

    def notify_power_disconnected(self):
        self.charging_notified_pcts = []
        subprocess.call(['beep', '-l', '300', '-f', '1000', '-n', '-l', '500', '-f', '600'])
        self.power_present = False

    def notify_pcts(self, pct, charging):
        if charging:
            msg = "Battery charged to %s per-cent" % pct
            self.charging_notified_pcts.append(pct)
        else:
            msg = "Battery drained to %s per-cent" % pct
            self.draining_notified_pcts.append(pct)
        self.sound_client.say(msg)


def main():
    rospy.init_node('battery_state_notifier')
    notifier = BatteryNotifier()
    rate = rospy.Rate(0.05)
    while not rospy.is_shutdown():
        notifier.check_status()
        rate.sleep()


if __name__ == '__main__':
    main()
