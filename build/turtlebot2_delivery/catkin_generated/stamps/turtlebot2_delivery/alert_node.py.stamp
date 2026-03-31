#!/usr/bin/env python3
"""
alert_node.py
=============
Translates high-level alert commands from mission_manager into
human-readable text alerts displayed on the HMI terminal.

No buzzer, no LED, no Arduino – all alerts are software only.

Topics subscribed:  /alert_cmd     (std_msgs/String)
Topics published:   /mission_alert (std_msgs/String)  – displayed by hmi_interface
"""

import rospy
from std_msgs.msg import String


ALERT_MESSAGES = {
    'stock_arrived':    '[ALERT] Robot is at the STOCK ROOM. Please place the medication box on the tray.',
    'ward_arrived':     '[ALERT] Robot is at the WARD. Please remove the medication box from the tray.',
    'mission_complete': '[ALERT] Mission complete. Robot has returned to base.',
}


class AlertNode:
    def __init__(self):
        rospy.init_node('alert_node', anonymous=False)

        self.alert_pub = rospy.Publisher('/mission_alert', String, queue_size=5)

        rospy.Subscriber('/alert_cmd', String, self.on_alert)

        rospy.loginfo('[AlertNode] Ready (software alerts only – no buzzer/Arduino).')
        rospy.spin()

    def on_alert(self, msg):
        cmd = msg.data.strip()
        text = ALERT_MESSAGES.get(cmd)
        if text is None:
            rospy.logwarn('[AlertNode] Unknown alert command: %s', cmd)
            return
        rospy.loginfo('%s', text)
        self.alert_pub.publish(text)


if __name__ == '__main__':
    try:
        AlertNode()
    except rospy.ROSInterruptException:
        pass
