#!/usr/bin/env python3
"""
mission_manager.py
==================
Central state machine for the TurtleBot2 medication delivery system.

State diagram:
  IDLE ──(delivery_request)──► TO_STOCK
                                    │
                              (arrived at stock)
                                    │
                              WAITING_LOAD ──(box detected by camera)──► TO_WARD
                                                                             │
                                                                       (arrived at ward)
                                                                             │
                                                                        DELIVERING ──(box gone)──► RETURNING
                                                                                                      │
                                                                                                (arrived at base)
                                                                                                      │
                                                                                                    IDLE

Topics subscribed:
  /delivery_request  (std_msgs/String)  – nurse sends medication name / room
  /load_status       (std_msgs/Bool)    – True when camera sees box on tray

Topics published:
  /mission_state  (std_msgs/String)  – current state label
  /alert_cmd      (std_msgs/String)  – command string for alert_node

Action client:
  move_base  (move_base_msgs/MoveBaseAction)
"""

import rospy
import yaml
import os
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool, String
from actionlib_msgs.msg import GoalStatus


STATES = ['IDLE', 'TO_STOCK', 'WAITING_LOAD', 'TO_WARD', 'DELIVERING', 'RETURNING']


def load_waypoints():
    """Load waypoint poses from config/waypoints.yaml."""
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    yaml_path = os.path.join(pkg_dir, 'config', 'waypoints.yaml')
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data['waypoints']


def build_goal(waypoint_data):
    """Convert a waypoint dict entry into a MoveBaseGoal."""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    pos = waypoint_data['position']
    ori = waypoint_data['orientation']
    goal.target_pose.pose = Pose(
        position=Point(x=pos['x'], y=pos['y'], z=0.0),
        orientation=Quaternion(x=ori['x'], y=ori['y'], z=ori['z'], w=ori['w'])
    )
    return goal


class MissionManager:
    def __init__(self):
        rospy.init_node('mission_manager', anonymous=False)
        rospy.loginfo('[MissionManager] Starting...')

        self.state = 'IDLE'
        self.waypoints = load_waypoints()

        # move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('[MissionManager] Waiting for move_base action server...')
        self.client.wait_for_server()
        rospy.loginfo('[MissionManager] move_base connected.')

        # Publishers
        self.state_pub = rospy.Publisher('/mission_state', String, queue_size=1, latch=True)
        self.alert_pub = rospy.Publisher('/alert_cmd',    String, queue_size=1)

        # Subscribers
        rospy.Subscriber('/delivery_request', String, self.on_request)
        rospy.Subscriber('/load_status',      Bool,   self.on_load_status)

        self.publish_state()
        rospy.loginfo('[MissionManager] Ready. Waiting for /delivery_request...')
        rospy.spin()

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def on_request(self, msg):
        if self.state != 'IDLE':
            rospy.logwarn('[MissionManager] Busy (%s). Ignoring request: %s', self.state, msg.data)
            return
        rospy.loginfo('[MissionManager] Delivery request received: %s', msg.data)
        self.transition('TO_STOCK')
        self.navigate_to('stock_room')

    def on_load_status(self, msg):
        if msg.data and self.state == 'WAITING_LOAD':
            rospy.loginfo('[MissionManager] Box detected on tray. Heading to ward.')
            self.transition('TO_WARD')
            self.navigate_to('ward')
        elif not msg.data and self.state == 'DELIVERING':
            rospy.loginfo('[MissionManager] Box removed at ward. Returning to base.')
            self.transition('RETURNING')
            self.navigate_to('base')

    # ------------------------------------------------------------------
    # Navigation helpers
    # ------------------------------------------------------------------

    def navigate_to(self, name):
        if name not in self.waypoints:
            rospy.logerr('[MissionManager] Unknown waypoint: %s', name)
            return
        goal = build_goal(self.waypoints[name])
        rospy.loginfo('[MissionManager] Navigating to %s', name)
        self.client.send_goal(goal, done_cb=self.on_goal_done)

    def on_goal_done(self, status, result):
        if status != GoalStatus.SUCCEEDED:
            rospy.logwarn('[MissionManager] Navigation ended with status %s', status)
            # Simple recovery: try to return to base
            if self.state not in ('RETURNING', 'IDLE'):
                self.transition('RETURNING')
                self.navigate_to('base')
            return

        if self.state == 'TO_STOCK':
            rospy.loginfo('[MissionManager] Arrived at stock room. Waiting for box...')
            self.transition('WAITING_LOAD')
            self.alert_pub.publish('stock_arrived')

        elif self.state == 'TO_WARD':
            rospy.loginfo('[MissionManager] Arrived at ward. Waiting for box pickup...')
            self.transition('DELIVERING')
            self.alert_pub.publish('ward_arrived')

        elif self.state == 'RETURNING':
            rospy.loginfo('[MissionManager] Back at base. Mission complete.')
            self.transition('IDLE')
            self.alert_pub.publish('mission_complete')

    # ------------------------------------------------------------------
    # State management
    # ------------------------------------------------------------------

    def transition(self, new_state):
        assert new_state in STATES, f'Invalid state: {new_state}'
        rospy.loginfo('[MissionManager] %s → %s', self.state, new_state)
        self.state = new_state
        self.publish_state()

    def publish_state(self):
        self.state_pub.publish(self.state)


if __name__ == '__main__':
    try:
        MissionManager()
    except rospy.ROSInterruptException:
        pass
