#!/usr/bin/env python3
"""
hmi_interface.py
================
Single-terminal operator interface for the TurtleBot2 delivery system.

All three operator roles are handled here in sequence:
  1. Nurse  – types the delivery request
  2. Stock  – presses ENTER once the medication box is on the tray
  3. Ward   – presses ENTER once the medication box is removed

Topics published:
  /delivery_request  (std_msgs/String)  – triggers the mission
  /load_status       (std_msgs/Bool)    – True=box loaded, False=box removed

Topics subscribed:
  /mission_state  (std_msgs/String)  – drives the sequential prompts
"""

import threading
import rospy
from std_msgs.msg import Bool, String

SEP  = '=' * 56
SEP2 = '-' * 56

STATE_LABELS = {
    'IDLE':         'Robot is at base – ready for a new request.',
    'TO_STOCK':     'Robot is heading to the stock room...',
    'WAITING_LOAD': 'Robot is at the stock room.',
    'TO_WARD':      'Robot is delivering to the ward...',
    'DELIVERING':   'Robot is at the ward.',
    'RETURNING':    'Robot is returning to base...',
}


class DeliveryTerminal:
    def __init__(self):
        rospy.init_node('hmi_interface', anonymous=False)

        self._state = 'IDLE'
        self._lock  = threading.Lock()

        self._request_pub = rospy.Publisher('/delivery_request', String,
                                            queue_size=1)
        self._load_pub    = rospy.Publisher('/load_status', Bool,
                                            queue_size=1, latch=True)

        rospy.Subscriber('/mission_state', String, self._on_state)

        # Mission loop runs in background; rospy.spin() handles callbacks here
        t = threading.Thread(target=self._mission_loop, daemon=True)
        t.start()
        rospy.spin()

    # ------------------------------------------------------------------
    # State callback
    # ------------------------------------------------------------------

    def _on_state(self, msg):
        with self._lock:
            prev = self._state
            self._state = msg.data
        if msg.data != prev:
            label = STATE_LABELS.get(msg.data, msg.data)
            print(f'\n[STATUS] {label}')

    def _get_state(self):
        with self._lock:
            return self._state

    def _wait_for_state(self, target):
        """Block until /mission_state equals target, or ROS shuts down."""
        while not rospy.is_shutdown():
            if self._get_state() == target:
                return True
            rospy.sleep(0.2)
        return False

    # ------------------------------------------------------------------
    # Main sequential loop
    # ------------------------------------------------------------------

    def _mission_loop(self):
        print(SEP)
        print('  TurtleBot2 Medication Delivery – Operator Terminal')
        print(SEP)
        print('  Flow:')
        print('    1. Type a delivery request and press ENTER')
        print('    2. When robot reaches stock room, press ENTER')
        print('    3. When robot reaches ward, press ENTER')
        print(SEP)

        while not rospy.is_shutdown():

            # ── Step 1 : wait until IDLE, then get request ────────────
            while not rospy.is_shutdown() and self._get_state() != 'IDLE':
                rospy.sleep(0.2)
            if rospy.is_shutdown():
                break

            print(f'\n{SEP2}')
            try:
                text = input('[NURSE] Enter delivery request: ').strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not text:
                label = STATE_LABELS.get(self._get_state(), self._get_state())
                print(f'[STATUS] {label}')
                continue

            if self._get_state() != 'IDLE':
                print('[HMI] Robot is busy. Wait for IDLE state and try again.')
                continue

            self._request_pub.publish(String(data=text))
            print(f'[HMI] Request sent: "{text}"')
            print('[...] Waiting for robot to reach the stock room...')

            # ── Step 2 : wait for WAITING_LOAD, then confirm box ─────
            if not self._wait_for_state('WAITING_LOAD'):
                continue

            print(f'\n{SEP}')
            print('[STOCK ROOM] Robot has arrived and is waiting.')
            print('  Place the medication box on the tray.')
            try:
                input('  Press ENTER when the box is on the tray: ')
            except (EOFError, KeyboardInterrupt):
                break

            if self._get_state() == 'WAITING_LOAD':
                self._load_pub.publish(Bool(data=True))
                print('[STOCK ROOM] Box loaded – robot heading to the ward.')
                print('[...] Waiting for robot to reach the ward...')

            # ── Step 3 : wait for DELIVERING, then confirm removal ───
            if not self._wait_for_state('DELIVERING'):
                continue

            print(f'\n{SEP}')
            print('[WARD] Robot has arrived with the medication.')
            print('  Remove the medication box from the tray.')
            try:
                input('  Press ENTER when the box has been removed: ')
            except (EOFError, KeyboardInterrupt):
                break

            if self._get_state() == 'DELIVERING':
                self._load_pub.publish(Bool(data=False))
                print('[WARD] Box removed – robot returning to base.')
                print('[...] Waiting for robot to return...')

            # ── Step 4 : wait for return, then loop ───────────────────
            if not self._wait_for_state('IDLE'):
                continue

            print(f'\n{SEP}')
            print('[DONE] Mission complete! Ready for the next delivery.')
            print(SEP)


if __name__ == '__main__':
    try:
        DeliveryTerminal()
    except rospy.ROSInterruptException:
        pass
