#!/usr/bin/env python3
"""
hmi_interface.py
================
Nurse-facing Human-Machine Interface (terminal based).

The nurse types a delivery request at the console; this node
publishes it to /delivery_request so mission_manager can act on it.
The node also echoes the current mission state received from
/mission_state so staff can monitor the robot without RViz.

Topics published:   /delivery_request (std_msgs/String)
Topics subscribed:  /mission_state    (std_msgs/String)
                    /mission_alert    (std_msgs/String)  – action prompts from alert_node

Run: rosrun turtlebot2_delivery hmi_interface.py
  or via delivery.launch (launched in a separate xterm window).
"""

import rospy
import threading

from std_msgs.msg import String

STATE_LABELS = {
    'IDLE':         'Robot is at base – ready for a new request.',
    'TO_STOCK':     'Robot is heading to the stock room...',
    'WAITING_LOAD': 'Robot is at the stock room. Please place the medication box on the tray.',
    'TO_WARD':      'Robot is delivering medication to the ward...',
    'DELIVERING':   'Robot is at the ward. Please remove the medication box from the tray.',
    'RETURNING':    'Robot is returning to base...',
}

SEPARATOR = '-' * 56


class HMIInterface:
    def __init__(self):
        rospy.init_node('hmi_interface', anonymous=False)

        self.current_state = 'IDLE'
        self.lock = threading.Lock()

        self.request_pub = rospy.Publisher('/delivery_request', String, queue_size=1)

        rospy.Subscriber('/mission_state', String, self.on_state)
        rospy.Subscriber('/mission_alert', String, self.on_alert)

        # Start keyboard input thread
        input_thread = threading.Thread(target=self.input_loop, daemon=True)
        input_thread.start()

        self.print_banner()
        rospy.spin()

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def on_state(self, msg):
        with self.lock:
            self.current_state = msg.data
        label = STATE_LABELS.get(msg.data, msg.data)
        print(f'\n[STATUS] {label}')
        if msg.data == 'IDLE':
            print('[HMI] Type a new delivery request or press Enter to refresh.')

    def on_alert(self, msg):
        print(f'\n{"=" * 56}')
        print(msg.data)
        print('=' * 56)

    # ------------------------------------------------------------------
    # Input loop
    # ------------------------------------------------------------------

    def input_loop(self):
        while not rospy.is_shutdown():
            try:
                text = input('\n[HMI] Enter delivery request (e.g. "Paracetamol to Room 3"): ').strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not text:
                with self.lock:
                    state = self.current_state
                label = STATE_LABELS.get(state, state)
                print(f'[STATUS] Current state: {state} – {label}')
                continue

            with self.lock:
                state = self.current_state

            if state != 'IDLE':
                print(f'[HMI] Robot is busy ({state}). Request queued – try again after mission.')
                continue

            self.request_pub.publish(String(data=text))
            print(f'[HMI] Request sent: "{text}"')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def print_banner(self):
        print(SEPARATOR)
        print('  TurtleBot2 Medication Delivery – Nurse Terminal')
        print(SEPARATOR)
        print('  Commands:')
        print('    Type any text  → Send delivery request')
        print('    Enter (blank)  → Show current status')
        print('    Ctrl+C         → Exit')
        print(SEPARATOR)


if __name__ == '__main__':
    try:
        HMIInterface()
    except rospy.ROSInterruptException:
        pass
