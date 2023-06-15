#!/usr/bin/env python3

import os
import time

import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Bool

CAR = os.environ['VEHICLE_NAME']
wheels_cmd = f'/{CAR}/wheels_driver_node/wheels_cmd'
wheels_cmd_executed = f'/{CAR}/wheels_driver_node/wheels_cmd_executed'


class WheelsDriver(DTROS):
    def __init__(self, node_name):
        super(WheelsDriver, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        self.wheels_cmd_pub = rospy.Publisher(
            wheels_cmd,
            WheelsCmdStamped,
            queue_size=1
        )

        self.red_detected_sub = rospy.Subscriber(
            'red_detected',
            Bool,
            self.red_detected_cb,
            queue_size=1
        )

        self.red_detected = False

    def red_detected_cb(self, msg):
        self.red_detected = msg.data

    def run(self):
        wheel = WheelsCmdStamped()
        while not rospy.is_shutdown():
            if self.red_detected:
                # Stop
                wheel.vel_right = 0
                wheel.vel_left = 0
                self.wheels_cmd_pub.publish(wheel)
                time.sleep(3)
                wheel.vel_right = 0.23
                wheel.vel_left = 0.8
                self.wheels_cmd_pub.publish(wheel)
                timeout = time.time() + 5
                while True:
                    if time.time() > timeout:
                        break
                    if not self.red_detected:
                        wheel.vel_right = 0.23
                        wheel.vel_left = 0.2
                        self.wheels_cmd_pub.publish(wheel)
            else:
                # Move straight
                wheel.vel_right = 0.23
                wheel.vel_left = 0.2

            self.wheels_cmd_pub.publish(wheel)

    def on_shutdown(self):
        rospy.loginfo('Shutting down...')
        wheel = WheelsCmdStamped()
        wheel.vel_right = 0
        wheel.vel_left = 0
        self.wheels_cmd_pub.publish(wheel)


if __name__ == "__main__":
    runner = WheelsDriver("wheels_driver_node")
    while not rospy.is_shutdown():
        try:
            runner.run()
            rospy.spin()
        except:
            rospy.on_shutdown(runner.on_shutdown)

