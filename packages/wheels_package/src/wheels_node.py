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
        right = 1
        while not rospy.is_shutdown():
            # detect duck
            # if self.duck_detected:
            #     wheel.vel_right = 0
            #     wheel.vel_left = 0
            #     self.wheels_cmd_pub.publish(wheel)
            #     break
            # # detect red line break for 5 seconds
            if self.red_detected:
                wheel.vel_right = 0
                wheel.vel_left = 0
                self.wheels_cmd_pub.publish(wheel)
                time.sleep(3.5)
                if right == 1:
                    # right turn
                    wheel.vel_right = 0.32
                    wheel.vel_left = 0.79
                    self.wheels_cmd_pub.publish(wheel)
                    right = 2
                    timeout = time.time() + 1.61
                    while True:
                        if time.time() > timeout:
                            break
                elif right == 2:
                    wheel.vel_right = 0.55
                    wheel.vel_left = 0.29
                    self.wheels_cmd_pub.publish(wheel)
                    timeout = time.time() + 3.49
                    right = 0
                    while True:
                        if time.time() > timeout:
                            break
                else:
                    wheel.vel_right = 0
                    wheel.vel_left = 0
                    self.wheels_cmd_pub.publish(wheel)
                    break
            else:
                # Move straight
                wheel.vel_right = 0.33
                wheel.vel_left = 0.3
                self.wheels_cmd_pub.publish(wheel)



if __name__ == "__main__":
    runner = WheelsDriver("wheels_driver_node")
    while not rospy.is_shutdown():
        try:
            runner.run()
            rospy.spin()
        except:
            rospy.on_shutdown(runner.on_shutdown)

