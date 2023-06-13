#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, BoolStamped
from std_msgs.msg import String


HOST = os.environ['VEHICLE_NAME']
cmd = f'/{HOST}/car_cmd_switch_node/cmd'
wheels_cmd_executed = f'/{HOST}/wheels_driver_node/wheels_cmd_executed'

class WheelsNode(DTROS):
    def __init__(self, node_name):
        super(WheelsNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # testing for all Twist2DStamped, WheelsCmdStamped

        # CMD
        self.cmd_pub = rospy.Publisher(
            cmd,
            Twist2DStamped,
            queue_size=1
        )

        self.cmd_sub = rospy.Subscriber(
            cmd,
            Twist2DStamped,
            self.cmd_cb,
            queue_size=1
        )

        # WHEELS_CMD_EXECUTED
        self.wheels_pub = rospy.Publisher(
            wheels_cmd_executed,
            WheelsCmdStamped,
            queue_size=1
        )

        self.wheels_sub = rospy.Subscriber(
            wheels_cmd_executed,
            WheelsCmdStamped,
            self.wheels_cb,
            queue_size=1
        )

    def cmd_cb(self, twist):
        rospy.loginfo(f'Twist: v={twist.v}, omega={twist.omega}')

    def wheels_cb(self, wheel):
        rospy.loginfo(f'Wheel: left={wheel.vel_left}, right={wheel.vel_right}')

    def stop(self):
        wh = self.get_wheel(0,0)
        tw = self.get_twist(0,0)
        self.move(wheel=wh, twist=tw)

    def run(self):
        rospy.Rate(0.5).sleep()
        wh = self.get_wheel(0.5, 0.5)
        tw = self.get_twist(20, 0)
        self.move(wheel=wh, twist=tw)
        rospy.Rate(0.5).sleep()
        rospy.loginfo('-------------------')
        wh = self.get_wheel(1, 0.2)
        tw = self.get_twist(10, 10)
        self.move(wh, tw)
        rospy.Rate(1).sleep()
        self.stop()

    def get_wheel(self, l, r):
        wheel = WheelsCmdStamped()
        wheel.vel_left = l
        wheel.vel_right = r
        return wheel

    def get_twist(self, v, omega):
        twist = Twist2DStamped()
        twist.v = v
        twist.omega = omega
        return twist

    def move(self, wheel, twist):
        self.wheels_pub.publish(wheel)
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    print('started')
    node = WheelsNode(node_name='wheels_node')
    node.run()
    rospy.spin()
