#!/usr/bin/env python
from detect_joystick import Joystick, JoystickInput
from sensor_msgs.msg import Joy
import rospy


def joystick_publisher(hz: int = 10):
    jinput_pub: rospy.Publisher = rospy.Publisher('joystick_input', Joy, queue_size=10)
    rospy.init_node('joystick_input_pub')
    rate: rospy.Rate = rospy.Rate(hz)
    js: Joystick = Joystick()
    while not rospy.is_shutdown():
        jinput: JoystickInput = js.get_input()
        rospy.logdebug("Joystick Input: %s" % jinput)
        jinput_msg: Joy = jinput.to_msg()
        jinput_pub.publish(jinput_msg)
        rate.sleep()


if __name__ == '__main__':
    joystick_publisher()
