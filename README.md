# joystick-ros
Senses a joystick controller and outputs to a ROS channel

Tested with Logitech F310 Gamepad in DirectInput mode. Tested with ROS Melodic.

Requires:
* Python 3
* ROS
* pygame 2 (tested with 2.0.3)

## Usage
Clone into your ROS workspace's `src` folder and run 
`catkin_make`

Run
 
    src/publish_joystick.py

The joystick input will appear on a topic 
`joystick_input` with message of message type 
`sensor_msgs/Joy`.

If you want to use the helper method, create a symlink 
to `src/detect_joystick.py` in the src of your project 
where you want to use it.

[comment]: <> (If you want to use the helper method, run)

[comment]: <> (    python setup.py install)

[comment]: <> (If you want to use a helper methed add the following to )

[comment]: <> (your code:)

[comment]: <> (    def joystick_input_from_msg&#40;msg&#41; -> JoystickInput:)

[comment]: <> (        return JoystickInput&#40;)

[comment]: <> (            left_stick_vert=msg.axes[1], left_stick_horz=msg.axes[0],)

[comment]: <> (            right_stick_vert=msg.axes[3], right_stick_horz=msg.axes[2],)

[comment]: <> (            buttons=msg.buttons, dpad=tuple&#40;&#40;msg.axes[4], msg.axes[5]&#41;&#41;)

[comment]: <> (        &#41;)

To consume the message setup a subscriber:

    import rospy
    from joystick.detect_joystick import JoystickInput, joystick_input_from_msg
    from sensor_msgs.msg import Joy

    rospy.Subscriber("joystick_sub", Joy, joystick_callback)

    def joystick_callback(data: Joystick_Input) -> None:
        jinput: Joy = joystick_input_from_msg(data)


