#!/usr/bin/env python3
import select
import sys
import termios
import tty
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray


class ActorKeyboard(object):
    def __init__(self):
        rospy.init_node("actor_keyboard")

        self.forward_speed = 0
        self.turn_speed = 0

        self.test_pub_1 = rospy.Publisher("forward", Int16, queue_size=3)
        self.test_pub_2 = rospy.Publisher("turn", Int16, queue_size=3)
        self.cmd_pub = rospy.Publisher("/actor_cmd", Float32MultiArray, queue_size=3)

        self.pub_timer = rospy.Timer(rospy.Duration(nsecs=100000000), self.pub_key)
        self.timeout_timer = rospy.Timer(rospy.Duration(nsecs=500000000), self.check_timeout)
        self.last_forward_control_time = None
        self.last_turn_control_time = None
        pass

    def check_timeout(self, time_event):
        if self.last_forward_control_time:
            if (rospy.Time.now() - self.last_forward_control_time).to_sec() > 0.2:
                self.forward_speed = 0
        if self.last_turn_control_time:
            if (rospy.Time.now() - self.last_turn_control_time).to_sec() > 0.2:
                self.turn_speed = 0
        pass

    def get_key(self, key_timeout):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
            if key == "i":
                self.forward_speed = 1
                self.last_forward_control_time = rospy.Time.now()
            if key == "j":
                self.turn_speed = -1
                self.last_turn_control_time = rospy.Time.now()
            if key == "l":
                self.turn_speed = 1
                self.last_turn_control_time = rospy.Time.now()
            if key == "u":
                self.forward_speed = 1
                self.turn_speed = -1
                self.last_forward_control_time = rospy.Time.now()
                self.last_turn_control_time = rospy.Time.now()
            if key == "o":
                self.forward_speed = 1
                self.turn_speed = 1
                self.last_forward_control_time = rospy.Time.now()
                self.last_turn_control_time = rospy.Time.now()
            if key == "k":
                self.forward_speed = 0
                self.turn_speed = 0
                self.last_forward_control_time = rospy.Time.now()
                self.last_turn_control_time = rospy.Time.now()

            if key == "q":
                exit(0)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def pub_key(self, time_event):
        self.test_pub_1.publish(self.forward_speed)
        self.test_pub_2.publish(self.turn_speed)
        data = Float32MultiArray(data=[self.forward_speed, self.turn_speed])
        self.cmd_pub.publish(data)
        pass

    def run(self):
        while 1:
            k = self.get_key(0)
        pass


if __name__ == '__main__':
    actor_keyboard = ActorKeyboard()
    actor_keyboard.run()
    rospy.spin()
    pass

