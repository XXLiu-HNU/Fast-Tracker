#!/usr/bin/env python3
import sys
# sys.path.insert(0, '/home/hdh5/anaconda3/lib/python3.8/site-packages')

import rospy
from std_msgs.msg import Float32MultiArray

# from PyQt5.QtWidgets import QApplication, QWidget
# from PyQt5.Qt import QKeyEvent, Qt

from python_qt_binding.QtWidgets import QApplication, QWidget
# from python_qt_binding.Qt import QKeyEvent
from python_qt_binding.QtCore import Qt

class KeyboardWidget(QWidget):
    def __init__(self, parent=None):
        super(KeyboardWidget, self).__init__(parent)

        self.forward_speed = 0
        self.turn_speed = 0
        rospy.init_node("actor_keyboard")
        self.cmd_pub = rospy.Publisher("/actor_cmd", Float32MultiArray, queue_size=3)
        self.pub_timer = rospy.Timer(rospy.Duration(nsecs=100000000), self.pub_key)
        pass

    def pub_key(self, time_event):
        # print("forward speed: ", self.forward_speed)
        # print("turn speed: ", self.turn_speed)
        data = Float32MultiArray(data=[self.forward_speed, self.turn_speed])
        self.cmd_pub.publish(data)
        pass

    def keyPressEvent(self, event):
        if not event.isAutoRepeat():
            if event.key() == Qt.Key_I:
                self.forward_speed = 1
            if event.key() == Qt.Key_K:
                self.forward_speed = -1
            if event.key() == Qt.Key_J:
                self.turn_speed = -1
            if event.key() == Qt.Key_L:
                self.turn_speed = 1

        return QWidget.keyPressEvent(self, event)

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat():
            if event.key() == Qt.Key_I or event.key() == Qt.Key_K:
                self.forward_speed = 0
            if event.key() == Qt.Key_J or event.key() == Qt.Key_L:
                self.turn_speed = 0

        return QWidget.keyReleaseEvent(self, event)


if __name__ == '__main__':
    app = QApplication(sys.argv)

    widget = KeyboardWidget()
    widget.show()

    app.exec()
    pass
