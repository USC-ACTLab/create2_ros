#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Controller():
    def __init__(self):
        self.lastData = None
        rospy.Subscriber("joy", Joy, self._joyChanged)
        self.pubNav = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def _joyChanged(self, data):
        self.lastData = data
        print(data)

    def run(self):
        while not rospy.is_shutdown():
            msg = Twist()
            if self.lastData != None:
                msg.linear.x = self.lastData.axes[1] * 0.5
                msg.linear.y = self.lastData.axes[3] * 0.5
            self.pubNav.publish(msg)

            rospy.sleep(0.01)


if __name__ == '__main__':
    rospy.init_node('teleop', anonymous=True)
    controller = Controller()
    controller.run()
    # rospy.spin()
