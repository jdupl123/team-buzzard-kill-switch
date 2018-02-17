#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16, Int16, Char

class publisher(object):

    brake_msg = Int16()
    throttle_msg = Int16()
    gear_msg = Char()

    brake_pub = rospy.Publisher('brake', Int16, queue_size=10)
    throttle_pub = rospy.Publisher('throttle', Int16, queue_size=10)
    gear_pub = rospy.Publisher('gear', Char, queue_size=10)

    def joy_callback(self, joy_data):
        self.brake_msg = max(0,255*joy_data.axes[1]*-1)
        self.throttle_msg = max(0,255*joy_data.axes[1])
        if joy_data.buttons[4] > 0:
            self.gear_msg = 'P'
        elif joy_data.buttons[5] > 0:
            self.gear_msg = 'D'

    def joy_republisher(self):
        rospy.init_node('joy_republish', anonymous=True)
        rospy.Subscriber("joy", Joy, self.joy_callback)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.brake_pub.publish(self.brake_msg)
            self.throttle_pub.publish(self.throttle_msg)
            self.gear_pub.publish(self.gear_msg)
            rate.sleep()

if __name__ == '__main__':
   try:
       publisher_ = publisher()
       publisher_.joy_republisher()
   except rospy.ROSInterruptException:
       pass