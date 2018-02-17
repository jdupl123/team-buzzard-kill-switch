#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, Char

class publisher(object):

    brake_msg = Int16()
    throttle_msg = Int16()
    gear_msg = Int16()
    ignition_msg = Int16()
    starter_msg = Int16()
    steering_msg = Int16()

    brake_pub = rospy.Publisher('brake', Int16, queue_size=10)
    throttle_pub = rospy.Publisher('throttle', Int16, queue_size=10)
    gear_pub = rospy.Publisher('gear', Int16, queue_size=10)
    ignition_pub = rospy.Publisher('ignition', Int16, queue_size=10)
    starter_pub = rospy.Publisher('starter', Int16, queue_size=10)
    steering_pub = rospy.Publisher('steering', Int16, queue_size=10)

    def joy_callback(self, joy_data):
        self.brake_msg = max(0,255*joy_data.axes[1]*-1)
        self.throttle_msg = max(0,255*joy_data.axes[1])

        if joy_data.buttons[4] > 0:
            self.gear_msg = 68 # P
        elif joy_data.buttons[5] > 0:
            self.gear_msg = 80 # D

        if joy_data.buttons[6] > 0:
            self.ignition_msg = 0
        elif joy_data.buttons[7] > 0:
            self.ignition_msg = 255

        if (joy_data.axes[2] < 0) & (joy_data.axes[5] < 0):
            self.starter_msg = 255
        else:
            self.starter_msg = 0

        self.steering_msg = 255*0.5*(joy_data.axes[3] + 1)


    def joy_republisher(self):
        rospy.init_node('joy_republish', anonymous=True)
        rospy.Subscriber("joy", Joy, self.joy_callback)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.brake_pub.publish(self.brake_msg)
            self.throttle_pub.publish(self.throttle_msg)
            self.gear_pub.publish(self.gear_msg)
            self.ignition_pub.publish(self.ignition_msg)
            self.starter_pub.publish(self.starter_msg)
            self.steering_pub.publish(self.steering_msg)
            rate.sleep()

if __name__ == '__main__':
   try:
       publisher_ = publisher()
       publisher_.joy_republisher()
   except rospy.ROSInterruptException:
       pass