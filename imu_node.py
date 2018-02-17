#!/usr/bin/env python
# license removed for brevity
import rospy
import FaBo9Axis_MPU9250
from sensor_msgs.msg import Imu

def talker():
   sensor = FaBo9Axis_MPU9250.MPU9250()
   pub = rospy.Publisher('imu', Imu, queue_size=10)
   rospy.init_node('imu', anonymous=True)
   rate = rospy.Rate(200)  # 10hz
   while not rospy.is_shutdown():
       msg = Imu()
       msg.header.frame_id = "imu_frame"
       msg.header.stamp = rospy.Time.now()
       msg.orientation_covariance[0] = -1

       acc = sensor.readAccel()
       msg.linear_acceleration.x = acc.get('x')
       msg.linear_acceleration.y = acc.get('y')
       msg.linear_acceleration.z = acc.get('z')

       vel = sensor.readGyro()
       msg.angular_velocity.x = vel.get('x')
       msg.angular_velocity.y = vel.get('y')
       msg.angular_velocity.z = vel.get('z')

       pub.publish(msg)
       rate.sleep()
if __name__ == '__main__':
   try:
        talker()
   except rospy.ROSInterruptException:
       pass