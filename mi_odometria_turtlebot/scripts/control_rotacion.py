#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math

roll = pitch = yaw = 0.0
target = 90
kp = 0.5

def get_rotation(msg: Odometry):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x,orientation_q.y, orientation_q.z, orientation_q.w ]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    rospy.logdebug('Angle: ' + str(yaw))

if __name__ == '__main__':
    rospy.init_node('my_quaternion_to_euler', anonymous=True)
    sub = rospy.Subscriber('/odom', Odometry, get_rotation)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    r = rospy.Rate(10)
    command = Twist()

    while not rospy.is_shutdown():
        target_rad = target*math.pi/180
        command.angular.z = kp * (target_rad - yaw)
        pub.publish(command)
        rospy.loginfo(f'target: {target_rad}, current: {yaw}')
        r.sleep()