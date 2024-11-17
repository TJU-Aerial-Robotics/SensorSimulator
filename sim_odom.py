#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import tf

def publish_odometry():
    rospy.init_node('odom_publisher', anonymous=True)
    odom_pub = rospy.Publisher('/sim/odom', Odometry, queue_size=10)
    rate = rospy.Rate(30)  # 30 Hz

    odom_msg = Odometry()
    odom_msg.header.frame_id = "world"
    odom_msg.child_frame_id = "base_link"

    x_position = 0.0
    y_position = 2.0
    z_position = 1.6
    velocity = 2.0  # m/s

    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = (current_time - start_time).to_sec()
        x_position = velocity * elapsed_time  # x = v * t

        # Update odometry message
        odom_msg.header.stamp = current_time
        odom_msg.pose.pose.position = Point(x_position, y_position, z_position)
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        odom_msg.pose.pose.orientation = Quaternion(*q)
        odom_msg.twist.twist.linear.x = velocity
        odom_msg.twist.twist.linear.y = 0
        odom_msg.twist.twist.linear.z = 0
        odom_msg.twist.twist.angular.x = 0
        odom_msg.twist.twist.angular.y = 0
        odom_msg.twist.twist.angular.z = 0

        # Publish the odometry message
        odom_pub.publish(odom_msg)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_odometry()
    except rospy.ROSInterruptException:
        pass
