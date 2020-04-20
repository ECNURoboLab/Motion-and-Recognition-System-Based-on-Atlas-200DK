#!/usr/bin/env python
# _*_ coding:utf-8 _*_

"""
  Copyright(c):2016-2019 ROS小课堂 www.corvin.cn
  Author: corvin
  Description:
    三轮全向移动小车底盘角速度标定的源码文件.
  History:
    20180425: init this file.
    20180919:增加结束时自动计算angular_scale的功能.
"""
import tf
import PyKDL
import rospy
from math import radians, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion


class CalibrateAngular():
    def __init__(self):
        rospy.init_node('calibrate_angular_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        #from config file get param
        self.get_param()

        # How fast will we check the odometry values?
        check_rate = rospy.Rate(self.rate)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(2)

        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(10.0))

        #initial parameters
        self.init_param()
        self.last_angle = 0

        # config move cmd msg
        move_cmd = Twist()
        move_cmd.angular.z = self.speed

        while not rospy.is_shutdown():
            if self.rest_angle > self.tolerance:
                # Get the current rotation angle from tf
                odom_angle = self.get_odom_angle()
                rospy.loginfo("current rotation angle: " + str(odom_angle))

                # Compute how far we have gone since the last measurement
                delta_angle = self.angular_scale*self.normalize_angle(odom_angle - self.last_angle)
                self.turn_angle += abs(delta_angle)

                # Compute the rest angle
                self.rest_angle = self.test_angle - self.turn_angle
                rospy.loginfo("-->rest_angle: " + str(self.rest_angle))

                # Store the current angle for the next comparison
                self.last_angle = odom_angle

                self.cmd_vel.publish(move_cmd)
                check_rate.sleep()
            else: #end test
                self.print_result()

    def get_param(self):
        try:
            self.rate = rospy.get_param("~check_odom_rate", 15)
            self.circle_cnt = rospy.get_param("~test_circle", 2)
            self.test_angle = eval('self.circle_cnt*2*pi')

            self.speed = rospy.get_param("~angular_speed", 0.2) #radians speed
            self.tolerance = rospy.get_param("~tolerance_angle", 0.05)
            self.angular_scale = rospy.get_param("~angular_scale", 1.00)

            cmd_topic = rospy.get_param("~cmd_topic", '/cmd_vel')
            self.cmd_vel = rospy.Publisher(cmd_topic, Twist, queue_size=10)

            # Get base frame and odom frame
            self.base_frame = rospy.get_param('~base_frame', '/base_footprint')
            self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        except:
            rospy.logerr("ERROR: Get config param error from yaml file...")

    def init_param(self):
        self.start_time = rospy.get_time()
        self.rest_angle = self.test_angle
        self.turn_angle = 0.0

    def print_result(self):
        end_time = rospy.get_time() #get test end time
        rospy.logwarn("---------------------------------")
        rospy.logwarn("---Angular Calibrate Completed---")
        rospy.logwarn("---------------------------------")
        rospy.logwarn("Test angle:" + str(self.test_angle))
        rospy.logwarn("Test Time:"  + str(end_time-self.start_time))
        rospy.logwarn("Test Speed:" + str(self.test_angle/(end_time-self.start_time)))
        rospy.logwarn("Input Speed:" + str(self.speed))
        rospy.logwarn("Angular Scale:" + str(self.angular_scale))
        actual_degree = input("Please input actual turn degree:")
        angular_scale = float(radians(actual_degree)/self.test_angle)
        self.angular_scale *= angular_scale
        rospy.logwarn("Now get new angular_scale:" + str(self.angular_scale))
        self.init_param()

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0*pi
        while res < -pi:
            res += 2.0*pi
        return res

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    # Get the current transform between the odom and base frames
    def get_odom_angle(self):
        try:
            (position, quat) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr("get_odom_angle: TF Exception !")
            return
        # Convert the rotation from a quaternion to an Euler angle
        return self.quat_to_angle(Quaternion(*quat))

    def shutdown(self):
        rospy.logwarn("shutdown():Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        CalibrateAngular()
        rospy.spin()
    except:
        rospy.logerr("Error: Calibration terminated.")

