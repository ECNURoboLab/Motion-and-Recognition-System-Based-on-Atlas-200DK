#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# Copyright: 2016-2019 https://www.corvin.cn ROS小课堂
# Author: corvin
# Description: 为树莓派IMU扩展板所使用的配套代码，由于默认
#    扩展板与树莓派使用IIC连接。所以这里的代码是直接从IIC接口
#    中读取IMU模块的三轴加速度、角度、四元数，然后组装成ROS
#    中的IMU消息格式，发布到/imu话题中，这样有需要的节点可以
#    直接订阅该话题即可获取到imu扩展板当前的数据。
# History:
#    20191031: Initial this file.
#    20191209: 新增发布IMU芯片温度的话题，发布频率与发布imu数据频率相同.

import rospy
import string
import math
import time
import sys

from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from rasp_imu_hat_6dof.cfg import imuConfig
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from imu_data import MyIMU


degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0

# Callback for dynamic reconfigure requests
def reconfig_callback(config, level):
    global imu_yaw_calibration
    imu_yaw_calibration = config['yaw_calibration']
    rospy.loginfo("Set imu_yaw_calibration to %d" % (imu_yaw_calibration))
    return config

rospy.init_node("imu_node")

# Get DIY config param
data_topic_name = rospy.get_param('~pub_data_topic', 'imu_data')
temp_topic_name = rospy.get_param('~pub_temp_topic', 'imu_temp')
link_name  = rospy.get_param('~link_name', 'base_imu_link')
pub_hz = rospy.get_param('~pub_hz', '10')

data_pub = rospy.Publisher(data_topic_name, Imu, queue_size=1)
temp_pub = rospy.Publisher(temp_topic_name, Float32, queue_size=1)
srv = Server(imuConfig, reconfig_callback)  # define dynamic_reconfigure callback
imuMsg = Imu()

# Orientation covariance estimation
imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

# Angular velocity covariance estimation
imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

# linear acceleration covariance estimation
imuMsg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

seq=0
# sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
accel_factor = 9.806/256.0

myIMU = MyIMU(0x50)
rate = rospy.Rate(pub_hz)

rospy.loginfo("Rasp IMU Module is woking ...")
while not rospy.is_shutdown():
    myIMU.get_YPRAG()

    yaw_deg = float(myIMU.raw_yaw)
    yaw_deg = yaw_deg + imu_yaw_calibration
    if yaw_deg >= 180.0:
        yaw_deg -= 360.0

    #rospy.loginfo("yaw_deg: %f", yaw_deg)
    yaw = yaw_deg*degrees2rad
    pitch = float(myIMU.raw_pitch)*degrees2rad
    roll  = float(myIMU.raw_roll)*degrees2rad
    #rospy.loginfo("yaw:%f pitch:%f  rool:%f", yaw, pitch, roll)

    # Publish imu message
    imuMsg.linear_acceleration.x = float(myIMU.raw_ax)*accel_factor
    imuMsg.linear_acceleration.y = float(myIMU.raw_ay)*accel_factor
    imuMsg.linear_acceleration.z = float(myIMU.raw_az)*accel_factor

    imuMsg.angular_velocity.x = float(myIMU.raw_gx)*degrees2rad
    imuMsg.angular_velocity.y = float(myIMU.raw_gy)*degrees2rad
    imuMsg.angular_velocity.z = float(myIMU.raw_gz)*degrees2rad

    # From IMU module get quatern param
    #myIMU.get_quatern()
    #imuMsg.orientation.x = myIMU.raw_q1
    #imuMsg.orientation.y = myIMU.raw_q2
    #imuMsg.orientation.z = myIMU.raw_q3
    #imuMsg.orientation.w = myIMU.raw_q0

    # Calculate quatern param from roll,pitch,yaw by ourselves
    q = quaternion_from_euler(roll, pitch, yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]

    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = link_name
    imuMsg.header.seq = seq
    seq = seq + 1
    data_pub.publish(imuMsg)

    # Get imu module temperature, then publish to topic
    myIMU.get_temp()
    temp_pub.publish(myIMU.temp)

    rate.sleep()

