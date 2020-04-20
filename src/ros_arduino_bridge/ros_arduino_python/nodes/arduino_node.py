#!/usr/bin/env python

"""
    A ROS Node for the Arduino microcontroller
"""

import os
import rospy
import thread
from ros_arduino_python.arduino_driver import Arduino
from ros_arduino_python.arduino_sensors import *
from ros_arduino_msgs.srv import *
from ros_arduino_python.base_controller import BaseController
from geometry_msgs.msg import Twist
from serial.serialutil import SerialException

class ArduinoROS():
    def __init__(self):
        rospy.init_node('mobilebase_arduino_node', log_level=rospy.INFO)

        # Get the actual node name in case it is set in the launch file
        self.name = rospy.get_name()

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        self.port = rospy.get_param("~port", "/dev/ttyAMA0")
        self.i2c_addr = rospy.get_param("~i2c_addr", 0x04)
        self.baud = int(rospy.get_param("~baud", 57600))
        self.timeout    = rospy.get_param("~timeout", 0.7)
        self.base_frame = rospy.get_param("~base_frame", 'base_footprint')

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 20))
        loop_rate = rospy.Rate(self.rate)

        # Rate at which summary SensorState message is published. Individual sensors publish
        # at their own rates.
        self.sensorstate_rate = int(rospy.get_param("~sensorstate_rate", 10))

        # Set up the time for publishing the next SensorState message
        now = rospy.Time.now()
        self.t_delta_sensors = rospy.Duration(1.0 / self.sensorstate_rate)
        self.t_next_sensors  = now + self.t_delta_sensors

        # Initialize a Twist message
        self.cmd_vel = Twist()

        # A cmd_vel publisher so we can stop the robot when shutting down
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=20)

        # The publisher periodically publishes the values of all sensors on a single topic.
        self.sensorStatePub = rospy.Publisher('~sensor_state', SensorState, queue_size=5)

        # A service to turn set the direction of a digital pin (0 = input, 1 = output)
        rospy.Service('~digital_set_direction', DigitalSetDirection, self.DigitalSetDirectionHandler)

        # A service to turn a digital sensor on or off
        rospy.Service('~digital_write', DigitalWrite, self.DigitalWriteHandler)

        # A service to read the value of a digital sensor
        rospy.Service('~digital_read', DigitalRead, self.DigitalReadHandler)

        # A service to set pwm values for the pins
        rospy.Service('~analog_write', AnalogWrite, self.AnalogWriteHandler)

        # A service to read the value of an analog sensor
        rospy.Service('~analog_read', AnalogRead, self.AnalogReadHandler)

        # A service to set alarm sensor
        rospy.Service('~alarm_write', AlarmWrite, self.AlarmWriteHandler)

        # A service to set light show
        rospy.Service('~light_show', LightShow, self.LightShowHandler)

        # Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.i2c_addr, self.timeout)

        # Make the connection
        self.controller.connect()
        #rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")

        # Reserve a thread lock
        mutex = thread.allocate_lock()

        # Initialize any sensors
        self.mySensors = list()
        sensor_params = rospy.get_param("~sensors", dict({}))

        for name, params in sensor_params.iteritems():
            # Set the direction to input if not specified
            try:
                params['direction']
            except:
                params['direction'] = 'input'

            if params['type'] == "GP2D12":
                sensor = GP2D12(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == "IR2Y0A02":
                sensor = IR2Y0A02(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == "GP2Y0A41":
                sensor = GP2Y0A41(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'Digital':
                sensor = DigitalSensor(self.controller, name, params['pin'], params['rate'], self.base_frame, direction=params['direction'])
            elif params['type'] == 'Analog':
                sensor = AnalogSensor(self.controller, name, params['pin'], params['rate'], self.base_frame, direction=params['direction'])
            elif params['type'] == 'Voltage':
                sensor = VoltageSensor(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'MotorTotalCurrent':
                sensor = MotorTotalCurrent(self.controller, name, params['pin'], params['rate'], self.base_frame)

            try:
                self.mySensors.append(sensor)
                rospy.loginfo(name + " " + str(params) + " published on topic " + rospy.get_name() + "/sensor/" + name)
            except:
                rospy.logerr("Sensor type " + str(params['type']) + " not recognized.")

        # Initialize the base controller
        self.myBaseController = BaseController(self.controller, self.base_frame, self.name + "_base_controller")

        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
            # Base controller
            mutex.acquire()
            self.myBaseController.poll()
            mutex.release()

            for sensor in self.mySensors:
                mutex.acquire()
                sensor.poll()
                mutex.release()

            # Publish all sensor values on a single topic for convenience
            now = rospy.Time.now()
            if now > self.t_next_sensors:
                msg = SensorState()
                msg.header.frame_id = self.base_frame
                msg.header.stamp    = now
                for i in range(len(self.mySensors)):
                    msg.name.append(self.mySensors[i].name)
                    msg.value.append(self.mySensors[i].value)
                try:
                    self.sensorStatePub.publish(msg)
                except:
                    pass

                self.t_next_sensors = now + self.t_delta_sensors

            loop_rate.sleep()

    def DigitalSetDirectionHandler(self, req):
        self.controller.pin_mode(req.pin, req.direction)
        return DigitalSetDirectionResponse()

    def DigitalWriteHandler(self, req):
        self.controller.digital_write(req.pin, req.value)
        return DigitalWriteResponse()

    def DigitalReadHandler(self, req):
        value = self.controller.digital_read(req.pin)
        return DigitalReadResponse(value)

    def AnalogWriteHandler(self, req):
        self.controller.analog_write(req.pin, req.value)
        return AnalogWriteResponse()

    def AnalogReadHandler(self, req):
        value = self.controller.analog_read(req.pin)
        return AnalogReadResponse(value)

    def AlarmWriteHandler(self, req):
        self.controller.alarm_write(req.value)
        return AlarmWriteResponse()

    def LightShowHandler(self, req):
        self.controller.light_show(req.value)
        return LightShowResponse()

    # Stop the robot
    def shutdown(self):
        rospy.logwarn("Shutting down Arduino Node...")
        try:
            rospy.logwarn("Stopping the robot...")
            self.cmd_vel_pub.Publish(Twist())
            rospy.sleep(2)
        except:
            pass

        # Close the serial port
        try:
            self.controller.close()
        except:
            pass
        finally:
            rospy.logwarn("Serial port closed.")
            os._exit(0)


if __name__ == '__main__':
    try:
        myArduino = ArduinoROS()
    except SerialException:
        rospy.logerr("Serial exception trying to open port.")
        os._exit(0)

