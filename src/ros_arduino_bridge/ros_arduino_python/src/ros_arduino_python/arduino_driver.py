#!/usr/bin/env python

"""
    A Python driver for the Arduino microcontroller running the
    ROSArduinoBridge firmware.
"""

from math import pi as PI
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial
import numpy as np
import thread
import rospy
import time
import os
import smbus

class Arduino:
    ''' Configuration Arduino DUE Board Parameters
    '''
    N_ANALOG_PORTS  = 10
    N_DIGITAL_PORTS = 54

    def __init__(self, port="/dev/ttyAMA0", baudrate=57600, i2c_addr=8, timeout=0.5):
        self.PID_RATE = 40 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 25

        self.i2c_bus = smbus.SMBus(2)
        self.i2c_addr = i2c_addr

        self.port     = port
        self.baudrate = baudrate
        self.timeout  = timeout
        self.encoder_count    = 0
        self.writeTimeout     = timeout
        self.interCharTimeout = timeout/30.

        # Keep things thread safe
        self.mutex = thread.allocate_lock()

        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS

        # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS

    def connect(self):
        try:
            #print "Connecting to Arduino on port :", self.port, "..."
            #self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            # The next line is necessary to give the firmware time to wake up.
            #time.sleep(1)
            test = self.get_baud()
            if test != self.baudrate:
                time.sleep(0.5)
                print "Connecting ..."
                test = self.get_baud()
                if test != self.baudrate:
                    print "get baud:"+str(test)

            # self.beep_ring(1)
            print "Connected at", self.baudrate
            print "Arduino is ready."
        except Exception:
            print sys.exc_info()
            print "Traceback follows:"
            traceback.print_exc(file=sys.stdout)
            print "Cannot connect to Arduino!"
            os._exit(1)

    def open(self):
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self):
        ''' Close the serial port.
        '''
        # self.beep_ring(0)
        self.i2c_bus.close()

    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd + '\r')

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        print "recv timeout:" + timeout
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        '''
        c = ''
        value = ''
        attempts = 0
        while c != '\r':
            #c = self.port.read(1)
            #print "ready get rec char"
            c = self.i2c_bus.read_byte(self.i2c_addr)
            #print "get rec char:" + c
            value += c
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return None

        value = value.strip('\r')
        return value

    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv(self.timeout)
        return ack == 'OK'

    def recv_int(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        value = self.recv(self.timeout)
        try:
            return int(value)
        except:
            return None

    def recv_array(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        try:
            values = self.recv(self.timeout * self.N_ANALOG_PORTS).split()
            return map(int, values)
        except:
            return []

    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning a single integer value.
        '''
        self.mutex.acquire()

        ntries   = 1
        attempts = 0

        try:
            for ch in cmd:
                print "input char:" + ch
                print "iic addr:" + self.i2c_addr
                self.i2c_bus.write_byte(hex(self.i2c_addr), ord('b'))
            self.i2c_bus.write_byte(self.i2c_addr, ord('\r'))
            #time.sleep(0.1)
            #value = self.recv(self.timeout)
            #while attempts < ntries and (value == '' or value == 'Invalid Command' or value == None):
            #    try:
            #        for ch in cmd:
            #            print "input char:" + ch
            #            self.i2c_bus.write_byte(self.i2c_addr, ord(ch))

            #        self.i2c_bus.write_byte(self.i2c_addr, ord('\r'))
            #        value = self.recv(self.timeout)
            #    except:
            #        print "Exception executing command: " + cmd
            #    attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            value = None

        self.mutex.release()
        return value

    def execute_array(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning an array.
        '''
        self.mutex.acquire()
        try:
            self.port.flushInput()
        except:
            pass

        ntries   = 1
        attempts = 0
        try:
            self.port.write(cmd + '\r')
            values = self.recv_array()
            while attempts < ntries and (values == '' or values == 'Invalid Command' or values == [] or values == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    values = self.recv_array()
                except:
                    print("Exception executing command: " + cmd)
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            raise SerialException
            return []

        try:
            values = map(int, values)
        except:
            values = []

        self.mutex.release()
        return values

    def execute_ack(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning True if response is ACK.
        '''
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        ntries   = 1
        attempts = 0

        try:
            self.port.write(cmd + '\r')
            ack = self.recv(self.timeout)
            while attempts < ntries and (ack == '' or ack == 'Invalid Command' or ack == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    ack = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + cmd
                attempts += 1
        except:
            self.mutex.release()
            print "execute_ack exception when executing", cmd
            print sys.exc_info()
            return 0

        self.mutex.release()
        return ack == 'OK'

    def update_pid(self, AWheel_Kp, AWheel_Kd, AWheel_Ki, AWheel_Ko,
                         BWheel_Kp, BWheel_Kd, BWheel_Ki, BWheel_Ko,
                         CWheel_Kp, CWheel_Kd, CWheel_Ki, CWheel_Ko ):
        ''' Set the PID parameters on the Arduino
        '''
        print "Updating PID parameters"
        #cmd = ' '+str(AWheel_Kp)+':'+str(AWheel_Kd)+':'+str(AWheel_Ki)+':'+str(AWheel_Ko)+':'
        #self.i2c_bus.write_i2c_block_data(self.i2c_addr, ord('u'), [ord(c) for c in cmd])

        #cmd = str(BWheel_Kp)+':'+str(BWheel_Kd)+':'+str(BWheel_Ki)+':'+str(BWheel_Ko)+':'
        #self.i2c_bus.write_i2c_block_data(self.i2c_addr, 0, [ord(c) for c in cmd])

        #cmd = str(CWheel_Kp)+':'+str(CWheel_Kd)+':'+str(CWheel_Ki)+':'+str(CWheel_Ko)
        #self.i2c_bus.write_i2c_block_data(self.i2c_addr, 0, [ord(c) for c in cmd])
        #self.execute_ack(cmd)
        #for c in cmd:
            #print(type(c))
        #    time.sleep(0.01)
            #print ""+ch
        #ascii = np.fromstring(cmd, dtype=np.uint8)
        #for asc in ascii:
        #    print(type(asc))
        #    self.i2c_bus.write_byte(self.i2c_addr, ord(chr(asc)))
        #self.i2c_bus.write_byte(self.i2c_addr, ord('\r'))

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        try:
            #return int(self.execute('b'));
            self.i2c_bus.write_byte(self.i2c_addr, ord('b'))
            self.i2c_bus.write_byte(self.i2c_addr, ord('\r'))
            time.sleep(0.1)
            values = self.i2c_bus.read_i2c_block_data(self.i2c_addr, 0, 5)
            values = map(chr, values)
            ret = ''.join(values)
            #print "ret:"+ret
            return int(ret)
        except:
            return None

    def get_encoder_counts(self):
        #values = self.execute_array('e')
        ch = ''
        values = ''
        cnt=0
        cmd="\r"
        #print "get encoder count..."

        try:
            #print "ready send cmd"
            #self.i2c_bus.write_i2c_block_data(self.i2c_addr, ord('e'), [ord(d) for d in cmd])
            self.i2c_bus.write_byte(self.i2c_addr, int(ord('e')))
            self.i2c_bus.write_byte(self.i2c_addr, ord('\r'))
            time.sleep(0.1)
            #print "send encoder cmd..."

            
	    # Read from arduino
	    result_string = ''.join([chr(e) for e in self.i2c_bus.read_i2c_block_data(self.i2c_addr, 0x06)])
            result_flag = result_string.index('\r')
	    values = [int(e) for e in ''.join(result_string[:result_flag]).split(' ')]
	except:
            print sys.exc_info()
            traceback.print_exc(file=sys.stdout)
            return None

        if len(values) != 3:
            print "Encoder count was not 3"
            #raise SerialException
            return None
        else:
            return values

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        #return self.execute_ack('r')
        try:
            self.i2c_bus.write_byte(self.i2c_addr, ord('r'))
            self.i2c_bus.write_byte(self.i2c_addr, ord('\r'))
            #time.sleep(0.1)
        except:
            return None

    def drive(self, AWheel, BWheel, CWheel):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        #rospy.loginfo("drive() A:" + str(AWheel) + ",B:" + str(BWheel) + ",C:" + str(CWheel))
        #return self.execute_ack('m %d %d %d' %(AWheel, BWheel, CWheel))
        cmd = (' %d %d %d\r' %(AWheel, BWheel, CWheel))
        try:
            self.i2c_bus.write_i2c_block_data(self.i2c_addr, ord('m'), [ord(c) for c in cmd])
        except:
            return None

    def drive_m_per_s(self, AWheel, BWheel, CWheel):
        ''' Set the motor speeds in meters per second.
        '''
        aWheel_revs_per_second = float(AWheel) / (self.wheel_diameter * PI)
        bWheel_revs_per_second = float(BWheel) / (self.wheel_diameter * PI)
        cWheel_revs_per_second = float(CWheel) / (self.wheel_diameter * PI)

        aWheel_ticks_per_loop = int(aWheel_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)
        bWheel_ticks_per_loop = int(bWheel_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)
        cWheel_ticks_per_loop = int(cWheel_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)

        self.drive(aWheel_ticks_per_loop , bWheel_ticks_per_loop, cWheel_ticks_per_loop)

    def stop(self):
        ''' Stop all three motors.
        '''
        self.drive(0, 0, 0)

    def analog_read(self, pin):
        return self.execute('a %d' %pin)

    def analog_write(self, pin, value):
        return self.execute_ack('x %d %d' %(pin, value))

    def digital_read(self, pin):
        return self.execute('d %d' %pin)

    def digital_write(self, pin, value):
        return self.execute_ack('w %d %d' %(pin, value))

    def pin_mode(self, pin, mode):
        return self.execute_ack('c %d %d' %(pin, mode))

    def alarm_write(self, value):
        return self.execute_ack('f %d' %value)

    def beep_ring(self, value):
        #return self.execute_ack('p %d' %value)
        self.i2c_bus.write_byte(self.i2c_addr, ord('p'))
        self.i2c_bus.write_byte(self.i2c_addr, ord(' '))
        if(value == 1):
            self.i2c_bus.write_byte(self.i2c_addr, ord('1'))
        else:
            self.i2c_bus.write_byte(self.i2c_addr, ord('0'))

        self.i2c_bus.write_byte(self.i2c_addr, ord('\r'))

    def detect_voltage(self):
        return self.execute('g')

    def detect_current(self):
        return self.execute('f')

    def light_show(self, value):
        return self.execute_ack('l %d' %value)

    def get_pidin(self):
        values = self.execute_array('i')
        if len(values) != 3:
            print "pidin was not 3"
            raise SerialException
            return None
        else:
            return values

    def get_pidout(self):
        values = self.execute_array('o')
        if len(values) != 3:
            print "pidout was not 3"
            raise SerialException
            return None
        else:
            return values


""" Basic test for connectivity """
if __name__ == "__main__":
    if os.name == "posix":
        portName = "/dev/ttyACM0"
    else:
        portName = "COM43" # Windows style COM port.

    baudRate = 57600

    myArduino = Arduino(port=portName, baudrate=baudRate, timeout=0.5)
    myArduino.connect()

    print "Sleeping for 1 second..."
    time.sleep(1)

    print "Reading on analog port 0", myArduino.analog_read(0)
    print "Reading on digital port 0", myArduino.digital_read(0)
    print "Blinking the LED 3 times"
    for i in range(3):
        myArduino.digital_write(13, 1)
        time.sleep(1.0)
    #print "Current encoder counts", myArduino.encoders()
    print "Connection test successful.",

    myArduino.stop()
    myArduino.close()

    print "Shutting down Arduino."

