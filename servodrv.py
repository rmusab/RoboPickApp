__author__ = 'ravil'
from serial import *
import os
from threading import Thread

class ServoDrv(object):

    COM_PORT = "/dev/ttyACM0"

    def __init__(self, text_edit):
        try:
            self.ser = Serial(self.COM_PORT, baudrate=9600, timeout=1.0)
        except OSError:
            print "Unable to find com-port " + self.COM_PORT
            return
        os.system("python -m serial.tools.list_ports")
        print ""
        self.text_edit = text_edit

        self.thread = ReceivingThread(self.ser, self.text_edit, self)
        self.thread.start()

        self.theta = [0., 0., 0., 0., 0.]
        self.servo_moving = False

    def send_package(self, srv_cmds):
        for cmd in srv_cmds:
            if cmd is '':
                return
            self.ser.write(self.char(cmd))

    def initiate(self, theta, alpha):
        for i in (1, 2, 3):
            self.theta[i] = 90.0 - theta[i - 1]
        self.theta[0] = 180.0 - alpha
        self.theta[4] = 90.0

    @staticmethod
    def char(x):
        return chr(int(round(x, 0)))

    def wait_for_done(self):
        while True:
            if self.servo_moving is True:
                return
            time.sleep(1)

    def grip(self, test = False):
        time.sleep(10)
        print "\nPreparing to grip..."
        self.servo_moving = True
        if test is False:
            self.send_package([self.theta[0], self.theta[1], 254, self.theta[3], 0, 0])
        else:
            self.send_package([self.theta[0], self.theta[1], 254, self.theta[3], 0, 254])
        self.wait_for_done()

        print "Moving closer..."
        self.servo_moving = True
        self.send_package([254, 254, self.theta[2], 254, 254, 254])
        self.wait_for_done()

        if test is False:
            print "Gripping..."
            self.servo_moving = True
            self.send_package([254, 254, 254, 254, 254, 70])
            self.wait_for_done()
            time.sleep(10)

            self.servo_moving = True
            self.send_package([254, 90, 90, 90, 254, 254])
            self.wait_for_done()

        print "Success!"
        return not test

    def take_to_plate(self):
        print "\nPreparing placing onto the plate..."
        self.servo_moving = True
        self.send_package([self.theta[0], self.theta[1], 254, self.theta[3], 254, 254])
        self.wait_for_done()
        time.sleep(10)

        print "Moving closer..."
        self.servo_moving = True
        self.send_package([254, 254, self.theta[2], 254, 254, 254])
        self.wait_for_done()

        print "Putting object..."
        self.servo_moving = True
        self.send_package([254, 254, 254, 254, 254, 0])
        self.wait_for_done()
        time.sleep(10)

        print "Moving to the centre and closing the gripper..."
        self.servo_moving = True
        self.send_package([254, 90, 90, 90, 254, 254])
        self.wait_for_done()

        self.servo_moving = True
        self.send_package([90, 254, 254, 254, 254, 90])
        self.wait_for_done()

        print "Success!"
        return True

    def close_thread(self):
        self.thread.stop()

    def __del__(self):
        self.ser.close()

class ReceivingThread(Thread):

    def __init__(self, ser, text_edit, servodrv):
        super(ReceivingThread, self).__init__()
        self.ser = ser
        self.stop_flag = False
        self.text_edit = text_edit
        self.servodrv = servodrv

    def run(self):
        while True:
            if self.stop_flag:
                break
            if self.ser.inWaiting() > 0:
                line = self.ser.read(self.ser.inWaiting())
                if line is "Done":
                    self.servodrv.servo_moving = False
                else:
                    self.text_edit.append(line)
            time.sleep(1)

    def stop(self):
        self.stop_flag = True
