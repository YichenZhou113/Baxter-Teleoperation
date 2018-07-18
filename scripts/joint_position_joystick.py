#!/usr/bin/env python

# -*- coding: utf8 -*-
#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#

import sys
import csv
import zmq

import time
from time import sleep

import numpy as np

import math
import argparse

import baxter_interface
from sensor_msgs.msg import Joy

from baxter_interface import CHECK_VERSION

import rospy
from rospy import Duration

"""setting the global variables"""
class ButtonTransition(object):
    """
    Monitor button transitions.
    The transition is measured when read.
    """
    def __init__(self, val_func, down_val=True, up_val=False):
        self._raw_value = val_func
        self._down_val = down_val
        self._up_val = up_val
        self._up_checked = True
        self._down_checked = False

    def down(self):
        val = self._raw_value()
        if val == self._down_val:
            if not self._down_checked:
                self._down_checked = True
                return True
        else:
            self._down_checked = False
        return False

    def up(self):
        val = self._raw_value()
        if val == self._up_val:
            if not self._up_checked:
                self._up_checked = True
                return True
        else:
            self._up_checked = False
        return False


class StickTransition(object):
    """
    Monitor transitions in stick movement.
    The transition is measured when read.
    """
    def __init__(self, val_func, epsilon=0.05):
        self._raw_value = val_func
        self._epsilon = epsilon
        self._value = 0.0

    def value(self):
        self.changed()
        return self._value

    def changed(self):
        value = self._raw_value()
        if abs(value - self._value) > self._epsilon:
            self._value = value
            return True
        return False

    def increased(self):
        value = self._raw_value()
        if (value - self._value) > self._epsilon:
            self._value = value
            return True
        return False

    def decreased(self):
        value = self._raw_value()
        if (self._value - value) > self._epsilon:
            self._value = value
            return True
        return False


class Joystick(object):
    """
    Abstract base class to handle joystick input.
    """

    def __init__(self, scale=1.0, offset=0.0, deadband=0.1):
        """
        Maps joystick input to robot control.
        @type scale: float
        @param scale: scaling applied to joystick values [1.0]
        @type offset: float
        @param offset: joystick offset values, post-scaling [0.0]
        @type deadband: float
        @param deadband: deadband post scaling and offset [0.1]
        Raw joystick valuess are in [1.0...-1.0].
        """
        sub = rospy.Subscriber("/joy", Joy, self._on_joy)
        self._scale = scale
        self._offset = offset
        self._deadband = deadband
        self._controls = {}

        self._buttons = {}
        self._sticks = {}
        button_names = (
            'btnLeft', 'btnUp', 'btnDown', 'btnRight',
            'dPadUp', 'dPadDown', 'dPadLeft', 'dPadRight',
            'leftBumper', 'rightBumper',
            'leftTrigger', 'rightTrigger',
            'leftStickClick', 'rightStickClick',
            'function1', 'function2')
        stick_names = (
            'leftStickHorz', 'leftStickVert',
            'rightStickHorz', 'rightStickVert')

        #doing this with lambda won't work
        def gen_val_func(name, type_name):
            def val_func():
                return type_name(
                    name in self._controls and self._controls[name])
            return val_func

        for name in button_names:
            self._buttons[name] = ButtonTransition(gen_val_func(name, bool))
        for name in stick_names:
            self._sticks[name] = StickTransition(gen_val_func(name, float))

    def _on_joy(self, msg):
        """ callback for messages from joystick input
        Args:
              msg(Joy): a joystick input message
        """
        raise NotImplementedError()

    def button_up(self, name):
        return self._buttons[name].up()

    def button_down(self, name):
        return self._buttons[name].down()

    def stick_changed(self, name):
        return self._sticks[name].changed()

    def stick_inc(self, name):
        return self._sticks[name].increased()

    def stick_dec(self, name):
        return self._sticks[name].decreased()

    def stick_value(self, name):
        """
        Returns:
            the deadbanded, scaled and offset value of the axis
        """
        value = self._sticks[name].value()
        if value > self._deadband or value < -self._deadband:
            return (value * self._scale) + self._offset
        return 0


class XboxController(Joystick):
    """
    Xbox specialization of Joystick.
    """
    def __init__(self, scale=1.0, offset=0.0, deadband=0.5):
        super(XboxController, self).__init__(scale, offset, deadband)

    def _on_joy(self, msg):
        """ callback for messages from joystick input
        Args:
              msg(Joy): a joystick input message
        """

        self._controls['btnLeft'] = (msg.buttons[2] == 1)
        self._controls['btnUp'] = (msg.buttons[3] == 1)
        self._controls['btnDown'] = (msg.buttons[0] == 1)
        self._controls['btnRight'] = (msg.buttons[1] == 1)

        self._controls['leftStickClick'] = (msg.buttons[9] == 1)
        self._controls['rightStickClick'] = (msg.buttons[10] == 1)

        self._controls['dPadUp'] = (msg.axes[7] > 0.5)
        self._controls['dPadDown'] = (msg.axes[7] < -0.5)
        self._controls['dPadLeft'] = (msg.axes[6] > 0.5)
        self._controls['dPadRight'] = (msg.axes[6] < -0.5)

        self._controls['leftStickHorz'] = msg.axes[0]
        self._controls['leftStickVert'] = msg.axes[1]
        self._controls['rightStickHorz'] = msg.axes[3]
        self._controls['rightStickVert'] = msg.axes[4]

        self._controls['leftBumper'] = (msg.buttons[4] == 1)
        self._controls['rightBumper'] = (msg.buttons[5] == 1)
        self._controls['leftTrigger'] = (msg.axes[2] < 0.0)
        self._controls['rightTrigger'] = (msg.axes[5] < 0.0)

        self._controls['function1'] = (msg.buttons[6] == 1)
        self._controls['function2'] = (msg.buttons[7] == 1)


class LogitechController(Joystick):
    """
    Logitech specialization of Joystick.
    """
    def __init__(self, scale=1.0, offset=0.0, deadband=0.1):
        super(LogitechController, self).__init__(scale, offset, deadband)

    def _on_joy(self, msg):
        """ callback for messages from joystick input
        Args:
              msg(Joy): a joystick input message
        """
        self._controls['btnLeft'] = (msg.buttons[2] == 1)
        self._controls['btnUp'] = (msg.buttons[3] == 1)
        self._controls['btnDown'] = (msg.buttons[0] == 1)
        self._controls['btnRight'] = (msg.buttons[1] == 1)

        self._controls['dPadUp'] = (msg.axes[7] > 0.5)
        self._controls['dPadDown'] = (msg.axes[7] < -0.5)
        self._controls['dPadLeft'] = (msg.axes[6] > 0.5)
        self._controls['dPadRight'] = (msg.axes[6] < -0.5)

        self._controls['leftStickHorz'] = msg.axes[0]
        self._controls['leftStickVert'] = msg.axes[1]
        self._controls['rightStickHorz'] = msg.axes[3]
        self._controls['rightStickVert'] = msg.axes[4]

        self._controls['leftBumper'] = (msg.buttons[4] == 1)
        self._controls['rightBumper'] = (msg.buttons[5] == 1)
        self._controls['leftTrigger'] = (msg.axes[2] < 0.0)
        self._controls['rightTrigger'] = (msg.axes[5] < 0.0)

        self._controls['function1'] = (msg.buttons[6] == 1)
        self._controls['function2'] = (msg.buttons[7] == 1)

class PS3Controller(Joystick):
    """
    PS3 specialization of Joystick.
    """
    def __init__(self, scale=1.0, offset=0.0, deadband=0.1):
        super(PS3Controller, self).__init__(scale, offset, deadband)

    def _on_joy(self, msg):
        """ callback for messages from joystick input
        Args:
              msg(Joy): a joystick input message
        """

        self._controls['btnLeft'] = (msg.buttons[15] == 1)
        self._controls['btnUp'] = (msg.buttons[12] == 1)
        self._controls['btnDown'] = (msg.buttons[14] == 1)
        self._controls['btnRight'] = (msg.buttons[13] == 1)

        self._controls['dPadUp'] = (msg.buttons[4] == 1)
        self._controls['dPadDown'] = (msg.buttons[6] == 1)
        self._controls['dPadLeft'] = (msg.buttons[7] == 1)
        self._controls['dPadRight'] = (msg.buttons[5] == 1)

        self._controls['leftStickHorz'] = msg.axes[0]
        self._controls['leftStickVert'] = msg.axes[1]
        self._controls['rightStickHorz'] = msg.axes[2]
        self._controls['rightStickVert'] = msg.axes[3]

        self._controls['leftBumper'] = (msg.buttons[10] == 1)
        self._controls['rightBumper'] = (msg.buttons[11] == 1)
        self._controls['leftTrigger'] = (msg.buttons[8] == 1)
        self._controls['rightTrigger'] = (msg.buttons[9] == 1)

        self._controls['function1'] = (msg.buttons[0] == 1)
        self._controls['function2'] = (msg.buttons[3] == 1)


#left_current_
left_dest_matrix = [[0] * 7 for i in range(100)]
right_dest_matrix = [[0] * 7 for i in range(100)]
temp_matrix = [[0] * 7] * 100
joint_buffer = [0] * 14
left_joint_buffer = [0] * 7
right_joint_buffer = [0] * 7
left_amount_buffer = [0] * 7
right_amount_buffer = [0] * 7
left_dest_buffer = [0] * 7
right_dest_buffer = [0] * 7
mode_left = 'continuous mode'
mode_right = 'continuous mode'                                                      #set the global modes for each limb
ksphere_size = '1'                                                                  #global size of how much further
plane = "m"                                                                         #global plane choose
left_row_destination = {'left_s0':0, 'left_s1':0, 'left_e0':0, 'left_e1':0, 'left_w0':0, 'left_w1':0, 'left_w2':0}
right_row_destination = {'right_s0':0, 'right_s1':0, 'right_e0':0, 'right_e1':0, 'right_w0':0, 'right_w1':0, 'right_w2':0}
left_row_dictionary = {'left_s0':0, 'left_s1':0, 'left_e0':0, 'left_e1':0, 'left_w0':0, 'left_w1':0, 'left_w2':0}
right_row_dictionary = {'right_s0':0, 'right_s1':0, 'right_e0':0, 'right_e1':0, 'right_w0':0, 'right_w1':0, 'right_w2':0}       #joint angles information for each limb
left_joints = ['left_s1', 'left_e1', 'left_w1']
right_joints = ['right_s1', 'right_e1', 'right_w1']                                 #joint selection for each limb
left_modes = ['continuous mode', 'direct mode']
right_modes = ['continuous mode', 'direct mode']                                    #mode selection for each limb
wanting_row = {}                                                                    #stores the temporary row information
testcase = 'passed'                                                                 #check for whether can reach specific position
left_count = 0
right_count = 0

def set_ksphere_size(size):
    global ksphere_size
    ksphere_size = size                                                             #set the extension size

def set_plane(level):
    global plane
    plane = level                                                                   #set the plane level

def rotate(thelist):
    """
    Rotates a list left.
    @param l: the list
    """
    if len(thelist):
        temp = thelist[0]
        thelist[:-1] = thelist[1:]
        thelist[-1] = temp
        print(thelist)                                                              #rotate the joints

def rotate_mode(thelist, limb):
    """
    Rotates a list left.
    @param l: the list
    """
    global mode_left
    global mode_right
    if len(thelist):
        temp = thelist[0]
        thelist[:-1] = thelist[1:]
        thelist[-1] = temp
        if limb == 'left':
            mode_left = thelist[0]
        if limb == 'right':
            mode_right = thelist[0]                                                 #rotate the modes for each limb


def set_j(cmd, limb, move):
    global left_row_dictionary
    global right_row_dictionary
    global wanting_row
    global testcase
    global left_joints
    global right_joints
    global left_modes
    global right_modes
    global mode_left
    global mode_right
    global joint_buffer
    global left_dest_buffer
    global left_dest_matrix
    global right_dest_matrix
    global left_amount_buffer
    global right_amount_buffer
    global left_joint_buffer
    global right_joint_buffer
    global left_count
    global right_count
    global temp_matrix
    #global ksphere_size
    movement = plane + '_' + move
    if limb == 'left':
        joint = left_joints[0]
        mode_left = left_modes[0]
        global_limb = 'left'
    if limb == 'right':
        joint = right_joints[0]
        mode_right = right_modes[0]
        global_limb = 'right'
    if joint == 'left_s1':
        body_part = 'left_link123'
    if joint == 'left_e1':
        body_part = 'left_link23'
    if joint == 'left_w1':
        body_part = 'left_link3'
    if joint == 'right_s1':
        body_part = 'right_link123'
    if joint == 'right_e1':
        body_part = 'right_link23'
    if joint == 'right_w1':
        body_part = 'right_link3'                                                   #set the parameters to search in the csv file
    if movement == 'm_centre':
        body_part = 'left_link123'
        #ksphere_size = 1

    print(joint)
    csv_file = csv.reader(open('baxter_databank.csv', "rb"), delimiter=",")
    for row in csv_file:
        if movement == row[0] and body_part == row[1] and ksphere_size == row[3]:
            wanting_row = row
            testcase = 'passed'                                                     #store the row information
            break
        testcase = 'failed'                                                         #no such position in database


    if testcase == 'passed':
        if limb == 'left':
            left_row_dictionary['left_s0'] = float(wanting_row[4])
            left_row_dictionary['left_s1'] = float(wanting_row[5])
            left_row_dictionary['left_e0'] = float(wanting_row[6])
            left_row_dictionary['left_e1'] = float(wanting_row[7])
            left_row_dictionary['left_w0'] = float(wanting_row[8])
            left_row_dictionary['left_w1'] = float(wanting_row[9])
            left_row_dictionary['left_w2'] = float(wanting_row[10])
            """for i in range(4,11):
                joint_buffer.insert(i-4, float(wanting_row[i]))
                #print(len(joint_buffer))
            if len(joint_buffer) > 7:
                joint_buffer = joint_buffer[:7]"""

            left_dest_buffer.insert(0,left_row_dictionary['left_s0'])
            left_dest_buffer.insert(1,left_row_dictionary['left_s1'])
            left_dest_buffer.insert(2,left_row_dictionary['left_e0'])
            left_dest_buffer.insert(3,left_row_dictionary['left_e1'])
            left_dest_buffer.insert(4,left_row_dictionary['left_w0'])
            left_dest_buffer.insert(5,left_row_dictionary['left_w1'])
            left_dest_buffer.insert(6,left_row_dictionary['left_w2'])
            #print('111111')
            #print(left_dest_buffer)
            #print('222222')
            if left_joint_buffer[0] == 0 and left_joint_buffer == 0:
                left_joint_buffer.insert(0,0)
                left_joint_buffer.insert(1,0)
                left_joint_buffer.insert(2,0)
                left_joint_buffer.insert(3,0)
                left_joint_buffer.insert(4,0)
                left_joint_buffer.insert(5,0)
                left_joint_buffer.insert(6,0)

            if len(joint_buffer) > 7:
                left_joint_buffer = left_joint_buffer[:7]
            #print('333333')
            #print(left_joint_buffer)
            #joint_buffer = left_joint_buffer + right_joint_buffer
            left_amount_buffer = [x - y for x, y in zip(left_dest_buffer, left_joint_buffer)]
            #print('444444')
            #print(left_amount_buffer)
            left_amount_buffer = [x/100 for x in left_amount_buffer]
            #print('555555')
            #print(left_amount_buffer)



            for i in range(100):
                for j in range(7):
                    left_dest_matrix[i][j] = float((i+1) * left_amount_buffer[j]) + float(left_joint_buffer[j])
            #print('wahahahahahaha')
            #print(left_dest_matrix)

            left_count = 0

        if limb == 'right':
            right_row_dictionary['right_s0'] = float(wanting_row[12])
            right_row_dictionary['right_s1'] = float(wanting_row[13])
            right_row_dictionary['right_e0'] = float(wanting_row[14])
            right_row_dictionary['right_e1'] = float(wanting_row[15])
            right_row_dictionary['right_w0'] = float(wanting_row[16])
            right_row_dictionary['right_w1'] = float(wanting_row[17])
            right_row_dictionary['right_w2'] = float(wanting_row[18])               #fill in the dictionary for setting positions
            """for i in range(12,19):
                joint_buffer.insert(i-12, float(wanting_row[i]))
            if len(joint_buffer) > 7:
                joint_buffer = joint_buffer[:7]"""

            right_dest_buffer.insert(0,right_row_dictionary['right_s0'])
            right_dest_buffer.insert(1,right_row_dictionary['right_s1'])
            right_dest_buffer.insert(2,right_row_dictionary['right_e0'])
            right_dest_buffer.insert(3,right_row_dictionary['right_e1'])
            right_dest_buffer.insert(4,right_row_dictionary['right_w0'])
            right_dest_buffer.insert(5,right_row_dictionary['right_w1'])
            right_dest_buffer.insert(6,right_row_dictionary['right_w2'])
            #print('111111')
            #print(right_dest_buffer)
            #print('222222')
            if right_joint_buffer[0] == 0 and right_joint_buffer[1] == 0:
                right_joint_buffer.insert(0,0)
                right_joint_buffer.insert(1,0)
                right_joint_buffer.insert(2,0)
                right_joint_buffer.insert(3,0)
                right_joint_buffer.insert(4,0)
                right_joint_buffer.insert(5,0)
                right_joint_buffer.insert(6,0)

            if len(joint_buffer) > 7:
                right_joint_buffer = right_joint_buffer[:7]
            #print('333333')
            #print(right_joint_buffer)
            #joint_buffer = right_joint_buffer + right_joint_buffer
            right_amount_buffer = [x - y for x, y in zip(right_dest_buffer, right_joint_buffer)]
            #print('444444')
            #print(right_amount_buffer)
            right_amount_buffer = [x/100 for x in right_amount_buffer]
            #print('555555')
            #print(right_amount_buffer)

            for i in range(100):
                for j in range(7):
                    right_dest_matrix[i][j] = float((i+1) * right_amount_buffer[j]) + float(right_joint_buffer[j])
                    #print(right_dest_matrix[i])


            right_count = 0

    if testcase == 'failed':
        print('cannot reach that position')




def map_joystick(joystick, speed):
    global ksphere_size
    global plane
    global left_row_dictionary
    global right_row_dictionary
    global left_joints
    global right_joints
    global left_modes
    global right_modes
    global mode
    global progress
    global joint_buffer
    global left_joint_buffer
    global right_joint_buffer
    global left_amount_buffer
    global right_amount_buffer
    global left_dest_buffer
    global right_dest_buffer
    global left_dest_matrix
    global right_dest_matrix
    global left_count
    global right_count

    #abbreviations
    jfor_right = lambda s1, s2: (joystick.stick_value(s1) < 0) and (joystick.stick_value(s2) > 0)
    jfor_left = lambda s1, s2: (joystick.stick_value(s1) > 0) and (joystick.stick_value(s2) > 0)
    jback_left = lambda s1, s2: (joystick.stick_value(s1) > 0) and (joystick.stick_value(s2) < 0)
    jback_right = lambda s1, s2: (joystick.stick_value(s1) < 0) and (joystick.stick_value(s2) < 0)
    jhigh = lambda s: joystick.stick_value(s) < 0
    jlow = lambda s: joystick.stick_value(s) > 0
    button_down = joystick.button_down
    button_up = joystick.button_up                                                  #condition check functions

    context = zmq.Context()
    print("Connecting to hello world server")
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")

    def print_help(bindings_list):
        print("Press Ctrl-C to quit.")
        for bindings in bindings_list:
            for (test, _cmd, doc) in bindings:
                if callable(doc):
                    doc = doc()
                print("%s: %s" % (str(test[1][0]), doc))                            #instruction printout

    """bindings_list includes all the condition received from the xbox controller"""

    bindings_list = []
    bindings = (
          ((button_down, ['btnUp']),
           (set_ksphere_size, ['1']), "set k_sphere size to 1"),
          ((button_down, ['btnRight']),
           (set_ksphere_size, ['2']), "set k_sphere size to 2"),
          ((button_down, ['btnDown']),
           (set_ksphere_size, ['3']), "set k_sphere size to 3"),
          ((button_down, ['rightTrigger']),
           (set_j, [right_row_dictionary, 'right', 'centre']), "right arm neatral position"),
          ((button_down, ['leftTrigger']),
           (set_j, [left_row_dictionary, 'left', 'centre']), "left arm neatral position"),
          ((button_down, ['dPadUp']),
           (set_plane, ['h']), "choose high plane"),
          ((button_down, ['dPadRight']),
           (set_plane, ['m']), "choose middle plane"),
          ((button_down, ['dPadLeft']),
           (set_plane, ['m']), "choose middle plane"),
          ((button_down, ['dPadDown']),
           (set_plane, ['d']), "choose low plane"),
          ((button_down, ['leftBumper']),
           (rotate, [left_joints]), lambda: "left: cycle joint" + left_joints[0]),
          ((button_down, ['rightBumper']),
           (rotate, [right_joints]), lambda: "right: cycle joint" + right_joints[0]),
          ((button_down, ['leftStickClick']),
           (rotate_mode, [left_modes, 'left']), lambda: "left: mode change to " + left_modes[0]),
          ((button_down, ['rightStickClick']),
           (rotate_mode, [right_modes, 'right']), lambda: "right: mode change to " + right_modes[0]),
          ((button_down, ['function1']),
           (print_help, [bindings_list]), "help"),
          ((button_down, ['function2']),
           (print_help, [bindings_list]), "help"),
          ((jlow, ['leftStickHorz']),
           (set_j, [left_row_dictionary, 'left', 'left']), lambda: "left arm going left " + left_joints[0]),
          ((jhigh, ['leftStickHorz']),
           (set_j, [left_row_dictionary, 'left', 'right']), lambda: "left arm going right " + left_joints[0]),
          ((jlow, ['rightStickHorz']),
           (set_j, [right_row_dictionary, 'right', 'left']), lambda: "right arm going left " + right_joints[0]),
          ((jhigh, ['rightStickHorz']),
           (set_j, [right_row_dictionary, 'right', 'right']), lambda: "right arm going right " + right_joints[0]),
          ((jhigh, ['leftStickVert']),
           (set_j, [left_row_dictionary, 'left', 'back']), lambda: "left arm going back " + left_joints[0]),
          ((jlow, ['leftStickVert']),
           (set_j, [left_row_dictionary, 'left', 'fore']), lambda: "left arm going forward " + left_joints[0]),
          ((jhigh, ['rightStickVert']),
           (set_j, [right_row_dictionary, 'right', 'back']), lambda: "right arm going back " + right_joints[0]),
          ((jlow, ['rightStickVert']),
           (set_j, [right_row_dictionary, 'right', 'fore']), lambda: "right arm going forward " + right_joints[0]),
          ((jfor_right, ['leftStickHorz', 'leftStickVert']),
           (set_j, [left_row_dictionary, 'left', 'foreright']), lambda: "left arm forward right " + left_joints[0]),
          ((jfor_left, ['leftStickHorz', 'leftStickVert']),
           (set_j, [left_row_dictionary, 'left', 'foreleft']), lambda: "left arm forward left " + left_joints[0]),
          ((jback_right, ['leftStickHorz', 'leftStickVert']),
           (set_j, [left_row_dictionary, 'left', 'backright']), lambda: "left arm back right " + left_joints[0]),
          ((jback_left, ['leftStickHorz', 'leftStickVert']),
           (set_j, [left_row_dictionary, 'left', 'backleft']), lambda: "left arm back left " + left_joints[0]),
          ((jfor_right, ['rightStickHorz', 'rightStickVert']),
           (set_j, [right_row_dictionary, 'right', 'foreright']), lambda: "right arm forward right " + right_joints[0]),
          ((jfor_left, ['rightStickHorz', 'rightStickVert']),
           (set_j, [right_row_dictionary, 'right', 'foreleft']), lambda: "right arm forward left " + right_joints[0]),
          ((jback_right, ['rightStickHorz', 'rightStickVert']),
           (set_j, [right_row_dictionary, 'right', 'backright']), lambda: "right arm back right " + right_joints[0]),
          ((jback_left, ['rightStickHorz', 'rightStickVert']),
           (set_j, [right_row_dictionary, 'right', 'backleft']), lambda: "right back back left " + right_joints[0]),
          )

    bindings_list.append(bindings)

    rate = rospy.Rate(100)
    print_help(bindings_list)
    print("Press Ctrl-C to stop. ")
    while not rospy.is_shutdown():
        for (test, cmd, doc) in bindings:
            if test[0](*test[1]):
                cmd[0](*cmd[1])
                if callable(doc):
                    print(doc())
                else:
                    print(doc)
        if len(left_row_dictionary):
            if mode_left == 'continuous mode':

                print('not yet')
            if mode_left == 'direct mode' and left_count < 100:

                #print('kkkkkkk')
                #if left_count == 100:

                left_joint_buffer.insert(0,left_dest_matrix[left_count][0])
                left_joint_buffer.insert(1,left_dest_matrix[left_count][1])
                left_joint_buffer.insert(2,left_dest_matrix[left_count][2])
                left_joint_buffer.insert(3,left_dest_matrix[left_count][3])
                left_joint_buffer.insert(4,left_dest_matrix[left_count][4])
                left_joint_buffer.insert(5,left_dest_matrix[left_count][5])
                left_joint_buffer.insert(6,left_dest_matrix[left_count][6])
                if len(joint_buffer) > 7:
                    left_joint_buffer = left_joint_buffer[:7]

                """print('current is')
                print(left_joint_buffer)
                print('lololol')
                print(left_joint_buffer[0]/left_row_dictionary['left_s0'])
                print(left_joint_buffer[1]/left_row_dictionary['left_s1'])
                print(left_joint_buffer[2]/left_row_dictionary['left_e0'])
                print(left_joint_buffer[3]/left_row_dictionary['left_e1'])
                print(left_joint_buffer[4]/left_row_dictionary['left_w0'])
                print(left_joint_buffer[5]/left_row_dictionary['left_w1'])
                print(left_joint_buffer[6]/left_row_dictionary['left_w2'])
                print('should add')
                print(left_amount_buffer)
                print('left_dest_matrix is')
                print(left_dest_matrix)"""
                if left_count < 100:
                    left_row_destination['left_s0'] = left_dest_matrix[left_count][0]
                    left_row_destination['left_s1'] = left_dest_matrix[left_count][1]
                    left_row_destination['left_e0'] = left_dest_matrix[left_count][2]
                    left_row_destination['left_e1'] = left_dest_matrix[left_count][3]
                    left_row_destination['left_w0'] = left_dest_matrix[left_count][4]
                    left_row_destination['left_w1'] = left_dest_matrix[left_count][5]
                    left_row_destination['left_w2'] = left_dest_matrix[left_count][6]

                #print(left_count)
                #print(left_dest_matrix)
                t_end = time.time() + speed
                while time.time() < t_end and left_count < 100:
                    sleep(speed)                       #in direct mode, directly go to required positions
                left_count = left_count + 1


                #print(left_joint_buffer)
                joint_buffer = left_joint_buffer + right_joint_buffer
                tosend = "start"
                for index in range(14):
                    temp = str(joint_buffer[index])[:7]
                    tosend = tosend + temp
                print(right_joint_buffer)
                socket.send(tosend)
                message = socket.recv()
                #print(joint_buffer)
                #socket.send("start        ")
                #message = socket.recv()
                #for request in range(14):
                    #print("Sending request %s" % request)
                #    print(str(joint_buffer[request]))
                #    socket.send(str(joint_buffer[request]))
                    #  Get the reply.
                #    message = socket.recv()
                    #print("Received reply %s [ %s ]" % (request, message))
                #print(joint_buffer)
                #del joint_buffer[:]
        if len(right_row_dictionary):
            if mode_right == 'continuous mode':
                print('not yet')
            if mode_right == 'direct mode' and right_count < 100:
                #right.set_joint_positions(right_row_dictionary)
                right_joint_buffer.insert(0,right_dest_matrix[right_count][0])
                right_joint_buffer.insert(1,right_dest_matrix[right_count][1])
                right_joint_buffer.insert(2,right_dest_matrix[right_count][2])
                right_joint_buffer.insert(3,right_dest_matrix[right_count][3])
                right_joint_buffer.insert(4,right_dest_matrix[right_count][4])
                right_joint_buffer.insert(5,right_dest_matrix[right_count][5])
                right_joint_buffer.insert(6,right_dest_matrix[right_count][6])
                if len(joint_buffer) > 7:
                    right_joint_buffer = right_joint_buffer[:7]
                #print(right_joint_buffer)

                if right_count < 100:
                    right_row_destination['right_s0'] = right_dest_matrix[right_count][0]
                    right_row_destination['right_s1'] = right_dest_matrix[right_count][1]
                    right_row_destination['right_e0'] = right_dest_matrix[right_count][2]
                    right_row_destination['right_e1'] = right_dest_matrix[right_count][3]
                    right_row_destination['right_w0'] = right_dest_matrix[right_count][4]
                    right_row_destination['right_w1'] = right_dest_matrix[right_count][5]
                    right_row_destination['right_w2'] = right_dest_matrix[right_count][6]

                t_end = time.time() + speed
                while time.time() < t_end and right_count < 100:
                    sleep(speed)                      #in direct mode, directly go to required positions
                right_count = right_count + 1

                joint_buffer = left_joint_buffer + right_joint_buffer
                tosend = "start"
                for index in range(14):
                    temp = str(joint_buffer[index])[:7]
                    tosend = tosend + temp
                print(right_joint_buffer)
                socket.send(tosend)
                message = socket.recv()
                #print(joint_buffer)
                #socket.send("start        ")
                #message = socket.recv()
                #for request in range(14):
                #    print("Sending right request %s" % request)
                #    print(str(joint_buffer[request]))
                #    socket.send(str(joint_buffer[request]))
                    #  Get the reply.
                #    message = socket.recv()
                    #print("Received reply %s [ %s ]" % (request, message))
                #print(joint_buffer)
                #del joint_buffer[:]

        rate.sleep()
    return False

def main():
    """RSDK Joint Position Example: Joystick Control
    Use a game controller to control the angular joint positions
    of Baxter's arms.
    Attach a game controller to your dev machine and run this
    example along with the ROS joy_node to control the position
    of each joint in Baxter's arms using the joysticks. Be sure to
    provide your *joystick* type to setup appropriate key mappings.
    Each stick axis maps to a joint angle; which joints are currently
    controlled can be incremented by using the mapped increment buttons.
    Ex:
      (x,y -> e0,e1) >>increment>> (x,y -> e1,e2)
    """
    epilog = """
See help inside the example with the "Start" button for controller
key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-j', '--speed', required=True,
        choices=['fast', 'medium', 'slow'],
        help='specify the speed to use'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    joystick = XboxController()
    if args.speed == 'fast':
        speed = 0.05
    elif args.speed == 'medium':
        speed = 0.1
    elif args.speed == 'slow':
        speed = 0.15
    else:
        parser.error("Unsupported speed '%s'" % (args.speed))

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_joystick")

    map_joystick(joystick,speed)
    print("Done.")


if __name__ == '__main__':
    main()
