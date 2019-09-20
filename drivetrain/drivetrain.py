"""

Drivetrain
===========

This module contains the necessary algorithms for utilizing different DC motor types in different configurations for the raspberry pi. Currently only supporting the R2D2 (AKA BiPed) & typical openRC (AKA QuadPed) configurations.
"""
# pylint: disable=arguments-differ,invalid-name

# from gpiozero import AngularServo
from .gpio_zero_stepper_motor import Stepper
from .motor import BiMotor, PhasedMotor, NRF24L01, USB


class Drivetrain:
    """A base class that is only used for inheriting various types of drivetrain configurations."""
    def __init__(self, motors, maxSpeed):
        for i, m in enumerate(motors):
            if not type(m, (BiMotor, PhasedMotor, Stepper)):
                raise ValueError('unknown motor (index {}) of type {}'.format(i, type(m)))
        self._motors = motors
        self.max_speed = max(0, min(maxSpeed, 100))  # ensure proper range

    def _gogo(self, aux, init=2):
        """A helper function to the child classes to handle extra periphial motors attached to the `Drivetrain` object. This is only useful for motors that serve a specialized purpose other than propulsion.

        :param list,tuple aux: A list or tuple of `int` motor input values to be passed in corresponding order to the motors.
        :param int init: The index of the `list`/`tuple` of motor commends from which to start passing values to the corresponding motors.

        """
        if len(aux) > init:
            for i in range(init, len(aux)):
                if i < len(self._motors):
                    # print('motor[', i, '].value = ', aux[i] / 100.0, sep = '')
                    self._motors[i].value = aux[i] / 100.0
                else:
                    print('motor[', i, '] not declared and/or installed', sep='')

    def _print(self, start=0):
        """Prints the value of each motor.

        :param int start: The index in the list of motors to start printing from until end of list.

        """
        for i in range(start, len(self._motors)):
            print('motor[', i, '].value = ', self._motors[i].value, sep='')

    def __del__(self):
        self._motors.clear()
        del self._motors
# end Drivetrain class


class BiPed(Drivetrain):
    """A Drivetrain class meant to be used for motor configurations where propulsion and steering are shared tasks. For example: R2D2 has 2 motors (1 in each leg -- the front retractable wheel is free pivoting) for propulsion and steering.

    :param list,tuple pins: A `list` of `tuple` of pins that are connected to motor drivers. Each item (`tuple`) in this `list` represents a single motor and contains all the pins specific to that motor.

        .. note:: the motor type is currently being detirmined on the number of pins used to control that motor. The supported types are as follows:
            * a `tuple` of 2 pins instantiates a bi-directional dc motor
            * a `tuple` of 1 pin instantiates a servo motor
            * a `tuple` of 4 pins instantiates a stepper motor

        .. important:: the 1st 2 items (`tuple`) in the `list` are used to propell and steer respectively.

    :param list phased: a `list` of `bool` values to indicate the corresponding DC motor driver's input configuration. If the motor driver used to drive the 1st DC motor in the ``pins`` list is using 1 PWM signal (to control motor speed) and 1 digital output signal (to control motor direction), then the 1st item in this `list` is `True`. If the 1st DC motor driver used to drive the motor expects 2 PWM signals (1 for each motor direction), then the first item is `False`. For example: a value of ``[False,False]`` means, the 1st 2 DC motor pins specified in the ``pins`` parameter will be treated as all PWM outputs.
    :param int maxSpeed: Sets the maximum speed (as a percentage) for the drivetrain's forward and backward motion. Defaults to 85%. This does not scale the motor's speed rangge, it just limits the top speed that the forward/backward motion can go.

    """
    def __init__(self, pins, phased=None, maxSpeed=85):
        super(BiPed, self).__init__(pins, phased, maxSpeed)
        self._right = 0
        self._left = 0

    def go(self, cmds, smooth=True):
        """This function applies the user input to motor output according to drivetrain's motor configuration.

        :param list,tuple cmds: A `list` or `tuple` of input motor commands to be processed and passed to the motors. This list must have at least 2 items (input values), but can have as many as nessecary

            .. important:: Ordering of the motor inputs contained in this list/tuple matters. They should correspond to the following order:

            1. forward/reverse magnitude in range [-100,100]
            2. left/right magnitude in range [-100,100]
            3. extra periphial motor's magnitude/percentage in range [-100,100]

            ..note:: there can be more input values than there are motors. Only the input values that correlate to a motor are used though.

        :param bool smooth: This controls a built-in algorithm that smooths the motor input values over the period of time (in seconds) specified by the ``ramptime`` parameter in the constructor. This defaults to `True`. Use this for robots with a rather high center of gravity (top-heavy) as the absence of properly smoothed acceleration/deceleration will likely make the robot fall over on sudden and drastic changes in speed.

        """
        # make sure speeds are an integer (not decimal/float) and send to motors
        # make sure arguments are in their proper range
        cmds[0] = round(max(-100, min(100, cmds[0])))
        cmds[1] = round(max(-100, min(100, cmds[1])) * (self.max_speed / 100.0))
        # assuming left/right axis is null (just going forward or backward)
        self._left = cmds[1]
        self._right = cmds[1]
        if abs(cmds[0]) == 100:
            # if forward/backward axis is null ("turning on a dime" functionality)
            cmds[0] *= self.max_speed / 100.0
            self._right = cmds[0]
            self._left = cmds[0] * -1
        else:
            # if forward/backward axis is not null and left/right axis is not null
            offset = (100 - abs(cmds[0])) / 100.0
            if cmds[0] > 0:
                self._right *= offset
            elif cmds[0] < 0:
                self._left *= offset
        if smooth:
            self._motors[0].cellerate(self._left / 100.0)
            self._motors[1].cellerate(self._right / 100.0)
        else:
            self._motors[0].value = self._left / 100.0
            self._motors[1].value = self._right / 100.0
        self._gogo(cmds)

    def print(self):
        """Prints all the motors current values."""
        print("left =", self._left)
        print("right =", self._right)
        super(BiPed, self)._print(2)
# end BiPed class


class QuadPed(Drivetrain):
    """A Drivetrain class meant to be used for motor configurations where propulsion and steering are shared tasks. For example: R2D2 has 2 motors (1 in each leg -- the front retractable wheel is free pivoting) for propulsion and steering.

    :param list,tuple pins: A `list` of `tuple` of pins that are connected to motor drivers. Each item (`tuple`) in this `list` represents a single motor and contains all the pins specific to that motor.

        .. note:: the motor type is currently being detirmined on the number of pins used to control that motor. The supported types are as follows:
            * a `tuple` of 2 pins instantiates a bi-directional dc motor
            * a `tuple` of 1 pin instantiates a servo motor
            * a `tuple` of 4 pins instantiates a stepper motor

        .. important:: the 1st 2 items (`tuple`) in the `list` are used to propell and steer respectively.

    :param list phased: a `list` of `bool` values to indicate the corresponding DC motor driver's input configuration. If the motor driver used to drive the 1st DC motor in the ``pins`` list is using 1 PWM signal (to control motor speed) and 1 digital output signal (to control motor direction), then the 1st item in this `list` is `True`. If the 1st DC motor driver used to drive the motor expects 2 PWM signals (1 for each motor direction), then the first item is `False`. For example: a value of ``[False,False]`` means, the 1st 2 DC motor pins specified in the ``pins`` parameter will be treated as all PWM outputs.
    :param int maxSpeed: Sets the maximum speed (as a percentage) for the drivetrain's forward and backward motion. Defaults to 85%. This does not scale the motor's speed rangge, it just limits the top speed that the forward/backward motion can go.

    """
    def __init__(self, pins, phased=None, maxSpeed=85):
        super(QuadPed, self).__init__(pins, phased, maxSpeed)
        self._forward_reverse = 0  # forward/reverse direction
        self._left_right = 0  # left/right direction

    def go(self, cmds, smooth=True):
        """This function applies the user input to motor output according to drivetrain's motor configuration.

        :param list,tuple cmds: A `list` or `tuple` of input motor commands to be processed and passedto the motors. This list must have at least 2 items (input values), but can have as many as nessecary

            .. important:: Ordering of the motor inputs contained in this list/tuple matters. They should correspond to the following order:

            1. forward/reverse magnitude in range [-100,100]
            2. left/right magnitude in range [-100,100]
            3. extra periphial motor's magnitude/percentage in range [-100,100]

            ..note:: there can be more input values than there are motors. Only the input values that correlate to a motor are used though.

        :param bool smooth: This controls a built-in algorithm that smooths the motor input values over the period of time (in seconds) specified by the ``ramptime`` parameter in the constructor. This defaults to `True`. Use this for robots with a rather high center of gravity (top-heavy) as the absence of properly smoothed acceleration/deceleration will likely make the robot fall over on sudden and drastic changes in speed.

        """
        # make sure arguments are in their proper range
        # make sure speeds are an integer (not decimal/float)
        cmds[0] = round(max(-100, min(100, cmds[0])))
        cmds[1] = round(max(-100, min(100, cmds[1])) * (self.max_speed / 100.0))
        # set the axis directly to their corresponding motors
        self._left_right = cmds[0]
        self._forward_reverse = cmds[1]
        if smooth:
            self._motors[0].cellerate(self._left_right / 100.0)
            self._motors[1].cellerate(self._forward_reverse / 100.0)
        else:
            self._motors[0].value = self._left_right / 100.0
            self._motors[1].value = self._forward_reverse / 100.0
        self._gogo(cmds)

    # for debugging purposes
    def print(self):
        """Prints all the motors current values."""
        print("forward/reverse =", self._forward_reverse)
        print("left/right =", self._left_right)
        super(QuadPed, self)._print(2)
# end QuadPed class

class External:
    """A class to be used for controlling drivetrains not physically attached to the host controller. Currently only supports USB (Serial) and nRF24L01 (an spi based radio) as interfaces"""
    def __init__(self, interface):
        if type(interface, (USB, NRF24L01)):
            self._interface = interface
        else:
            raise ValueError('The "External" drivetrain class only supports interfaces of type class USB or NRF24L01')

    def go(self, cmds):
        """pass motor control commands to the appropriate interface (specified upon instantiation)."""
        self._interface.go(cmds)
