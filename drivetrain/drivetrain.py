# The MIT License (MIT)
#
# Copyright (c) 2019 Brendan Doherty
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""

Drivetrain Configuration Classes
================================

This module contains the necessary algorithms for utilizing different DC motor types in different
configurations for the raspberry pi. Currently only supporting the R2D2 (aliased here as `Tank`)
& typical openRC (aliased here as `Automotive`) configurations.

"""
# pylint: disable=arguments-differ,invalid-name
from digitalio import DigitalInOut
from .stepper import StepperMotor
from .motor import Solenoid, BiMotor, PhasedMotor, NRF24L01, USB
IS_THREADED = True
try:
    from threading import Thread
except ImportError:
    IS_THREADED = False

class Drivetrain:
    """A base class that is only used for inheriting various types of drivetrain configurations."""

    def __init__(self, motors, max_speed=100):
        for i, m in enumerate(motors):
            if not type(m, (Solenoid, BiMotor, PhasedMotor, StepperMotor)):
                raise ValueError(
                    'unknown motor (index {}) of type {}'.format(i, type(m)))
        if not motors:
            raise ValueError('No motors were passed to the drivetrain.')
        self._motors = motors
        self._max_speed = max(0, min(max_speed, 100))

    def tick(self):
        """This function should be used at least once per main loop iteration. It will trigger each
        motor's subsequent tick(), thus applying the smoothing input operations if needed. This is
        not needed if the smoothing algorithms are not utilized/necessary in the application"""
        for motor in self._motors:
            motor.tick()

    def _gogo(self, aux, init=2):
        """A helper function to the child classes to handle extra periphial motors attached to the
        `Drivetrain` object. This is only useful for motors that serve a specialized purpose
        other than propulsion.

        :param list,tuple aux: A `list` or `tuple` of `int` motor input values to be passed in
            corresponding order to the motors. Each input value must be on range [-65535, 65535].

        :param int init: The index of the `list`/`tuple` of motor commends from which to start
            passing values to the corresponding motors.

        """
        if len(aux) > init:
            for i in range(init, len(aux)):
                if i < len(self._motors):
                    self._motors[i].value = aux[i]
                else:
                    print(f'motor[{i}] not declared and/or installed')

    @property
    def is_cellerating(self):
        """This attribute contains a `bool` indicating if the drivetrain's motors' speed is in the
        midst of changing. (read-only)"""
        for m in self._motors:
            if m.is_cellerating:
                return True
        return False

    @property
    def max_speed(self):
        """This attribute determines a motor's top speed. Valid input values range [0, 100]."""
        return self._max_speed

    @max_speed.setter
    def max_speed(self, val):
        self._max_speed = max(0, min(val if val is not None else 0, 100))

    def print(self):
        """Prints all the motors current values."""
        for i, m in self._motors:
            print(f'motor {i} value = {m.value}')

    def __del__(self):
        self._motors.clear()
        del self._motors


class Tank(Drivetrain):
    """A Drivetrain class meant to be used for motor configurations where propulsion and steering
    are shared tasks. For example: R2D2 has 2 motors (1 in each leg -- the front retractable
    wheel is free pivoting) for propulsion and steering.

    :param list motors: A `list` of motors that are to be controlled in concert. Each item in this
        `list` represents a single motor object and must be of type `Solenoid`, `BiMotor`,
        `PhasedMotor`, or `StepperMotor`. The first 2 motors in this `list` are used to propell and
        steer respectively.

    :param int max_speed: The maximum speed as a percentage in range [0, 100] for the drivetrain's
        forward and backward motion. Defaults to 100%. This does not scale the motor speed's range,
        it just limits the top speed that the forward/backward motion can go.

    """

    def __init__(self, motors, max_speed=100):
        if len(motors) != 2:
            raise ValueError('The drivetrain requires 2 motors to operate.')
        super(Tank, self).__init__(motors, max_speed)

    def go(self, cmds, smooth=True):
        """This function applies the user input to the motors' output according to drivetrain's motor
        configuration stated in the contructor documentation.

        :param list,tuple cmds: A `list` or `tuple` of input motor commands to be processed and
            passed to the motors. This list must have at least 2 items (input values), and any
            additional items will be ignored. A `list`/`tuple` of length less than 2 will throw a
            `ValueError` exception.

            .. important:: Ordering of the motor inputs contained in this list/tuple matters. They
                should correspond to the following order:

                1. forward/reverse magnitude in range [-65535, 65535]
                2. left/right magnitude in range [-65535, 65535]

        :param bool smooth: This controls the motors' built-in algorithm that smooths input values
            over a period of time (in milliseconds) contained in the motors'
            :attr:`~drivetrain.motor.BiMotor.ramp_time` attribute. This defaults to `True`.
            Optionally, if the :attr:`~drivetrain.motor.BiMotor.ramp_time` attribute is set to
            ``0`` then, the smoothing algorithm is automatically bypassed despite this parameter's
            value.

            .. note:: Assert this parameter (set as `True`) for robots with a rather high center of
                gravity or if some parts are poorly attached. The absence of properly smoothed
                acceleration/deceleration will likely make the robot fall over or loose parts
                become dislodged on sudden and drastic changes in speed.

        """
        if len(cmds) < 2:
            raise ValueError("the list/tuple of commands must be at least 2 items long")
        cmds[0] = max(-65535, min(65535, int(cmds[0])))
        cmds[1] = max(-65535, min(65535, int(cmds[1])))
        if cmds[1] > self._max_speed:
            cmds[1] = self._max_speed
        # assuming left/right axis is null (just going forward or backward)
        left = cmds[1]
        right = cmds[1]
        if abs(cmds[0]) == 65535:
            # if forward/backward axis is null ("turning on a dime" functionality)
            if cmds[0] > self._max_speed:
                cmds[0] = self._max_speed
            right = cmds[0]
            left = cmds[0] * -1
        else:
            # if forward/backward axis is not null and left/right axis is not null
            offset = (65535 - abs(cmds[0])) / 65535.0
            if cmds[0] > 0:
                right *= offset
            elif cmds[0] < 0:
                left *= offset
        if smooth:
            self._motors[0].cellerate(left * 655.35)
            self._motors[1].cellerate(right * 655.35)
        else:
            self._motors[0].value = left * 655.35
            self._motors[1].value = right * 655.35
        self._gogo(cmds)


class Automotive(Drivetrain):
    """A Drivetrain class meant to be used for motor configurations where propulsion and steering
    are separate tasks. The first motor is used to steer, and the second motor is used to
    propell.

    :param list motors: A `list` of motors that are to be controlled in concert. Each item in this
        `list` represents a single motor object and must be of type `Solenoid`, `BiMotor`,
        `PhasedMotor`, or `StepperMotor`. The first 2 motors in this `list` are used to propell and
        steer respectively.

    :param int max_speed: The maximum speed as a percentage in range [0, 100] for the drivetrain's
        forward and backward motion. Defaults to 100%. This does not scale the motor speed's range,
        it just limits the top speed that the forward/backward motion can go.

    """

    def __init__(self, motors, max_speed=100):
        if len(motors) != 2:
            raise ValueError('The drivetrain requires 2 motors to operate.')
        super(Automotive, self).__init__(motors, max_speed)

    def go(self, cmds, smooth=True):
        """This function applies the user input to motor output according to drivetrain's motor
        configuration.

        :param list,tuple cmds: A `list` or `tuple` of input motor commands to be passed to the
            motors. This `list`/`tuple` must have at least 2 items (input values), and any
            additional item(s) will be ignored. A `list`/`tuple` of length less than 2 will throw a
            `ValueError` exception.

            .. important:: Ordering of the motor inputs contained in this list/tuple matters. They
                should correspond to the following order:

                1. forward/reverse magnitude in range [-65535, 65535]
                2. left/right magnitude in range [-65535, 65535]

        :param bool smooth: This controls the motors' built-in algorithm that smooths input values
            over a period of time (in milliseconds) contained in the motors'
            :attr:`~drivetrain.motor.BiMotor.ramp_time` attribute. This defaults to `True`.
            Optionally, if the :attr:`~drivetrain.motor.BiMotor.ramp_time` attribute is set to
            ``0`` then, the smoothing algorithm is automatically bypassed despite this parameter's
            value.

            .. note:: Assert this parameter (set as `True`) for robots with a rather high center of
                gravity or if some parts are poorly attached. The absence of properly smoothed
                acceleration/deceleration will likely make the robot fall over or loose parts
                become dislodged on sudden and drastic changes in speed.

        """
        if len(cmds) < 2:
            raise ValueError(
                "the list/tuple of commands must be at least 2 items long")
        # make sure speeds are an integer (not decimal/float)
        cmds[0] = max(-65535, min(65535, int(cmds[0])))
        cmds[1] = max(-65535, min(65535, int(cmds[1])))
        if cmds[1] > self._max_speed:
            cmds[1] = self._max_speed
        if smooth:
            self._motors[0].cellerate(cmds[0])
            self._motors[1].cellerate(cmds[1])
        else:
            self._motors[0].value = cmds[0]
            self._motors[1].value = cmds[1]
        self._gogo(cmds)


class External:
    """A class to be used for controlling drivetrains not physically attached to the host
    controller. Currently only supports USB (Serial) and nRF24L01 (an spi based radio transceiver)
    as interfaces. Other interface options are being considered, like the RFM69 radio, and a
    customized Arduino/CircuitPython device to acts as a slave I2C/SPI device.

    :param ~motor.USB,~motor.NRF24L01 interface: The specialized interface type used to remotely
        control an external drivetrain.

    .. warning:: This class is HIGHLY EXPERIMENTAL, and needs battlehardening. At this stage of
        development, our only working solution involves separate libraries (written in python or
        Arduino flavored C++) that acts as a counterpart to the interface specified here, but
        nothing has been finalized yet.

    """

    def __init__(self, interface):
        if type(interface, (USB, NRF24L01)):
            self._interface = interface
        else:
            raise ValueError('The "External" drivetrain class only supports interfaces of type'
                             ' class USB or NRF24L01')
        self._last_cmds = []

    def go(self, cmds):
        """This function simply passes motor control commands to the specified ``interface``
        (passed to the construstor).

        .. important:: The receiving interface will be running code not found in this library. It
            is up to you write that code. We are currently still testing this feature with another
            library meant to act as a counterpart. Links and docs will be provided when stable
            enough for pre-release; please be patient and `stay tuned to this issue.
            <https://github.com/DVC-Viking-Robotics/Drivetrain/issues/3>`_

        """
        self._interface.go(cmds)


class Locomotive(Drivetrain):
    """This class relies soley on one `Solenoid` object controlling 2 solenoids in tandem. Like
    with a locomotive train, applied force is alternated between the 2 solenoids using a
    boolean-ized pressure sensor or switch to determine when the applied force is alternated.

    :param ~derivetrain.motor.Solenoid solenoids: This object has 1 or 2 solenoids attached. It will be
        used to apply the force for propulsion.

    :param ~microcontroller.Pin switch: This should be the (`board` module's) pin that is connected
        to the sensor that will be used to determine when the force for propulsion should be
        alternated between solenoids.

    .. note:: There is no option to control the speed in this drivetrain class due to the nature of
        using solenoids for propulsion. Electronic solenoids apply either their full force or none
        at all. We currently are not supporting dynamic linear actuators (in which the force
        applied can vary) because they are basically motors simulating linear motion via a gear box
        controlling a shaft's extension/retraction. This may change when we support servos though.

    """

    def __init__(self, solenoids, switch):
        super(Locomotive, self).__init__([solenoids])
        self._switch = DigitalInOut(switch)
        self._switch.switch_to_input()
        self._is_forward = True
        self._is_in_motion = False
        self._cancel_thread = not IS_THREADED
        self._moving_thread = None

    def stop(self):
        """This function stops the process of alternating applied force between the solenoids."""
        if IS_THREADED:
            self._stop_thread()
        self._is_in_motion = False

    def _stop_thread(self):
        if self._moving_thread is not None:
            self._cancel_thread = True
            self._moving_thread.join()
            self._cancel_thread = False
        self._moving_thread = None

    def go(self, forward):
        """This function starts the process of alternating applied force between the solenoids
        with respect to the specified direction.

        :param bool forward: `True` cylces the forces in a way that invokes a forward motion.
            `False` does the same but invokes a force in the backward direction.

        .. note:: Since we are talking about applying linear force to a wheel or axle, the
            direction is entirely dependent on the physical orientation of the solenoids. In
            other words, the armature of one solenoid should be attached to the wheel(s) or
            axle(s) in a position that is always opposite the position of the other solenoid's
            armature on the same wheel(s) or axel(s).

        """
        self._is_forward = forward
        self._is_in_motion = True
        self.stop()
        if IS_THREADED:
            self._moving_thread = Thread(target=self._move)
            self._moving_thread.start()
        else:
            self.tick()

    def _move(self):
        do_while = True
        while do_while:
            self.tick()
            do_while = not self._cancel_thread

    def tick(self):
        """This function should be used at least once per main loop iteration. It will triggerthe alternating of each solenoid's applied force. This IS needed on MCUs (microcontroller units) that can't use the threading module."""
        alternate = (1 if self._switch.value else -1) * \
            (-1 if self._is_forward else 1)
        self._motors[0].go(alternate)

    @property
    def is_cellerating(self):
        """This attribute contains a `bool` indicating if the drivetrain's applied force via
        solenoids is in the midst of alternating. (read-only)"""
        if self._moving_thread is not None or self._is_in_motion:
            return True
        return False
