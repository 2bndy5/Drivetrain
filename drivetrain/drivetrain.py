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
MICROPY = False
try:
    from digitalio import DigitalInOut
except ImportError:
    import machine
    MICROPY = True
from .stepper import StepperMotor
from .motor import Solenoid, BiMotor, PhasedMotor
IS_THREADED = True
try:
    from threading import Thread
except ImportError:
    IS_THREADED = False


class Drivetrain:
    """A base class that is only used for inheriting various types of drivetrain configurations."""

    def __init__(self, motors, max_speed=100, smooth=True):
        #  prototype motors lust to avoid error in __del__ on exceptions
        self._motors = []
        for i, m in enumerate(motors):
            if not isinstance(m, (Solenoid, BiMotor, PhasedMotor, StepperMotor)):
                raise ValueError(
                    'unknown motor (index {}) of type {}'.format(i, type(m)))
        if not motors:
            raise ValueError('No motors were passed to the drivetrain.')
        self._motors = motors
        self._max_speed = max(0, min(max_speed, 100))
        self._prev_cmds = [0, 0]
        self._smooth = smooth

    @property
    def smooth(self):
        """This attribute enables (`True`) or disables (`False`) the input smoothing alogrithms for all motors (solenoids excluded) in the drivetrain."""
        return self._smooth

    @smooth.setter
    def smooth(self, enable):
        self._smooth = enable

    def sync(self):
        """This function should be used at least once per main loop iteration. It will trigger each
        motor's subsequent sync(), thus applying the smoothing input operations if needed. This is
        not needed if the smoothing algorithms are not utilized/necessary in the application"""
        for motor in self._motors:
            motor.sync()

    def stop(self):
        """This function will stop all motion in the drivetrain's motors"""
        commands = []
        for _ in self._motors:
            commands.append(0)
        self.go(commands)

    def go(self, cmds, smooth=None):
        """A helper function to the child classes to handle extra periphial motors attached to the
        `Drivetrain` object. This is only useful for motors that serve a specialized purpose
        other than propulsion.

        :param list,tuple cmds: A `list` or `tuple` of `int` motor input values to be passed in
            corresponding order to the motors. Each input value must be on range [-65535, 65535].

        :param bool smooth: This controls the motors' built-in algorithm that smooths input values
            over a period of time (in milliseconds) contained in the motors'
            :attr:`~drivetrain.motor.BiMotor.ramp_time` attribute. If this parameter is not
            specified, then the drivetrain's `smooth` attribute is used by default.
            This can be disabled per motor by setting the :attr:`~drivetrain.motor.BiMotor.ramp_time`
            attribute to ``0``, thus the smoothing algorithm is automatically bypassed despite this
            parameter's value.

            .. note:: Assert this parameter (set as `True`) for robots with a rather high center of
                gravity or if some parts are poorly attached. The absence of properly smoothed
                acceleration/deceleration will likely make the robot fall over or loose parts
                become dislodged on sudden and drastic changes in speed.
        """
        self._prev_cmds = cmds
        for i, cmd in enumerate(cmds):
            if i < len(self._motors):
                if (smooth if smooth is not None else self._smooth):
                    # if input is getting smoothed
                    self._motors[i].cellerate(cmd)
                else:
                    self._motors[i].value = cmd

    @property
    def value(self):
        return self._prev_cmds

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

    def __repr__(self):
        """Aggregates all the motors current values into a printable string."""
        output = ''
        for i, m in self._motors:
            output += 'motor {} value = {}'.format(i, m.value)
            if i != len(self._motors) - 1:
                output += ','

    def __del__(self):
        self._motors.clear()
        del self._motors


class Tank(Drivetrain):
    """A Drivetrain class meant to be used for motor configurations where propulsion and steering
    are shared tasks (also known as a "Differential" Drivetrain). For example: The military's tank vehicle essentially has 2 motors (1 on each side) where propulsion is done by both motors, and steering is controlled by varying the different motors' input commands.

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

    def go(self, cmds, smooth=None):
        """This function applies the user input to the motors' output according to drivetrain's
        motor configuration stated in the contructor documentation.

        :param list cmds: A `list` of input motor commands to be processed and
            passed to the motors. This list must have at least 2 items (input values), and any
            additional items will be ignored. A `list` of length less than 2 will throw a
            `ValueError` exception.

            .. important:: Ordering of the motor inputs contained in this list/tuple matters. They
                should correspond to the following order:

                1. left/right magnitude in range [-65535, 65535]
                2. forward/reverse magnitude in range [-65535, 65535]

        :param bool smooth: This controls the motors' built-in algorithm that smooths input values
            over a period of time (in milliseconds) contained in the motors'
            :attr:`~drivetrain.motor.BiMotor.ramp_time` attribute. If this parameter is not
            specified, then the drivetrain's `smooth` attribute is used by default.
            This can be disabled per motor by setting the :attr:`~drivetrain.motor.BiMotor.ramp_time`
            attribute to ``0``, thus the smoothing algorithm is automatically bypassed despite this
            parameter's value.

            .. note:: Assert this parameter (set as `True`) for robots with a rather high center of
                gravity or if some parts are poorly attached. The absence of properly smoothed
                acceleration/deceleration will likely make the robot fall over or loose parts
                become dislodged on sudden and drastic changes in speed.
        """
        if len(cmds) < 2:
            raise ValueError(
                "the list of commands must be at least 2 items long")
        cmds[0] = max(-65535, min(65535, int(cmds[0])))
        cmds[1] = max(-65535, min(65535, int(cmds[1])))
        # apply speed governor
        if abs(cmds[1]) > self._max_speed * 655.35:
            cmds[1] = self._max_speed * (655.35 if cmds[1] > 0 else -655.35)
        # assuming left/right axis is null (just going forward or backward)
        left = cmds[1]
        right = cmds[1]
        if not cmds[1]:
            # if forward/backward axis is null ("turning on a dime" functionality)
            # re-apply speed governor to only axis with a non-zero value
            if abs(cmds[0]) > self._max_speed * 655.35:
                cmds[0] = self._max_speed * \
                    (655.35 if cmds[0] > 0 else -655.35)
            right = cmds[0]
            left = cmds[0] * -1
        else:
            # if forward/backward axis is not null and left/right axis is not null
            # apply differential to motors accordingly
            offset = (65535 - abs(cmds[0])) / 65535.0
            if cmds[0] > 0:
                right *= offset
            elif cmds[0] < 0:
                left *= offset
        # send translated commands to motors
        super().go([left, right], smooth)


class Automotive(Drivetrain):
    """A Drivetrain class meant to be used for motor configurations where propulsion and steering
    are separate tasks. The first motor is used to steer, and the second motor is used to
    propell. An example of this would be any remote control toy vehicle.

    :param list motors: A `list` of motors that are to be controlled in concert. Each item in this
        `list` represents a single motor object and must be of type `Solenoid` (steering only), `BiMotor`,
        `PhasedMotor`, or `StepperMotor`. The 2 motors in this `list` are used to steer and propell
        respectively.

    :param int max_speed: The maximum speed as a percentage in range [0, 100] for the drivetrain's
        forward and backward motion. Defaults to 100%. This does not scale the motor speed's range,
        it just limits the top speed that the forward/backward motion can go.
    """

    def __init__(self, motors, max_speed=100):
        if len(motors) != 2:
            raise ValueError('The drivetrain requires 2 motors to operate.')
        if not isinstance(motors[0], (Solenoid, BiMotor, PhasedMotor, StepperMotor)):
            raise ValueError(type(motors[0]), 'unrecognized or unsupported motor object')
        if not isinstance(motors[1], (BiMotor, PhasedMotor, StepperMotor)):
            raise ValueError(type(motors[1]), 'unrecognized or unsupported motor object')
        super(Automotive, self).__init__(motors, max_speed)

    def go(self, cmds, smooth=None):
        """This function applies the user input to motor output according to drivetrain's motor
        configuration.

        :param list cmds: A `list` of input motor commands to be passed to the
            motors. This `list` must have at least 2 items (input values), and any
            additional item(s) will be ignored. A `list` of length less than 2 will throw a
            `ValueError` exception.

            .. important:: Ordering of the motor inputs contained in this list/tuple matters. They
                should correspond to the following order:

                1. left/right magnitude in range [-65535, 65535]
                2. forward/reverse magnitude in range [-65535, 65535]

        :param bool smooth: This controls the motors' built-in algorithm that smooths input values
            over a period of time (in milliseconds) contained in the motors'
            :attr:`~drivetrain.motor.BiMotor.ramp_time` attribute. If this parameter is not
            specified, then the drivetrain's `smooth` attribute is used by default.
            This can be disabled per motor by setting the :attr:`~drivetrain.motor.BiMotor.ramp_time`
            attribute to ``0``, thus the smoothing algorithm is automatically bypassed despite this
            parameter's value.

            .. note:: Assert this parameter (set as `True`) for robots with a rather high center of
                gravity or if some parts are poorly attached. The absence of properly smoothed
                acceleration/deceleration will likely make the robot fall over or loose parts
                become dislodged on sudden and drastic changes in speed.
        """
        if len(cmds) < 2:
            raise ValueError(
                "the list of commands must be at least 2 items long")
        cmds[0] = max(-65535, min(65535, int(cmds[0])))
        cmds[1] = max(-65535, min(65535, int(cmds[1])))
        # apply speed governor
        if abs(cmds[1]) > self._max_speed * 655.35:
            cmds[1] = self._max_speed * (655.35 if cmds[1] > 0 else -655.35)
        # send commands to motors
        super().go(cmds, smooth)


class Locomotive(Drivetrain):
    """This class relies soley on one `Solenoid` object controlling 2 solenoids in tandem. Like
    with a locomotive train, applied force is alternated between the 2 solenoids using a
    boolean-ized pressure sensor or switch to determine when the applied force is alternated.

    :param ~drivetrain.motor.Solenoid solenoids: This object has 1 or 2 solenoids attached. It
        will be used to apply the force for propulsion.

    :param ~microcontroller.Pin switch: This should be the (`board` module's)
        :py:class:`~microcontroller.Pin` that is connected to the sensor that will be used to
        determine when the force for propulsion should be alternated between solenoids.

    .. note:: There is no option to control the speed in this drivetrain class due to the nature of
        using solenoids for propulsion. Electronic solenoids apply either their full force or none
        at all. We currently are not supporting dynamic linear actuators (in which the force
        applied can vary) because they are basically motors simulating linear motion via a gear box
        controlling a shaft's extension/retraction. This may change when we support servos though.
    """

    def __init__(self, solenoids, switch_pin=None):
        if isinstance(solenoids, (BiMotor, PhasedMotor, StepperMotor)):
            raise ValueError('this drivetrain only uses a 1 Solenoid object')
        if switch_pin is None:
            raise ValueError('this drivetrain requires an input switch.')
        super(Locomotive, self).__init__([solenoids])
        self._switch_pin = switch_pin
        if MICROPY:
            self._switch_pin.init(machine.Pin.IN)
        else:
            self._switch_pin.switch_to_input()
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
            self.sync()

    def _move(self):
        do_while = True
        while do_while:
            self.sync()
            do_while = not self._cancel_thread

    def sync(self):
        """This function should be used at least once in the application's main loop. It will
        trigger the alternating of each solenoid's applied force. This IS needed on MCUs
        (microcontroller units) that can't use the threading module."""
        alternate = (1 if self._switch_pin.value else -1) * \
            (-1 if self._is_forward else 1)
        self._motors[0].go(alternate)

    @property
    def is_cellerating(self):
        """This attribute contains a `bool` indicating if the drivetrain's applied force via
        solenoids is in the midst of alternating. (read-only)"""
        if self._moving_thread is not None or self._is_in_motion:
            return True
        return False


class Mecanum(Drivetrain):
    """A Drivetrain class meant for motor configurations that involve 4 motors for propulsion
    and steering are shared tasks (like having 2 Tank Drivetrains). Each motor drives a single
    mecanum wheel which allows for the ability to strafe.

    :param list motors: A `list` of motors that are to be controlled in concert. Each item in this
        `list` represents a single motor object and must be of type `BiMotor`,
        `PhasedMotor`, or `StepperMotor`. The motors `list` should be ordered as follows:

            * Front-Right
            * Rear-Right
            * Rear-Left
            * Front-Left

    :param int max_speed: The maximum speed as a percentage in range [0, 100] for the drivetrain's
        forward and backward motion. Defaults to 100%. This does not scale the motor speed's range,
        it just limits the top speed that the forward/backward motion can go.
    """

    def __init__(self, motors, max_speed=100):
        if len(motors) != 4:
            raise ValueError('The drivetrain requires 4 motors to operate.')
        for i, m in enumerate(motors):
            if not isinstance(m, (BiMotor, PhasedMotor, StepperMotor)):
                raise ValueError(
                    'unknown motor (index {}) of type {}'.format(i, type(m)))
        super(Mecanum, self).__init__(motors, max_speed=max_speed)

    def go(self, cmds, smooth=None):
        """This function applies the user input to the motors' output according to drivetrain's
        motor configuration stated in the contructor documentation.

        :param list cmds: A `list` of input motor commands to be processed and
            passed to the motors. This list must have at least 2 items (input values), and any
            additional items will be ignored. A `list` of length less than 2 will throw a
            `ValueError` exception.

            .. important:: Ordering of the motor inputs contained in this list/tuple matters. They
                should correspond to the following order:

                1. left/right magnitude in range [-65535, 65535]
                2. forward/reverse magnitude in range [-65535, 65535]
                3. strafe boolean. `True` uses the left/right magnituse as strafing speed. `False`
                   uses the left/right magnitude for turning.

        :param bool smooth: This controls the motors' built-in algorithm that smooths input values
            over a period of time (in milliseconds) contained in the motors'
            :attr:`~drivetrain.motor.BiMotor.ramp_time` attribute. If this parameter is not
            specified, then the drivetrain's `smooth` attribute is used by default.
            This can be disabled per motor by setting the :attr:`~drivetrain.motor.BiMotor.ramp_time`
            attribute to ``0``, thus the smoothing algorithm is automatically bypassed despite this
            parameter's value.

            .. note:: Assert this parameter (set as `True`) for robots with a rather high center of
                gravity or if some parts are poorly attached. The absence of properly smoothed
                acceleration/deceleration will likely make the robot fall over or loose parts
                become dislodged on sudden and drastic changes in speed.
        """
        if len(cmds) < 3:
            raise ValueError(
                "the list of commands must be at least 3 items long")
        cmds[0] = max(-65535, min(65535, int(cmds[0])))
        cmds[1] = max(-65535, min(65535, int(cmds[1])))
        # apply speed governor
        if abs(cmds[1]) > self._max_speed * 655.35:
            cmds[1] = self._max_speed * (655.35 if cmds[1] > 0 else -655.35)
        # assuming left/right axis is null (just going forward or backward)
        left = cmds[1]
        right = cmds[1]
        if not cmds[1]:
            # if forward/backward axis is null
            # re-apply speed governor to only axis with a non-zero value
            if abs(cmds[0]) > self._max_speed * 655.35:
                cmds[0] = self._max_speed * \
                    (655.35 if cmds[0] > 0 else -655.35)
            right = cmds[0]
            left = cmds[0] * -1
            if not cmds[2]:
                # "turning on a dime" functionality
                super().go([left, left, right, right], smooth)
            else:
                # straight strafing functionality
                super().go([left * -1, left, right, right * -1], smooth)
        else:
            # if forward/backward axis is not null and left/right axis is not null
            # apply differential to motors accordingly
            if not cmds[2]:
                # veer and propell
                offset = (65535 - abs(cmds[0])) / 65535.0
                if cmds[1]:
                    if cmds[0] > 0:
                        right *= offset
                    elif cmds[0] < 0:
                        left *= offset
                super().go([left, left, right, right], smooth)
            else:
                # strafe and propell
                if 65535 > cmds[0] > 32767:
                    right *= (65535 - abs(cmds[0])) / -32767
                elif 0 < cmds[0] <= 32767:
                    right *= (32767 - abs(cmds[0])) / 32767
                elif 0 > cmds[0] >= -32767:
                    left *= (32767 - abs(cmds[0])) / 32767
                elif -32767 > cmds[0] > -65535:
                    left *= (65535 - abs(cmds[0])) / -32767
                if cmds[1] > 0:
                    super().go([left, right, left, right], smooth)
                else:
                    super().go([right, left, right, left], smooth)
