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
A collection of driver classes for different single phase DC motors implementing the threading
module.
"""
try:
    from digitalio import DigitalInOut
except ImportError: # running on a MicroPython board
    from .helpers.digi_io import DigitalInOut
from circuitpython_nrf24l01 import RF24
from .helpers.smoothing_input import SmoothMotor
# pylint: disable=too-few-public-methods,invalid-name

class MotorPool(list):
    """A grouping object for various different motors. This object coordinates the motor input
    commands with the motor objects, but optimizes the number of times the communication buses are
    used for the entire motor pool."""
    def __init__(self, motors):
        for i, m in enumerate(motors):
            if not isinstance(m, (SmoothMotor, Solenoid)):
                raise ValueError("unrecognized or unsupported motor object type"
                                 " {} in list at index {}".format(type(m), i))
        self._motors = motors

    def go(self, cmds, smooth=True):
        """This function takes the input commands and passes them to the motors.

        :param list,tuple cmds: A `list` or `tuple` of values to be passed to the motors.
        :param bool smooth: This controls the motors' built-in algorithm that smooths input values
            over a period of time (in milliseconds) contained in the motors'
            :attr:`~drivetrain.helpers.smoothing_input.SmoothMotor.ramp_time` attribute. This can be
            disabled per motor by setting the
            :attr:`~drivetrain.helpers.smoothing_input.SmoothMotor.ramp_time` attribute to ``0``, thus
            the smoothing algorithm is automatically bypassed despite this parameter's value.
        """
        # for i, cmd in enumerate(cmds):
        #     if i < len(self._motors):
        #         if smooth: # if input is getting smoothed
        #             self._motors[i].cellerate(cmd)
        #         else:
        #             self._motors[i].value = cmd

        i = 0
        while i < len(self._motors):
            skip = len(self._motors[i])
            for j in range(skip):
                if smooth:
                    self._motors[i + j].cellerate(cmds[i + j])
                else:
                    self._motors[i + j].value = cmds[i + j]
            i += skip

    @property
    def is_cellerating(self):
        for m in self._motors:
            if m.is_cellerating:
                return True
        return False

    def sync(self):
        """This function will trigger each motor's asynchronous code to execute (mainly used for
        smoothing inputs). See also :py:meth:`~drivetrain.helpers.smoothing_input.SmoothMotor.sync()`
        """
        for m in self._motors:
            m.sync()

    def __getitem__(self, key):
        return self._motors[key]

    def __len__(self):
        return len(self._motors)

    def __iter__(self):
        self._it = 0
        return self

    def __next__(self):
        if self._it < len(self._motors) - 1:
            self._it += 1
            return self._motors[self._it]
        del self._it
        raise StopIteration

class Solenoid:
    """This class is meant be used to control up to 2 solenoids (see also `value` attribute
    for more details) as in the case of an actual locomotive train. Solenoids, by nature, cannot
    be controlled dynamically (cannot be any value other than `True` or `False`).

    :param list pins: A `list` of (`board` module's) :py:class:`~microcontoller.Pin` numbers that
        are used to drive the solenoid(s). The length of this `list` must be in range [1, 2] (any
        additional items/pins will be ignored).
    """
    def __init__(self, plus_pin=None, neg_pin=None):
        len_pins = bool(plus_pin is not None) + bool(neg_pin is not None)
        if not len_pins:
            raise ValueError('The number of pins used must be at least 1.')
        self._signals = []
        for i in range(len_pins):
            if plus_pin is not None:
                self._signals.append(plus_pin)
            elif neg_pin is not None:
                self._signals.append(neg_pin)
            self._signals[i].switch_to_output(False)

    @property
    def value(self):
        """This attribute contains the current output value of the solenoid(s) in range
        [-1, 1]. An invalid input value will be clamped to an `int` in the proper range.

        .. note:: Because this class is built to handle 2 pins (passed in the ``pins`` parameter
            to the constructor) and tailored for solenoids, any negative value will only energize
            the solenoid driven by the second pin . Any positive value will only energize the
            solenoid driven by the first pin. Alternatively, a ``0`` value will de-energize both
            solenoids.
        """
        return self._signals[0].value - self._signals[1].value if \
            len(self._signals) > 1 else self._signals[0].value

    @value.setter
    def value(self, val):
        val = max(-1, min(1, int(val)))
        if val: # going forward == val is positive; going backward == val is negative
            self._signals[0].value = True if val > 0 else False
            if len(self._signals) > 1:
                self._signals[1].value = not (True if val > 0 else False)
        else: # otherwise stop
            self._signals[0] = False
            if len(self._signals) > 1:
                self._signals[1] = False

    def __del__(self):
        if self.value:
            self.value = 0
        for signal in self._signals:
            signal.deinit()
        self._signals.clear()
# end Solenoid parent class

class BiMotor(SmoothMotor):
    """This class is meant be used for motors driven by driver boards/ICs that expect 2 PWM outputs
    . Each pin represent the controlling signal for the motor's speed in a single rotational
    direction.

    :param list pins: A `list` of (`board` module's) :py:class:`~microcontoller.Pin` numbers that
        are used to drive the motor. The length of this `list` or `tuple` must be in range [1, 2];
        any additional items/pins will be ignored, and a `ValueError` exception is thrown if no
        pins are passed (an empty `tuple`/`list`). If only 1 pin is passed, then the motor will
        only rotate in 1 direction depending on how the motor is connected to the motor driver.

    :param int ramp_time: The time (in milliseconds) that is used to smooth the motor's input.
        Default is 500. This time represents the maximum amount of time that the input will be
        smoothed. Since the change in speed is also used to determine how much time will be used
        to smooth the input, this parameter's value will represent the time it takes for the motor
        to go from full reverse to full forward and vice versa. If the motor is going from rest to
        either full reverse or full forward, then the time it takes to do that will be half of
        this parameter's value. This can be changed at any time by changing the `ramp_time`
        attribute.
    """
    def __init__(self, plus_pin=None, neg_pin=None, ramp_time=500):
        len_pins = bool(plus_pin is not None) + bool(neg_pin is not None)
        if not len_pins:
            raise ValueError('The number of pins used must be at least 1.')
        self._signals = []
        for _ in range(len_pins):
            if plus_pin is not None:
                self._signals.append(plus_pin)
            elif neg_pin is not None:
                self._signals.append(neg_pin)
        super(BiMotor, self).__init__(ramp_time)

    @property
    def value(self):
        """This attribute contains the current output value of the solenoid(s) in range
        [-65535, 65535]. An invalid input value will be clamped to an `int` in the proper range.
        A negative value represents the motor's speed in reverse rotation. A positive value
        reprsents the motor's speed in forward rotation."""
        self._value = self._signals[0].duty_cycle - \
            (self._signals[1].duty_cycle if len(self._signals) > 1 else 0)
        return self._value

    @value.setter
    def value(self, val):
        val = max(-65535, min(65535, int(val)))
        if val: # going forward == val is positive; going backward == val is negative
            self._signals[0].duty_cycle = val if val > 0 else 0
            if len(self._signals) > 1:
                self._signals[1].duty_cycle = val * (-1 if val < 0 else 0)
        else: # otherwise stop
            self._signals[0].duty_cycle = 0
            if len(self._signals) > 1:
                self._signals[1].duty_cycle = 0

    def __del__(self):
        super().__del__()
        for signal in self._signals:
            signal.deinit()
        self._signals.clear()
# end BiMotor child class


class PhasedMotor(SmoothMotor):
    """This class is meant be used for motors driven by driver boards/ICs that expect:

        * 1 PWM output (to control the motor's speed)
        * 1 digital output (to control the motor's rotational direction)

    :param list pins: A `list` of (`board` module's) :py:class:`~microcontoller.Pin` numbers that
        are used to drive the motor. The length of this `list`/`tuple` must be 2, otherwise a
        `ValueError` exception is thrown.

        .. note:: The first pin in the `tuple`/`list` is used for the digital output signal
            that signifies the motor's rotational direction. The second pin is used for PWM output
            that signifies the motor's speed.

    :param int ramp_time: The time (in milliseconds) that is used to smooth the motor's input.
        Default is 500. This time represents the maximum amount of time that the input will be
        smoothed. Since the change in speed is also used to determine how much time will be used
        to smooth the input, this parameter's value will represent the time it takes for the motor
        to go from full reverse to full forward and vice versa. If the motor is going from rest to
        either full reverse or full forward, then the time it takes to do that will be half of
        this parameter's value. This can be changed at any time by changing the `ramp_time`
        attribute.
    """
    def __init__(self, phased_pin=None, pwm_pin=None, ramp_time=500):
        len_pins = bool(phased_pin is not None) + bool(pwm_pin is not None)
        if len_pins != 2:
            raise ValueError('The number of pins used must be 2.')
        self._signals = []
        for _ in range(len_pins):
            if phased_pin is not None:
                self._signals.insert(0, phased_pin)
                self._signals[0].switch_to_output(False)
            elif pwm_pin is not None:
                self._signals.insert(1, pwm_pin)
        super(PhasedMotor, self).__init__(ramp_time)

    @property
    def value(self):
        """This attribute contains the current output value of the solenoid(s) in range
        [-65535, 65535]. An invalid input value will be clamped to an `int` in the proper range.
        A negative value represents the motor's speed in reverse rotation. A positive value
        reprsents the motor's speed in forward rotation."""
        self._value = self._signals[1].duty_cycle * (1 if self._signals[0].value else -1)
        return self._value

    @value.setter
    def value(self, val):
        val = max(-65535, min(65535, int(val)))
        if val: # going forward == val is positive; going backward == val is negative
            self._signals[0].value = val > 0
            self._signals[1].duty_cycle = val * (-1 if val < 0 else 1)
        # otherwise stop
        else:
            self._signals[0].value = False
            self._signals[1].duty_cycle = 0

    def __del__(self):
        super().__del__()
        for signal in self._signals:
            signal.deinit()
        self._signals.clear()
# end PhasedMotor child class
