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
================================
Solenoid, BiMotor, & PhasedMotor
================================

A collection of driver classes for different single phase DC motors implementing the threading
module. Includes Solenoid (parent base class), BiMotor & PhasedMotor (children of Solenoid)

"""
from math import pi as PI, cos
import time
MICROPY = False
try:
    from digitalio import DigitalInOut
except ImportError: # running on a MicroPython board
    import machine
    MICROPY = True

IS_THREADED = True
try:
    from threading import Thread
except ImportError:
    IS_THREADED = False

from circuitpython_nrf24l01 import RF24

try:
    from pulseio import PWMOut
except ImportError: # running on Raspberry Pi, nVidia Jetson, or a MicroPython board
    from .pwm import PWMOut

# pylint: disable=too-many-instance-attributes,too-few-public-methods,invalid-name

class Solenoid:
    """This base class is meant be used as a parent to `BiMotor` and `PhasedMotor` classes of this
    module, but can be used for solenoids if needed. Solenoids, by nature, cannot be controlled
    dynamically (cannot be any value other than `True` or `False`). Despite the fact that this class
    holds all the smoothing input algorithms for its child classes, the output values, when
    instantiated objects with this base class, are not actually smoothed. With that said, this
    class can be used to control up to 2 solenoids (see also `value` attribute for more details) as
    in the case of an actual locomotive train.

    :param list pins: A `list` of (`board` module's) :py:class:`~microcontoller.Pin` numbers that
        are used to drive the solenoid(s). The length of this `list` must be in range [1, 2] (any
        additional items/pins will be ignored).

    :param int ramp_time: This parameter is really a placeholder for the child classes
        `BiMotor` & `PhasedMotor` as it has no affect on objects instantiated with this base class.
        Changing this value has not been tested and will probably slightly delay the
        solenoid(s) outputs.
    """
    def __init__(self, pins, ramp_time=0):
        if not pins:
            raise ValueError('The number of pins used must be at least 1')
        self._signals = []
        for i, pin in enumerate(pins):
            if i >= 2:
                break
            if MICROPY:
                self._signals.append(machine.Pin(pin, machine.Pin.OUT))
            else:
                self._signals.append(DigitalInOut(pin))
                self._signals[i].switch_to_output(False)
        # variables used to track acceleration
        self._init_speed = 0
        self._target_speed = 0
        self._init_smooth = 0
        self._end_smooth = 0
        self._dt = ramp_time
        self._smoothing_thread = None
        self._cancel_thread = not IS_THREADED

    @property
    def ramp_time(self):
        """This attribute is the maximum amount of time (in milliseconds) used to smooth the input
        values. A negative value will be used as a positive number. Set this to ``0`` to
        disable all smoothing on the motor input values or just set the `value` atribute
        directly to bypass the smoothing algorithm.

        .. note:: Since the change in speed (target - initial) is also used to determine how much
            time will be used to smooth the input, this attribute's value will represent the
            maximum time it takes for the motor to go from full reverse to full forward and vice
            versa. If the motor is going from rest to either full reverse or full forward, then
            the time it takes to do that will be half of this attribute's value.
        """
        return self._dt

    @ramp_time.setter
    def ramp_time(self, delta_t):
        self._dt = abs(delta_t)

    def _start_thread(self):
        if self._smoothing_thread is not None:
            self._stop_thread()
        self._smoothing_thread = Thread(target=self._smooth)
        self._smoothing_thread.start()

    def _stop_thread(self):
        if self._smoothing_thread is not None:
            self._cancel_thread = True
            self._smoothing_thread.join()
            self._cancel_thread = False

    def _smooth(self):
        while not self._cancel_thread:
            self.sync()
            if self._cancel_thread:
                break

    def sync(self):
        """This function should be used at least in the application's main loop iteration. It will
        trigger the smoothing input operations on the output value if needed. This is not needed if
        the smoothing algorithms are not utilized/necessary in the application"""
        time_i = int(time.monotonic() * 1000)
        # print('target speed: {}, init speed: {}'.format(self._target_speed, self._init_speed))
        # print('time_i: {}'.format(time_i))
        # print('end smoothing: {}, init smooth: {}'.format(self._end_smooth, self._init_smooth))
        if time_i < self._end_smooth and self.value != self._target_speed and \
            type(self) not in (Solenoid,):
            delta_speed = (1 - cos((time_i - self._init_smooth) / float(self._end_smooth \
                - self._init_smooth) * PI)) / 2
            self.value = int(delta_speed * (self._target_speed - self._init_speed) + \
                 self._init_speed)
            # print('delta speed: {}'.format(delta_speed))
        else:
            # print('done changing speed')
            self.value = self._target_speed
            self._cancel_thread = True

    @property
    def is_cellerating(self):
        """This attribute contains a `bool` indicating if the motor's speed is in the midst of
        changing. (read-only)"""
        if self._smoothing_thread is not None:
            return self._smoothing_thread.is_alive()
        return self._target_speed != self.value

    # let target_speed be the percentual target speed [-1,1]
    def cellerate(self, target_speed):
        """A function to smoothly accelerate/decelerate the motor to a specified target speed.

        :param int target_speed: The desired target speed in range of [-65535, 65535]. Any invalid
            inputs will be clamped to an `int` value in the proper range.
        """
        self._target_speed = max(-65535, min(65535, int(target_speed)))
        # integer of milliseconds
        self._init_smooth = int(time.monotonic() * 1000)
        self._init_speed = self.value
        delta_time = abs((self._target_speed - self._init_speed) / 131070)
        # print('dt calculated: {}'.format(delta_time))
        self._end_smooth = int(self._init_smooth + delta_time * self.ramp_time)
        if IS_THREADED:
            self._start_thread()
        else:
            self.sync()

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
        return self._signals[0].value or self._signals[1].value if \
            len(self._signals) > 1 else self._signals[0].value

    @value.setter
    def value(self, val):
        val = max(-1, min(1, int(val)))
        # going forward
        if val > 0:
            self._signals[0] = True
            if len(self._signals) > 1:
                self._signals[1] = False
        # going backward
        elif val < 0:
            self._signals[0] = False
            if len(self._signals) > 1:
                self._signals[1] = True
        # otherwise stop
        else:
            self._signals[0] = False
            if len(self._signals) > 1:
                self._signals[1] = False

    def __del__(self):
        if self.value:
            self.value = 0
        self._cancel_thread = False
        if self._smoothing_thread is not None:
            del self._smoothing_thread
        for signal in self._signals:
            signal.deinit()
        self._signals.clear()
# end Solenoid parent class

class BiMotor(Solenoid):
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
    def __init__(self, pins, ramp_time=500):
        super(BiMotor, self).__init__(pins, ramp_time)
        for i in range(len(self._signals) - 1, -1, -1):
            self._signals[i].deinit()
            del self._signals[i]
        for pin in pins:
            if len(self._signals) == 2:
                break
            self._signals.append(PWMOut(pin))

    @property
    def value(self):
        """This attribute contains the current output value of the solenoid(s) in range
        [-65535, 65535]. An invalid input value will be clamped to an `int` in the proper range.
        A negative value represents the motor's speed in reverse rotation. A positive value
        reprsents the motor's speed in forward rotation."""
        return self._signals[0].duty_cycle - \
            (self._signals[1].duty_cycle if len(self._signals) > 1 else 0)

    @value.setter
    def value(self, val):
        val = max(-65535, min(65535, int(val)))
        # going forward
        if val > 0:
            self._signals[0].duty_cycle = val
            if len(self._signals) > 1:
                self._signals[1].duty_cycle = 0
        # going backward
        elif val < 0:
            self._signals[0].duty_cycle = 0
            if len(self._signals) > 1:
                self._signals[1].duty_cycle = (val * -1)
        # otherwise stop
        else:
            self._signals[0].duty_cycle = 0
            if len(self._signals) > 1:
                self._signals[1].duty_cycle = 0
# end BiMotor child class


class PhasedMotor(Solenoid):
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
    def __init__(self, pins, ramp_time=500):
        if len(pins) != 2:
            raise ValueError('The number of pins used must be 2.')
        super(PhasedMotor, self).__init__(pins, ramp_time)
        # free up the 2nd digital pin from parent's declaration
        self._signals[1].deinit()
        # remove 2nd pin from list and re-append it as PWMOut object
        del self._signals[1]
        self._signals.append(PWMOut(pins[1]))

    @property
    def value(self):
        """This attribute contains the current output value of the solenoid(s) in range
        [-65535, 65535]. An invalid input value will be clamped to an `int` in the proper range.
        A negative value represents the motor's speed in reverse rotation. A positive value
        reprsents the motor's speed in forward rotation."""
        return self._signals[1].duty_cycle * (1 if self._signals[0].value else -1)

    @value.setter
    def value(self, val):
        val = max(-65535, min(65535, int(val)))
        # going forward
        if val > 0:
            self._signals[0].value = True
            self._signals[1].duty_cycle = val
        # going backward
        elif val < 0:
            self._signals[0].value = False
            self._signals[1].duty_cycle = (val * -1)
        # otherwise stop
        else:
            self._signals[0].value = False
            self._signals[1].duty_cycle = 0
# end PhasedMotor child class
