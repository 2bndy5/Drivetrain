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
    from .digi_io import DigitalInOut
from circuitpython_nrf24l01 import RF24
from .smoothing_input import SmoothMotor
# pylint: disable=too-few-public-methods,invalid-name

class MotorPool:
    """A grouping object for various different motors. This object coordinates the motor input
    commands with the motor objects, but optimizes the number of times the communication buses are
    used for the entire motor pool.

    :param list,tuple motors: This `list` or `tuple` of motors will be used for outputting
        motor signal(s) from the user input drivetrain commands. Each item in this list
        must be a `SmoothMotor` or `Solenoid` type object. This parameter isn't
        strictly required during instantiation and the internal motors `list` can be altered via
        the `insert()`, `append()`, `pop()`, and `remove()` methods.
    """
    def __init__(self, motors=None):
        self._motors = []
        if motors is not None:
            for i, m in enumerate(motors):
                if not isinstance(m, (SmoothMotor, Solenoid)):
                    raise ValueError("unrecognized or unsupported motor object type"
                                     " {} in list at index {}".format(type(m), i))
            self._motors = list(motors)

    def go(self, cmds, smooth=True):
        """This function takes the input commands and passes them to the motors.

        :param list,tuple cmds: A `list` or `tuple` of values to be passed to the motors.
        :param bool smooth: This controls the motors' built-in algorithm that smooths input values
            over a period of time (in milliseconds) contained in the motors'
            :attr:`~drivetrain.smoothing_input.SmoothMotor.ramp_time` attribute. This can be
            disabled per motor by setting the
            :attr:`~drivetrain.smoothing_input.SmoothMotor.ramp_time` attribute to ``0``, thus
            the smoothing algorithm is automatically bypassed despite this parameter's value.
        """
        for i, cmd in enumerate(cmds):
            if i < len(self._motors):
                if smooth: # if input is getting smoothed
                    self._motors[i].cellerate(cmd)
                else:
                    self._motors[i].value = cmd

        # i = 0
        # while i < len(self._motors):
        #     skip = len(self._motors[i])
        #     for j in range(skip):
        #         if smooth:
        #             self._motors[i + j].cellerate(cmds[i + j])
        #         else:
        #             self._motors[i + j].value = cmds[i + j]
        #     i += skip

    @property
    def is_cellerating(self):
        for m in self._motors:
            if m.is_cellerating:
                return True
        return False

    def sync(self):
        """This function will trigger each motor's asynchronous code to execute (mainly used for
        smoothing inputs). See also :meth:`~drivetrain.smoothing_input.SmoothMotor.sync`.
        """
        for m in self._motors:
            m.sync()

    def insert(self, index, obj):
        """Use this function to insert a motor into the `MotorPool` object's internal motors `list`.

        :param int index: The position in the `list` of motors to insert the object at.
        :param Solenoid,SmoothMotor obj: The motor object to be inserted into the `list` of motors.
        """
        if not isinstance(obj, (SmoothMotor, Solenoid)):
            raise ValueError("unrecognized or unsupported motor object type {}".format(type(obj)))
        self._motors.insert(index, obj)

    def remove(self, obj):
        """Use this function to remove a specific item from the `MotorPool` object's internal motors
        `list`. This function does not release the pins associated with the motor object, so
        you can re-use the motor object for a different `MotorPool` object. To release the pins
        used by a motor object, you need only delete the object (eg. ``del
        motor_object_variable_name``).

        :param Solenoid,SmoothMotor obj: The motor object to be removed from the `list` of motors.
        """
        self._motors.remove(obj)

    def append(self, obj):
        """Use this function to append to the end of the `MotorPool` object's internal motors
        `list`.

        :param Solenoid,SmoothMotor obj: The motor object to be appended to the `list` of motors.
        """
        if not isinstance(obj, (SmoothMotor, Solenoid)):
            raise ValueError("unrecognized or unsupported motor object type {}".format(type(obj)))
        self._motors.append(obj)

    def pop(self, index):
        """Use this function to remove an item from the `MotorPool` object's internal motors
        `list`. This function also returns the object removed from the `list`, however the pins
        are not released meaning the motor object can still be used for another `MotorPool` object.
        To release the pins used by a motor object, you need only delete the object (eg. ``del
        motor_object_variable_name``).

        :param int index: The position of the motor object to be removed from the `list` of motors.
        """
        if index in range(len(self._motors) - 1):
            return self._motors.pop(index)

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

    :param ~digitalio.DigitalInOut,drivetrain.digi_io.DigitalInOut pos_pin: The digital
        output pin to control the first solenoid.
    :param ~digitalio.DigitalInOut,drivetrain.digi_io.DigitalInOut neg_pin: The digital
        output pin to control the second solenoid.

    .. note:: this class was primarily intended for abstract inheritence, thus the pin parameters
        are optional.
    """
    def __init__(self, pos_pin=None, neg_pin=None):
        self._signals = []
        if pos_pin is not None and neg_pin is not None:
            if pos_pin is not None:
                pos_pin.switch_to_output(False)
                self._signals.append(pos_pin)
            if neg_pin is not None:
                neg_pin.switch_to_output(False)
                self._signals.append(neg_pin)

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

    def deinit(self):
        """ initialize the pins associated with the solenoid(s) """
        if self.value:
            self.value = 0
        for signal in self._signals:
            signal.deinit()

    def __del__(self):
        self.deinit()
        self._signals.clear()
# end Solenoid parent class

class BiMotor(SmoothMotor, Solenoid):
    """This class is meant be used for motors driven by driver boards/ICs that expect 2 PWM outputs
    . Each pin represent the controlling signal for the motor's speed in a single rotational
    direction.

    :param ~pulseio.PWMOut,drivetrain.pwm.PWMOut pos_pin: The PWM output pin to control
        the motor's speed rotating forwards.
    :param ~pulseio.PWMOut,drivetrain.pwm.PWMOut neg_pin: The PWM output pin to control
        the motor's speed rotating backwards.
    :param int ramp_time: The time (in milliseconds) that is used to smooth the motor's input.
        Default is 500. This time represents the maximum amount of time that the input will be
        smoothed. Since the change in speed is also used to determine how much time will be used
        to smooth the input, this parameter's value will represent the time it takes for the motor
        to go from full reverse to full forward and vice versa. If the motor is going from rest to
        either full reverse or full forward, then the time it takes to do that will be half of
        this parameter's value. This can be changed at any time by changing the `ramp_time`
        attribute.

    .. note:: If only 1 pin is passed, then the motor will only rotate in 1 direction depending on
        how the motor is connected to the motor driver. Tthis class assumes that the one direction
        it is rotating is the forward direction (``plus_pin`` parameter).
    """
    def __init__(self, pos_pin, neg_pin=None, ramp_time=500):
        super(BiMotor, self).__init__(ramp_time=ramp_time)
        self._signals = [pos_pin]
        if neg_pin is not None:
            self._signals.append(neg_pin)

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
# end BiMotor child class

class PhasedMotor(SmoothMotor, Solenoid):
    """This class is meant be used for motors driven by driver boards/ICs that expect:

        * 1 PWM output (to control the motor's speed)
        * 1 digital output (to control the motor's rotational direction)

    :param ~digitalio.DigitalInOut,drivetrain.digi_io.DigitalInOut phased_pin: The digital
        output pin to control the motor's rotational direction.
    :param ~pulseio.PWMOut,drivetrain.pwm.PWMOut pwm_pin: The PWM output pin to control
        the motor's speed.
    :param int ramp_time: The time (in milliseconds) that is used to smooth the motor's input.
        Default is 500. This time represents the maximum amount of time that the input will be
        smoothed. Since the change in speed is also used to determine how much time will be used
        to smooth the input, this parameter's value will represent the time it takes for the motor
        to go from full reverse to full forward and vice versa. If the motor is going from rest to
        either full reverse or full forward, then the time it takes to do that will be half of
        this parameter's value. This can be changed at any time by changing the `ramp_time`
        attribute.
    """
    def __init__(self, phased_pin, pwm_pin, ramp_time=500):
        super(PhasedMotor, self).__init__(ramp_time=ramp_time)
        self._signals.insert(0, phased_pin)
        self._signals[0].switch_to_output(False)
        self._signals.insert(1, pwm_pin)

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
# end PhasedMotor child class
