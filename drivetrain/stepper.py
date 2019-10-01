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

StepperMotor
=============

a unipolar stepper motor driver class

"""
import time
from math import floor, ceil
import digitalio
IS_THREADED = True
try:
    from multiprocessing import Process
except ImportError:
    IS_THREADED = False

# pylint: disable=too-many-branches,too-many-instance-attributes

class StepperMotor():
    """A class designed to control unipolar or bipolar stepper motors. It is still a work in
    progress as there is no smoothing algorithm nor limited maximum speed applied to the motor's
    input.

    :param list,tuple pins: A `list` or `tuple` of (`board` module) pins that are used to drive the
        stepper motor. The length of this `list` or `tuple` must be divisible by 2, otherwise a
        `ValueError` exception is thrown.

    :param int steps_per_rev: An `int` that represents how many steps it takes to complete a whole
        revolution. Defaults to 4096. This should correlate with information found in your
        motor's datasheet.

    :param int,float degree_per_step: The value that represents how many degrees the motor moves
        per single step. Defaults to 45/512 or 5.625°/64. This should correlate with information
        found in your motor's datasheet.

    :param string step_type: This parameter is used upon instantiation to specify what kind of
        stepping pattern the motor uses. Valid values are limited to:

            * ``half`` (default value)
            * ``full``
            * ``wave``

         This should correlate with information found in your motor's datasheet.

    :param int,float rpm: The maximum amount of rotations per minute. This should correlate with
        information found in your motor's datasheet.

    """
    def __init__(self, pins, steps_per_rev=4096, degree_per_step=45/512, step_type='half', rpm=60):
        if len(pins) % 2 or not pins:
            raise ValueError('The number of pins should be divisible by 2 and greater than 0.')
        if step_type not in ('half', 'full', 'wave'):
            raise ValueError(f"Unrecognized step type {step_type}. Expected 'half', 'full', or 'wave'")
        self._spr = steps_per_rev # max # of steps in 1 revolution
        self._dps = degree_per_step # used to calculate angle
        self._step_type = step_type # can be 'wave', 'full', or 'half' for stepping pattern
        self._rpm = rpm # max speed in rotations per minute
        self._pins = [] # list of pins including getter & setters
        for i, pin in enumerate(pins):
            # configure each pin for digital output
            self._pins.append(digitalio.DigitalInOut(pin))
            self._pins[i].switch_to_output()
        self.reset0angle() # init self._steps = 0
        # self._steps = steps (specific to motor) for position tracking
        self._target_pos = 0
        self._it = 0 # iterator for rotating stepper
        self._end_delay_time = 0
        self._brake = not IS_THREADED
        self._move_thread = None

    @property
    def is_cellerating(self):
        """This attribute contains a `bool` indicating if the motor is in the midst of moving.
            (read-only)

        """
        return self._target_pos != self._steps if self._target_pos is not None and self._move_thread is not None else False

    def stop(self):
        """Use this function when you want to abort any motion from the motor."""
        if IS_THREADED:
            self._stop_thread()
        self._target_pos = self._steps

    def _stop_thread(self):
        self._brake = True
        if self._move_thread is not None:
            self._move_thread.terminate()
        self._move_thread = None
        self._brake = False

    def _start_thread(self):
        self._move_thread = Process(target=self._move_steps)
        self._move_thread.start()

    def _move_steps(self):
        while not self._brake:
            self.sync()

    def sync(self):
        """This function should be used only once per main loop iteration. It will trigger stepping operations on the motor if needed."""
        if self._steps != self._target_pos and time.monotonic() < self._end_delay_time:
            # print(self._steps, '!=', self._target_pos)
            # iterate self._steps
            self._step()
            # write to pins
            self._write()
            # wait a certain amount of time based on motor speed
            self._end_delay_time = time.monotonic() + self._delay
        else:
            self._brake = True

    def reset0angle(self):
        """A calibrating function that will reset the motor's zero angle to its current position.
            This function is also called when the motor's `value`, `steps`, or `angle`
            attributes are set to `None`. Additionally, this function will stop all movement in the
            motor.

        """
        self.stop()
        self._steps, self._target_pos = (0, 0)


    def _wrap_it(self, maximum, minimum=0, theta=None):
        """
        Ensure that argument 'theta' is kept accordingly within range [minimum,maximum]
        """
        if theta is None:
            theta = self._it
        while theta >= maximum:
            theta -= maximum
        while theta < minimum:
            theta += maximum
        return theta

    @property
    def _is_cw(self):
        """determine the shortest route toward target position"""
        curr_pos = self._wrap_it(self._spr, 0, self._steps % self._spr)
        if curr_pos > self._target_pos:
            return curr_pos - self._target_pos < self._spr / 2
        return self._target_pos - curr_pos > self._spr / 2

    def _step(self):
        """ increment or decrement _steps """
        if self._is_cw: # going CW
            self._steps -= 1
            self._it -= 1
        else: # going CCW
            self._steps += 1
            self._it += 1

    def _write(self):
        """change pin states based on _steps iterator"""
        if self._step_type == "half":
            max_steps = 8
            self._it = self._wrap_it(max_steps)
            base = floor(self._it / 2)
            plus1 = base + 1 if base + 1 < len(self._pins) else 0
            for i, pin in enumerate(self._pins):
                if i == base or (i == plus1 and self._it % 2 == 1):
                    pin.value = True
                else:
                    pin.value = False
        elif self._step_type == "full":
            max_steps = 4
            self._it = self._wrap_it(max_steps)
            plus1 = self._it + 1 if self._it + 1 < len(self._pins) else 0
            for i, pin in enumerate(self._pins):
                if i in (self._it, plus1):
                    pin.value = True
                else:
                    pin.value = False
        elif self._step_type == "wave":
            max_steps = 4
            self._it = self._wrap_it(max_steps)
            for i, pin in enumerate(self._pins):
                if i == self._it:
                    pin.value = True
                else:
                    pin.value = False

    @property
    def _delay(self):
        """time (in seconds) to delay between iterations based on motor's rpm attribute.
        (read-only)

        """
        return self._rpm / 60000.0

    @property
    def rpm(self):
        """This `int` attribute contains the maximum Rotations Per Minute and can be changed at any
        time.

        """
        return self._rpm

    @rpm.setter
    def rpm(self, val):
        self._rpm = int(val)

    @property
    def _pin_bin(self):
        """get all stepper's pin's boolean values as binary number where each bit represents a
        pin

        """
        pin_bin = 0b0
        for i in range(len(self._pins)):
            pin_bin = (pin_bin << 1) | int(self._pins[i].value)
        return pin_bin

    def __repr__(self):
        return f'pins: {bin(self._pin_bin)} Angle: {self.angle} Steps: {self.steps} Value: {self.value}'

    @property
    def angle(self):
        """Represents the number of the motor's angle from its zero angle position with respect to
            the ``steps_per_rev`` parameter passed to the constructor. This value will
            be in range [-180, 180]. Input values can be any `int` or `float` as any overflow
            outside the range [0, 360] is handled accordingly.

        """
        return self._wrap_it(360, 0, (self._steps % self._spr) * self._dps)

    @angle.setter
    def angle(self, val):
        # _wrap_it angle to constraints of [0,360] degrees
        if val is None:
            self.reset0angle()
        else:
            self.stop()
            self._target_pos = ceil(self._wrap_it(360, 0, val) / self._dps)
            # print('targetPos =', self._target_pos, 'curr_pos =', self.steps)
            if IS_THREADED:
                self._start_thread()
            else:
                self.sync()

    @property
    def steps(self):
        """Represents the number of the motor's steps from its zero angle position with respect to
            the ``steps_per_rev`` parameter passed to the constructor. This value will
            be in range [``steps_per_rev / -2``, ``steps_per_rev / 2``]. Input values
            can be any `int` as any overflow outside the range [``0``, ``steps_per_rev``] is
            handled accordingly.

        """
        return self._wrap_it(self._spr, 0, self._steps)

    @steps.setter
    def steps(self, val):
        if val is None:
            self.reset0angle()
        else:
            self.stop()
            self._target_pos = self._wrap_it(self._spr, 0, val)
            # print('targetPos =', self._target_pos, 'curr_pos =', self.steps)
            if IS_THREADED:
                self._start_thread()
            else:
                self.sync()

    @property
    def value(self):
        """Represents the percentual value of the motor's angle in range [-100, 100] with respect to
            the ``steps_per_rev`` parameter passed to the constructor. Invalid input values
            will be constrained to an `int` in the range [-100, 100].

        """
        return round(self._wrap_it(self._spr / 2, self._spr / -2, self._steps) / (self._spr / 2) * 100, 1)

    @value.setter
    def value(self, val):
        if val is None:
            self.reset0angle()
        else:
            self.stop()
            self._target_pos = self._wrap_it(self._spr, 0, round(self._spr * max(-100.0, min(100.0, val)) / 200.0))
            # print('targetPos =', self._target_pos, 'curr_pos =', self.steps)
            if IS_THREADED:
                self._start_thread()
            else:
                self.sync()

    def __del__(self):
        for pin in self._pins:
            pin.deinit()
