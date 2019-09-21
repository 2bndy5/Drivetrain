"""

Stepper
==========

a unipolar stepper motor driver class

"""
import time
from math import floor, ceil
from threading import Thread
import digitalio

# pylint: disable=too-many-branches,too-many-instance-attributes

class Stepper:
    """A class designed to control unipolar stepper motors. It is still a work in progress as there is no smoothing algorithm applied to the motor's input.

    :param list, tuple pins: A `list` or `tuple` of pins that are connected to the stepper motor. Usually this list is 4 items long, but there is no value checking done here (the board module will still throw an exception when applicable).
    :param int steps_per_rev: An `int` that represents how many steps it takes to complete a whole revolution. Defaults to 4096.
    :param float degree_per_step: A decimal value that represents how many degrees the motor moves per single step.
    :param string step_type: This parameter is used upon instantiation to specify what kind of stepping pattern the motor uses. Valid values are limited to ``half``,``full``, or ``wave``; defaults to ``half``.
    :param int rpm: An `int` representing the maximum amount of rotations per minute.

    """
    def __init__(self, pins, steps_per_rev=4096, degree_per_step=0.087890625, step_type='half', rpm=60):
        self._spr = steps_per_rev # max # of steps in 1 revolution
        self._dps = degree_per_step # used to calculate angle
        self._step_type = step_type # can be 'wave', 'full', or 'half' for stepping pattern
        self._rpm = rpm # max speed in rotations per minute
        self._pins = [] # list of pins including getter & setters
        for i, pin in enumerate(pins):
            # configure each pin for digital output
            self._pins.append(digitalio.DigitalInOut(pin))
            self._pins[i].switch_to_output()
        self.reset_0_angle() # init self._steps = 0
        # self._steps = steps (specific to motor) for position tracking
        self._target_pos = 0
        self._brake = False
        self._it = 0 # iterator for rotating stepper
        self._move_thread = None

    def reset_0_angle(self):
        """A calibrating function that will reset the motor's zero angle to its current position."""
        self._brake = True
        self._stop_thread()
        self._steps = 0
        self._brake = False

    def _step(self):
        # increment or decrement step
        if self._is_cw: # going CW
            self._steps -= 1
            self._it -= 1
        else: # going CCW
            self._steps += 1
            self._it += 1
        # now check for proper range according to stepper type

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
        curr_pos = self._wrap_it(self._spr, 0, self._steps % self._spr)
        if curr_pos > self._target_pos:
            return curr_pos - self._target_pos < self._spr / 2
        else: return self._target_pos - curr_pos > self._spr / 2

    def _write(self):
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
        else: # step_type specified is invalid
            raise RuntimeError('Invalid Stepper Type = %s' % repr(self._step_type))

    def _delay(self):
        # delay between iterations based on motor speed (mm/sec)
        time.sleep(self._rpm / 60000.0)

    def _get_pin_bin(self):
        pin_bin = 0b0
        for i in range(len(self._pins)):
            pin_bin = (pin_bin << 1) | int(self._pins[i].value)
        return pin_bin

    def __repr__(self):
        return 'pins: {} Angle: {} Steps: {} Value: {}'.format(bin(self._get_pin_bin()), repr(self.angle), repr(self.steps), repr(self.value))

    def _stop_thread(self):
        if self._move_thread is not None:
            self._move_thread.join()
        self._move_thread = None

    def _move_steps(self):
        while self._target_pos is not None and self._steps != self._target_pos and not self._brake:
            # print(self._steps, '!=', self._target_pos)
            # iterate self._steps
            self._step()
            # write to pins
            self._write()
            # wait a certain amount of time based on motor speed
            self._delay()

    @property
    def is_active(self):
        """This attribute will return a `bool` indicating if the motor is in the midst of moving. (read-only)"""
        return self._target_pos != self._steps if self._target_pos is not None and self._move_thread is not None else False

    @property
    def angle(self):
        """
        Returns current angle of motor rotation [0,360]. Setting
        this property changes the state of the device.
        """
        return self._wrap_it(360, 0, (self._steps % self._spr) * self._dps)

    @angle.setter
    def angle(self, val):
        """
        Rotate motor to specified angle w/ respect to SPR
        All input angle is valid since it is wrapped to range [0,360]. *see _wrap_it()
        """
        # _wrap_it angle to constraints of [0,360] degrees
        if val is None:
            self.reset_0_angle()
        else:
            self._target_pos = ceil(self._wrap_it(360, 0, val) / self._dps)
            # print('targetPos =', self._target_pos, 'curr_pos =', self.steps)
            self._brake = True
            self._stop_thread()
            self._move_thread = Thread(target=self._move_steps)
            self._brake = False
            self._move_thread.start()

    @property
    def steps(self):
        """
        Returns counter of steps taken since instantiation or reset_0_angle()
        """
        return self._wrap_it(self._spr, 0, self._steps)

    @steps.setter
    def steps(self, val):
        """
        Task motor to execute specified # of steps
        """
        if val is None:
            self.reset_0_angle()
        else:
            self._target_pos = self._wrap_it(self._spr, 0, val)
            # print('targetPos =', self._target_pos, 'curr_pos =', self.steps)
            self._brake = True
            self._stop_thread()
            self._move_thread = Thread(target=self._move_steps)
            self._brake = False
            self._move_thread.start()

    @property
    def value(self):
        """
        Returns the percent angle of the motor.
        """
        return round(self._wrap_it(self._spr / 2, self._spr / -2, self._steps) / (self._spr / 2) * 100, 1)

    @value.setter
    def value(self, val):
        """
        Sets the percent angle of the motor in range [-180,180] w/ respect to SPR
        Valid input value is constrained to range [-100,100]
        """
        if val is None:
            self.reset_0_angle()
        else:
            self._target_pos = self._wrap_it(self._spr, 0, round(self._spr * max(-100.0, min(100.0, val)) / 200.0))
            # print('targetPos =', self._target_pos, 'curr_pos =', self.steps)
            self._brake = True
            self._stop_thread()
            self._move_thread = Thread(target=self._move_steps)
            self._brake = False
            self._move_thread.start()

    def __del__(self):
        for pin in self._pins:
            pin.deinit()

if __name__ == "__main__":
    # pylint: disable=invalid-name
    import board
    motor = Stepper([board.D13, board.D12, board.D11, board.D10])
    Steps = [-256, 256, 0] # 1024, 2048, 4096]
    Angle = [-15, 15, 0] # 180, 360]
    Value = [-25, 25, 0] # -50, 100, 0]
    for test in Value:
        motor.value = test # send input instructions
        # do a no delay wait for at least 2 seconds
        start = time.monotonic()
        t = start
        end = None
        while motor.is_active or t < start + 2:
            t = time.monotonic()
            if not motor.is_active and end is None:
                end = t
                print(repr(motor))
                print('value acheived in', end-start, 'seconds')
            # elif motor.is_active:
            #     print(repr(motor))
    del motor
