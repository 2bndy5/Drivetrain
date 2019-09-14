"""
A driver class to implement the threaded module for unipolar stepper motors.
"""
import time
import math
from gpiozero import DigitalOutputDevice, SourceMixin, CompositeDevice
from gpiozero.exc import OutputDeviceBadValue
from gpiozero.threads import GPIOThread


class Stepper(SourceMixin, CompositeDevice):
    """A class designed to control unipolar stepper motors. It is still a work in progress as there is no smoothing algorithm applied to the motor's input.

    :param list, tuple pins: A `list` or `tuple` of pins that are connected to the stepper motor. Usually this list is 4 items long, but there is no value checking done here (the board module will still throw an exception when applicable).
    :param int steps_per_rev: An `int` that represents how many steps it takes to complete a whole revolution. Defaults to 4096.
    :param float degree_per_step: A decimal value that represents how many degrees the motor moves per single step.
    :param string step_type: This parameter is used upon instantiation to specify what kind of stepping pattern the motor uses. Valid values are limited to ``half``,``full``, or ``wave``; defaults to ``half``.
    :param int rpm: An `int` representing the maximum amount of rotations per minute.

    """
    def __init__(self, pins, min_angle=-180, max_angle=180, speed=60, step_type='half', teps_per_rev=4069, degree_per_step=0.087890625, pin_factory=None):
        self.spr = teps_per_rev
        self.dps = degree_per_step
        self.step_type = step_type
        self.speed = speed
        self._pins = pins
        self._target_pos = None
        self.min_angle = min_angle
        self.max_angle = max_angle
        super(Stepper, self).__init__(pin_factory=pin_factory)
        if len(pins) == 4:
            self._pins = CompositeDevice(
                DigitalOutputDevice(pins[0], pin_factory=pin_factory),
                DigitalOutputDevice(pins[1], pin_factory=pin_factory),
                DigitalOutputDevice(pins[2], pin_factory=pin_factory),
                DigitalOutputDevice(pins[3], pin_factory=pin_factory),
                pin_factory=pin_factory)
        else:  # did not pass exactly 4 gpio pins
            raise RuntimeError(
                'pins passed to stepper must be an iterable list of exactly 4 numbers!')
        self._it = 0  # iterator for rotating stepper
        # self._steps = steps specific to motor
        self.reset_0_angle()  # init self._steps = 0
        self._move_thread = None

    # override [] operators to return the CompositeDevice's list of DigitalOutputDevice(s)
    def __getitem__(self, key):
        return self._pins[key]

    def __setitem__(self, key, val):
        self._pins[key].value = bool(math.ceil(abs(val)))

    def reset_0_angle(self):
        """A calibrating function that will reset the motor's zero angle to its current position."""
        self._steps = 0

    def _step(self, is_cw=True):
        # increment or decrement step
        if is_cw:  # going CW
            self._steps -= 1
            self._it -= 1
        else:  # going CCW
            self._steps += 1
            self._it += 1
        # now check for proper range according to stepper type

    def wrap_it(self, maximum, minimum=0, theta=None):
        """
        Ensure that argument 'theta' is kept accordingly within range [minimum,maximum]
        """
        if theta is None:
            theta = self._it
        while theta > maximum:
            theta -= maximum
        while theta < minimum:
            theta += maximum
        return theta

    def is_cw(self):
        """This determines the rotational direction that is the fastest route towards target_pos"""
        if self._target_pos is None and self._target_pos != self._steps:
            return None
        curr_pos = self.wrap_it(self.spr, 0, self._steps % self.spr)
        return self.wrap_it(self.spr, 0, curr_pos - self.spr / 2) < self._target_pos < curr_pos

    def _write(self):
        if self.step_type == "half":
            max_steps = 8
            self._it = self.wrap_it(max_steps)
            base = int(self._it / 2)
            plus1 = base + 1
            if plus1 == len(self._pins):
                plus1 = 0
            for i, pin in enumerate(self._pins):
                if i == base or (i == plus1 and self._it % 2 == 1):
                    pin.value = True
                else:
                    pin.value = False
        elif self.step_type == "full":
            max_steps = 4
            self._it = self.wrap_it(max_steps)
            if self._it + 1 == max_steps:
                plus1 = 0
            else:
                plus1 = self._it + 1
            for i, pin in enumerate(self._pins):
                if i in (self._it, plus1):
                    pin.value = True
                else:
                    pin.value = False
        elif self.step_type == "wave":
            max_steps = 4
            self._it = self.wrap_it(max_steps)
            for i, pin in enumerate(self._pins):
                if i == self._it:
                    pin.value = True
                else:
                    pin.value = False
        else:  # step_type specified is invalid
            raise RuntimeError('Invalid Stepper Type = {}'.format(repr(self.step_type)))

    def _delay(self):
        # delay between iterations based on motor speed (mm/sec)
        time.sleep(self.speed / 60000.0)

    def _get_pin_bin(self):
        pin_bin = 0b0
        for i in range(len(self._pins)):
            pin_bin |= (int(self._pins[i].value) << i)
        return pin_bin

    def __repr__(self):
        output = 'pins = {} Angle: {} Steps: {}'.format(
            bin(self._get_pin_bin()), repr(self.angle), repr(self.steps))
        return output

    def _stop_thread(self):
        if getattr(self, '_move_thread', None):
            self._move_thread.stop()
        self._move_thread = None

    def _move_steps(self):
        while self._target_pos is not None and self._steps != self._target_pos:
            print(self._steps, '!=', self._target_pos)
            # iterate self._steps
            self._step(self.is_cw())
            # write to pins
            self._write()
            # wait a certain amount of time based on motor speed
            self._delay()

    @property
    def angle(self):
        """
        Returns current angle of motor rotation [0,360]. Setting
        this property changes the state of the device.
        """
        return self.wrap_it(360, 0, (self._steps % self.spr) * self.dps)

    @angle.setter
    def angle(self, angle):
        """
        Rotate motor to specified angle where direction is
        automatically detirmined toward the shortest route.
        All input angle is valid since it is wrapped to range [0,360]. *see wrap_it()
        """
        # wrap_it angle to constraints of [0,360] degrees
        angle = self.wrap_it(360, 0, angle)
        self._target_pos = None
        self._stop_thread()
        self._target_pos = round(angle / self.dps)
        print('targetPos =', self._target_pos)
        self._move_thread = GPIOThread(target=self._move_steps)
        self._move_thread.start()

    @property
    def steps(self):
        """
        Returns counter of steps taken since instantiation or reset_0_angle()
        """
        return self._steps

    @steps.setter
    def steps(self, numb_steps):
        """
        Task motor to execute specified numb_steps where direction is the +/- sign on the numb_steps variable
        """
        self._target_pos = None
        self._stop_thread()
        self._target_pos = self.wrap_it(self.spr, 0, numb_steps)
        print('targetPos =', self._target_pos)
        self._move_thread = GPIOThread(target=self._move_steps)
        self._move_thread.start()

    @property
    def is_active(self):
        """
        Returns :data:`True` if the motor is currently running and
        :data:`False` otherwise.
        """
        if self._move_thread is not None:
            return not self._move_thread._is_stopped
        else:
            return False

    @property
    def value(self):
        """
        Returns the percent angle of the motor.
        """
        return (self._steps % self.spr) / self.spr

    @value.setter
    def value(self, value):
        """
        Sets the percent angle of the motor in range [-180,180].
        Valid input value is in range [-1,1]
        """
        if value is None:
            self.reset_0_angle()
        elif -1 <= value <= 1:
            self._target_pos = None
            self._stop_thread()
            self._target_pos = self.wrap_it(self.spr, 0, self.spr / 2 * value)
            print('targetPos =', self._target_pos)
            self._move_thread = GPIOThread(target=self._move_steps)
            self._move_thread.start()
        else:
            raise OutputDeviceBadValue("stepper value must be between -1 and 1, or None")


if __name__ == "__main__":
    # pylint: disable=invalid-name
    from gpiozero.pins.mock import MockFactory
    mockpins = MockFactory()
    m = Stepper([5, 6, 12, 16], pin_factory=mockpins)
    m.angle = -15
    # m.steps = 64
    time.sleep(1)
    print(repr(m))
    m.value = 0.5
    # m.angle = 15
    # m.steps = 500
    time.sleep(2)
    print(repr(m))
    # m.steps = -512
    m.angle = 0
    time.sleep(1)
    print(repr(m))
