"""a mixin module that holds all code related to smoothing the motor inputs"""
from math import pi as PI, cos
import time
IS_THREADED = True
try:
    from threading import Thread
except ImportError:
    IS_THREADED = False

class SmoothMotor:
    """mixin class to incorporate smoothing input algorithm on each motor"""
    def __init__(self, ramp_time=500):
        # variables used to track acceleration
        self._value = 0
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
        disable all smoothing on the motor input values or just set the
        `value` attribute directly to bypass the smoothing algorithm.

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

    @property
    def value(self):
        """This value attribute's setter and getter are overidden by child classes because this is where
        the PWM go out to the motors (usually)"""
        return self._value

    @value.setter
    def value(self, val):
        self._value = val

    def sync(self):
        """This function should be used at least in the application's main loop iteration. It will
        trigger the smoothing input operations on the output value if needed. This is not needed if
        the smoothing algorithms are not utilized/necessary in the application"""
        time_i = int(time.monotonic() * 1000)
        # print('target speed: {}, init speed: {}'.format(self._target_speed, self._init_speed))
        # print('time_i: {}'.format(time_i))
        # print('end smoothing: {}, init smooth: {}'.format(self._end_smooth, self._init_smooth))
        if time_i < self._end_smooth and self.value != self._target_speed:
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
        return self._target_speed != self._value

    # let target_speed be the percentual target speed [-1,1]
    def cellerate(self, target_speed):
        """A function to smoothly accelerate/decelerate the motor to a specified target speed.

        :param int target_speed: The desired target speed in range of [-65535, 65535]. Any invalid
            inputs will be clamped to an `int` value in the proper range.
        """
        self._target_speed = max(-65535, min(65535, int(target_speed)))
        # integer of milliseconds
        self._init_smooth = int(time.monotonic() * 1000)
        self._init_speed = self._value
        delta_time = abs((self._target_speed - self._init_speed) / 131070)
        # print('dt calculated: {}'.format(delta_time))
        self._end_smooth = int(self._init_smooth + delta_time * self.ramp_time)
        if IS_THREADED:
            self._start_thread()
        else:
            self.sync()

    def __del__(self):
        self._cancel_thread = False
        if self._smoothing_thread is not None:
            del self._smoothing_thread

class SmoothDrivetrain:
    """A base class that is only used for inheriting various types of drivetrain configurations."""
    def __init__(self, max_speed=100, smooth=True):
        #  prototype motors lust to avoid error in __del__ on exceptions
        self._motors = []
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
        Drivetrain object. This is only useful for motors that serve a specialized purpose
        other than propulsion.

        :param list,tuple cmds: A `list` or `tuple` of `int` motor input values to be passed in
            corresponding order to the motors. Each input value must be on range [-65535, 65535].

        :param bool smooth: This controls the motors' built-in algorithm that smooths input values
            over a period of time (in milliseconds) contained in the motors'
            :attr:`~drivetrain.smoothing_input.SmoothMotor.ramp_time` attribute. If this parameter is not
            specified, then the drivetrain's `smooth` attribute is used by default.
            This can be disabled per motor by setting the :attr:`~drivetrain.smoothing_input.SmoothMotor.ramp_time`
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
        self.sync()

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
