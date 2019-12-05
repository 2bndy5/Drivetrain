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
        super().__init__()

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
        self._smoothing_thread = Thread(target=self._smooth_loop)
        self._smoothing_thread.start()

    def _stop_thread(self):
        if self._smoothing_thread is not None:
            self._cancel_thread = True
            self._smoothing_thread.join()
            self._cancel_thread = False

    def _smooth_loop(self):
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

    def __len__(self):
        return 1

    def __del__(self):
        if self.value:
            self.value = 0
        self._cancel_thread = True
        if self._smoothing_thread is not None:
            self._stop_thread()
        del self._smoothing_thread,  self._value, self._init_speed, self._target_speed, self._init_smooth, self._end_smooth, self._dt, self._smoothing_thread, self._cancel_thread
        # super().__del__()

class SmoothDrivetrain:
    """A base class that is only used for inheriting various types of drivetrain configurations."""
    def __init__(self, max_speed=100, smooth=True):
        #  prototype motors lust to avoid error in __del__ on exceptions
        self._motor_pool = None
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
        self._motor_pool.sync()

    def stop(self):
        """This function will stop all motion in the drivetrain's motors"""
        commands = []
        for _ in range(len(self._motor_pool)):
            commands.append(0)
        self.go(commands)

    def go(self, cmds, smooth=None):
        """A helper function to the child classes to handle all motors attached to the
        Drivetrain object.

        :param list,tuple cmds: A `list` or `tuple` of drivetrain input values to be
            interpreted and passed to the motors.
        :param bool smooth: This controls the motors' built-in algorithm that smooths input values
            over a period of time (in milliseconds) contained in the motors'
            :attr:`~drivetrain.smoothing_input.SmoothMotor.ramp_time` attribute. If this parameter
            is not specified, then the drivetrain's `smooth` attribute is used by default.
            This can be disabled per motor by setting the
            :attr:`~drivetrain.smoothing_input.SmoothMotor.ramp_time` attribute to ``0``, thus
            the smoothing algorithm is automatically bypassed despite this parameter's value.

            .. note:: Assert this parameter (set as `True`) for robots with a rather high center of
                gravity or if some parts are poorly attached. The absence of properly smoothed
                acceleration/deceleration will likely make the robot fall over or loose parts
                become dislodged on sudden and drastic changes in speed.
        """
        self._prev_cmds = cmds
        # pylint disable=no-member
        self._motor_pool.go(cmds, smooth=(smooth if smooth is not None else self._smooth))
        # pylint enable=no-member
        self.sync()

    @property
    def value(self):
        """This attribute returns the last `list`/`tuple` of drivetrain commands passed to
        :py:meth:`~drivetrain.helpers.smoothing_input.SmoothDrivetrain.go()` (read-only)"""
        return self._prev_cmds

    @property
    def is_cellerating(self):
        """This attribute contains a `bool` indicating if any the drivetrain's motors' speed is
        in the midst of changing. (read-only)"""
        return self._motor_pool.is_cellerating

    @property
    def max_speed(self):
        """This attribute determines a motor's top speed. Valid input values range [0, 100]."""
        return self._max_speed

    @max_speed.setter
    def max_speed(self, val):
        self._max_speed = max(0, min(val if val is not None else 0, 100))

    def __len__(self):
        return 1

    def __del__(self):
        del self._motor_pool, self._max_speed, self._prev_cmds, self._smooth
