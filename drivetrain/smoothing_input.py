"""a mixin module that holds all code related to smoothing the motor inputs"""
from math import pi as PI, cos
import time
IS_THREADED = True
try:
    from threading import Thread
except ImportError:
    IS_THREADED = False

class Smooth:
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
        :py:attr:`~drivetrain.interfaces.NRF24L01.value` attribute
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
        if time_i < self._end_smooth and self._value != self._target_speed:
            delta_speed = (1 - cos((time_i - self._init_smooth) / float(self._end_smooth \
                - self._init_smooth) * PI)) / 2
            self._value = int(delta_speed * (self._target_speed - self._init_speed) + \
                 self._init_speed)
            # print('delta speed: {}'.format(delta_speed))
        else:
            # print('done changing speed')
            self._value = self._target_speed
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
