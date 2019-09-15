"""
motor
======

A collection of driver classes for different single phase DC motors implementing the threading module. Includes Solonoid (parent base class), BiMotor & PhasedMotor (children of Solonoid)
"""
import math
from threading import Thread
import time
import digitalio
import pulseio

class Solonoid:
    """This base class is meant be used as a parent to `BiMotor` and `PhasedMotor` classes of this module, but can be used for solonoids if needed. Solonoids, by nature, cannot be controlled analogously (cannot be any value other than 0 or 100). Despite the fact that this class holds all the smoothing input algorithms for its child classes, the output values are not actually smoothed. With that said, this class can be used to control up to 2 solonoids (see also `value` attribute for more details).

    :param list, tuple pins: A list of pins connected to the solonoid(s).
    :param int ramp_time: This parameter is really a placeholder for the child classes BiMotor & PhasedMotor as it has no affect on objects instantiated with this base class.

    """
    def __init__(self, pins, ramp_time=100):
        self.signals = []
        for i, pin in enumerate(pins):
            # save pwm duty cycles as int [0,100]; direction signals will be treated like bool [0|1]
            self.signals.append(digitalio.DigitalInOut(pin))
            self.signals[i].switch_to_output(False)
        # variables used to track acceleration
        self.init_speed = 0
        self.final_speed = 0
        self.start_smooth = 0
        self.end_smooth = 0
        self.smoothing_thread = None
        self.cancel_thread = False
        # time in milliseconds to change/ramp speed from self.value to self.final_speed
        self._dt = ramp_time

    def _stop_thread(self):
        if self.smoothing_thread is not None:
            self.smoothing_thread.join()
        self.smoothing_thread = None

    def _smooth(self):
        """
        delta_speed, self.final_speed, and self.init_speed are all percentage [-1,1]
        delta_time & dt is in microseconds
        """
        delta_time = int(time.monotonic() * 1000) - self.start_smooth
        while delta_time < (self.end_smooth - self.start_smooth) and self.cancel_thread:
            delta_speed = (1 - math.cos(delta_time / float(self.end_smooth - self.start_smooth) * math.pi)) / 2
            self.value = (delta_speed * (self.final_speed - self.init_speed) + self.init_speed) / 100.0
            delta_time = int(time.monotonic() * 1000) - self.start_smooth
        self.value = self.final_speed / 100.0

    # let final_speed be the percentual target speed [-1,1]
    def cellerate(self, final_speed):
        """
        let final_speed = target speed in range of [-1,1]
        let delta_t = percent [0,1] of delta time (self._dt in milliseconds)
        """
        self.final_speed = max(-100, min(100, round(final_speed * 100)))  # bounds check
        # integer of milliseconds
        self.start_smooth = int(time.monotonic() * 1000)
        self.init_speed = int(self.value * 100)
        delta_t = abs((self.final_speed - self.init_speed) / 200.0)
        self.end_smooth = self.start_smooth + delta_t * self._dt
        self.cancel_thread = False
        self._stop_thread()
        self.smoothing_thread = Thread(target=self._smooth)
        self.cancel_thread = True
        self.smoothing_thread.start()

    @property
    def value(self):
        """This attribute will always return the current output value(s) of the solonoid(s) as a percentage in range [-100,100]. An invalid input value will be clamped to range [-100,100]

        .. note:: Because this class is built to handle 2 pins and tailored for solonoids, any negative value will only energize the solonoid attached to the second pin (passed in ``pins`` parameter to the constructor). Any positive value will only energize the solonoid attached to the first pin (passed in ``pins`` parameter to the constructor). Alternatively, a zero value will de-energize both solonoids

        """
        return self.signals[0].duty_cycle or self.signals[1].duty_cycle if len(self.signals) > 1 else self.signals[0].duty_cycle

    @value.setter
    def value(self, val):
        # check proper range of variable val [-1,1]
        val = max(-100, min(100, round(val * 100)))
        # going forward
        if val > 0:
            self.signals[0] = True
            if len(self.signals) > 1:
                self.signals[1] = False
        # going backward
        elif val < 0:
            self.signals[0] = False
            if len(self.signals) > 1:
                self.signals[1] = True
        # otherwise stop
        else:
            self.signals[0] = False
            if len(self.signals) > 1:
                self.signals[1] = False

    def __del__(self):
        self.cancel_thread = False
        if self.smoothing_thread is not None:
            del self.smoothing_thread
# end Solonoid parent class


class BiMotor(Solonoid):
    """This class is meant be used for motors driver by driver boards that expect 2 PWM outputs. Each pin represent the controlling signal for the motor's speed in a single rotational direction.

    :param list, tuple pins: A list of pins connected to the motor driver. If only one pins is passed than the motor will only turn in one direction.
    :param int ramp_time: The time (in milliseconds) that is used to smooth the motor's input. Default is 500. This time represents the maximum amount of time that the input will be smoothed. Since the change in speed is also used to determine how much time will be used to smooth the input, this parameter's value will represent the time it takes for the motor to go from full reverse to full forward and vice versa. If the motor is going from rest to either full reverse or full forward, then the time it takes to do that will be half of this parameter's value. This can be changed at any time by changing the `ramp_time` attribute

    """
    def __init__(self, pins, rampTime=1000):
        super(BiMotor, self).__init__(pins, rampTime)
        # save pin numbers as GPIO.PWM objects
        self.signals.clear()
        for pin in pins:
            self.signals.append(pulseio.PWMOut(pin))

    @property
    def value(self):
        return (self.signals[0].duty_cycle - (self.signals[1].duty_cycle if len(self.signals) > 1 else 0)) / 0xffffff * 100.0

    # let val be the percentual target speed in range of [-1,1]
    @value.setter
    def value(self, val):
        # check proper range of variable val
        val = max(-100, min(100, round(val * 100)))
        # going forward
        if val > 0:
            self.signals[0].duty_cycle = val * 0xffffff
            if len(self.signals) > 1:
                self.signals[1].duty_cycle = 0
        # going backward
        elif val < 0:
            self.signals[0].duty_cycle = 0
            if len(self.signals) > 1:
                self.signals[1].duty_cycle = (val * -1) * 0xffffff
        # otherwise stop
        else:
            self.signals[0].duty_cycle = 0
            if len(self.signals) > 1:
                self.signals[1].duty_cycle = 0

    # destructor to disable GPIO.PWM operation
    def __del__(self):
        super(BiMotor, self).__del__()
        for signal in self.signals:
            signal.deinit()
# end BiMotor child class


class PhasedMotor(Solonoid):
    """This class is meant be used for motors driver by driver boards that expect 1 PWM output (to control the motor's speed) and 1 digital output (to control the motor's rotational direction). Each pin represent the controlling signal for the motor's speed in a single rotational direction.

    :param list, tuple pins: A list of pins connected to the motor driver. If less than 2 pins are passed, then a `ValueError` is raised.
    :param int ramp_time: The time (in milliseconds) that is used to smooth the motor's input. Default is 500. This time represents the maximum amount of time that the input will be smoothed. Since the change in speed is also used to determine how much time will be used to smooth the input, this parameter's value will represent the time it takes for the motor to go from full reverse to full forward and vice versa. If the motor is going from rest to either full reverse or full forward, then the time it takes to do that will be half of this parameter's value. This can be changed at any time by changing the `ramp_time` attribute

    """
    def __init__(self, pins, rampTime=1000):
        super(PhasedMotor, self).__init__(pins, rampTime)
        self.signals.clear()
        self.signals.append(pulseio.PWMOut(pins[0]))
        if len(pins) >= 1:
            # save direction signal pin # & set coresponding signal value to true
            self.signals.append(digitalio.DigitalInOut(pins[1]))
            self.signals[1].switch_to_output(value=True)

    @property
    def value(self):
        if len(self.signals) > 1:
            return float(self.signals[0]) / 0xffffff * 100.0 * (1 if bool(self.signals[1].value) else -1)
        else:
            return float(self.signals[0]) / 0xffffff * 100.0

    @value.setter
    def value(self, val):
        # let val be the percentual target speed (in range of -100 to 100)
        # check proper range of variable val
        val = max(-100, min(100, round(val * 100)))
        # going forward
        if val > 0:
            self.signals[0].duty_cycle = val * 0xffffff
            if len(self.signals) > 1:
                self.signals[1].value = True
        # going backward
        elif val < 0:
            self.signals[0].duty_cycle = (val * -1) * 0xffffff
            if len(self.signals) > 1:
                self.signals[1].value = False
        # otherwise stop
        else:
            self.signals[0].duty_cycle = 0
            if len(self.signals) > 1:
                self.signals[1] = True

    # destructor to disable GPIO.PWM operation
    def __del__(self):
        super(PhasedMotor, self).__del__()
        for signal in self.signals:
            signal.deinit()
# end PhasedMotor child class

# pylint: disable=invalid-name
if __name__ == "__main__":
    motor = BiMotor([18, 17])
    motor.value = 0
    motor.cellerate(1.0)
    time.sleep(1)
    print(motor.value)
    motor.cellerate(-1.0)
    time.sleep(2)
    print(motor.value)
    motor.cellerate(0.0)
    time.sleep(1)
    print(motor.value)
