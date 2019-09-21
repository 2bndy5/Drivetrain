"""
================================
Solonoid, BiMotor, & PhasedMotor
================================

A collection of driver classes for different single phase DC motors implementing the threading module. Includes Solonoid (parent base class), BiMotor & PhasedMotor (children of Solonoid)

"""
from math import pi as PI, cos
from threading import Thread
import struct
import time
import digitalio
import pulseio
import serial
from busio import SPI
from circuitpython_nrf24l01 import RF24

# pylint: disable=invalid-name,too-many-instance-attributes

class Solonoid:
    """This base class is meant be used as a parent to `BiMotor` and `PhasedMotor` classes of this module, but can be used for solonoids if needed. Solonoids, by nature, cannot be controlled analogously (cannot be any value other than 0 or 100). Despite the fact that this class holds all the smoothing input algorithms for its child classes, the output values are not actually smoothed. With that said, this class can be used to control up to 2 solonoids (see also `value` attribute for more details).

    :param list, tuple pins: A list of pins connected to the solonoid(s).
    :param int ramp_time: This parameter is really a placeholder for the child classes BiMotor & PhasedMotor as it has no affect on objects instantiated with this base class.

    """
    def __init__(self, pins, ramp_time=100):
        self._signals = []
        for i, pin in enumerate(pins):
            # save pwm duty cycles as int [0,100]; direction signals will be treated like bool [0|1]
            self._signals.append(digitalio.DigitalInOut(pin))
            self._signals[i].switch_to_output(False)
        # variables used to track acceleration
        self._init_speed = 0
        self._final_speed = 0
        self._start_smooth = 0
        self._end_smooth = 0
        self._smoothing_thread = None
        self._cancel_thread = False
        # time in milliseconds to change/ramp speed from self.value to self._final_speed
        self._dt = ramp_time

    @property
    def ramp_time(self):
        """This attribute is the maximum amount of time (in milliseconds) used to smooth the input values. A negative value will be used as a positive number. Set this to ``0`` to disable all smoothing on the motor input values or just set the `value` atribute directly to bypass the smoothing algorithm.

        .. note:: Since the change in speed is also used to determine how much time will be used to smooth the input, this parameter's value will represent the time it takes for the motor to go from full reverse to full forward and vice versa. If the motor is going from rest to either full reverse or full forward, then the time it takes to do that will be half of this parameter's value.

        """
        return self._dt

    @ramp_time.setter
    def ramp_time(self, delta_t):
        self._dt = abs(delta_t)


    def _stop_thread(self):
        if self._smoothing_thread is not None:
            self._smoothing_thread.join()
        self._smoothing_thread = None

    def _smooth(self):
        """
        delta_speed, self._final_speed, and self._init_speed are all percentage [-1,1]
        delta_time & dt is in microseconds
        """
        delta_time = int(time.monotonic() * 1000) - self._start_smooth
        while delta_time < (self._end_smooth - self._start_smooth) and self._cancel_thread:
            delta_speed = (1 - cos(delta_time / float(self._end_smooth - self._start_smooth) * PI)) / 2
            self.value = (delta_speed * (self._final_speed - self._init_speed) + self._init_speed) / 100.0
            delta_time = int(time.monotonic() * 1000) - self._start_smooth
        self.value = self._final_speed / 100.0

    # let final_speed be the percentual target speed [-1,1]
    def cellerate(self, final_speed):
        """
        let final_speed = target speed in range of [-1,1]
        let delta_t = percent [0,1] of delta time (self._dt in milliseconds)
        """
        self._final_speed = max(-100, min(100, round(final_speed * 100)))  # bounds check
        # integer of milliseconds
        self._start_smooth = int(time.monotonic() * 1000)
        self._init_speed = int(self.value * 100)
        delta_t = abs((self._final_speed - self._init_speed) / 200.0)
        self._end_smooth = self._start_smooth + delta_t * self._dt
        self._cancel_thread = False
        self._stop_thread()
        self._smoothing_thread = Thread(target=self._smooth)
        self._cancel_thread = True
        self._smoothing_thread.start()

    @property
    def value(self):
        """This attribute will always return the current output value(s) of the solonoid(s) as a percentage in range [-100,100]. An invalid input value will be clamped to range [-100,100]

        .. note:: Because this class is built to handle 2 pins and tailored for solonoids, any negative value will only energize the solonoid attached to the second pin (passed in ``pins`` parameter to the constructor). Any positive value will only energize the solonoid attached to the first pin (passed in ``pins`` parameter to the constructor). Alternatively, a zero value will de-energize both solonoids

        """
        return self._signals[0].duty_cycle or self._signals[1].duty_cycle if len(self._signals) > 1 else self._signals[0].duty_cycle

    @value.setter
    def value(self, val):
        # check proper range of variable val [-1,1]
        val = max(-100, min(100, round(val * 100)))
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
        self._cancel_thread = False
        if self._smoothing_thread is not None:
            del self._smoothing_thread
# end Solonoid parent class


class BiMotor(Solonoid):
    """This class is meant be used for motors driver by driver boards that expect 2 PWM outputs. Each pin represent the controlling signal for the motor's speed in a single rotational direction.

    :param list,tuple pins: A list of pins connected to the motor driver. If only one pins is passed than the motor will only turn in one direction.
    :param int ramp_time: The time (in milliseconds) that is used to smooth the motor's input. Default is 500. This time represents the maximum amount of time that the input will be smoothed. Since the change in speed is also used to determine how much time will be used to smooth the input, this parameter's value will represent the time it takes for the motor to go from full reverse to full forward and vice versa. If the motor is going from rest to either full reverse or full forward, then the time it takes to do that will be half of this parameter's value. This can be changed at any time by changing the `ramp_time` attribute

    """
    def __init__(self, pins, rampTime=1000):
        super(BiMotor, self).__init__(pins, rampTime)
        # save pin numbers as GPIO.PWM objects
        self._signals.clear()
        for pin in pins:
            self._signals.append(pulseio.PWMOut(pin))

    @property
    def value(self):
        return (self._signals[0].duty_cycle - (self._signals[1].duty_cycle if len(self._signals) > 1 else 0)) / 0xffffff * 100.0

    # let val be the percentual target speed in range of [-1,1]
    @value.setter
    def value(self, val):
        # check proper range of variable val
        val = max(-100, min(100, round(val * 100)))
        # going forward
        if val > 0:
            self._signals[0].duty_cycle = val * 0xffffff
            if len(self._signals) > 1:
                self._signals[1].duty_cycle = 0
        # going backward
        elif val < 0:
            self._signals[0].duty_cycle = 0
            if len(self._signals) > 1:
                self._signals[1].duty_cycle = (val * -1) * 0xffffff
        # otherwise stop
        else:
            self._signals[0].duty_cycle = 0
            if len(self._signals) > 1:
                self._signals[1].duty_cycle = 0

    # destructor to disable GPIO.PWM operation
    def __del__(self):
        super(BiMotor, self).__del__()
        for signal in self._signals:
            signal.deinit()
# end BiMotor child class


class PhasedMotor(Solonoid):
    """This class is meant be used for motors driver by driver boards that expect 1 PWM output (to control the motor's speed) and 1 digital output (to control the motor's rotational direction). Each pin represent the controlling signal for the motor's speed in a single rotational direction.

    :param list, tuple pins: A list of pins connected to the motor driver. If less than 2 pins are passed, then a `ValueError` is raised.
    :param int ramp_time: The time (in milliseconds) that is used to smooth the motor's input. Default is 500. This time represents the maximum amount of time that the input will be smoothed. Since the change in speed is also used to determine how much time will be used to smooth the input, this parameter's value will represent the time it takes for the motor to go from full reverse to full forward and vice versa. If the motor is going from rest to either full reverse or full forward, then the time it takes to do that will be half of this parameter's value. This can be changed at any time by changing the `ramp_time` attribute

    """
    def __init__(self, pins, rampTime=1000):
        super(PhasedMotor, self).__init__(pins, rampTime)
        self._signals.clear()
        self._signals.append(pulseio.PWMOut(pins[0]))
        if len(pins) >= 1:
            # save direction signal pin # & set coresponding signal value to true
            self._signals.append(digitalio.DigitalInOut(pins[1]))
            self._signals[1].switch_to_output(value=True)

    @property
    def value(self):
        if len(self._signals) > 1:
            return float(self._signals[0]) / 0xffffff * 100.0 * (1 if bool(self._signals[1].value) else -1)
        else:
            return float(self._signals[0]) / 0xffffff * 100.0

    @value.setter
    def value(self, val):
        # let val be the percentual target speed (in range of -100 to 100)
        # check proper range of variable val
        val = max(-100, min(100, round(val * 100)))
        # going forward
        if val > 0:
            self._signals[0].duty_cycle = val * 0xffffff
            if len(self._signals) > 1:
                self._signals[1].value = True
        # going backward
        elif val < 0:
            self._signals[0].duty_cycle = (val * -1) * 0xffffff
            if len(self._signals) > 1:
                self._signals[1].value = False
        # otherwise stop
        else:
            self._signals[0].duty_cycle = 0
            if len(self._signals) > 1:
                self._signals[1] = True

    # destructor to disable GPIO.PWM operation
    def __del__(self):
        super(PhasedMotor, self).__del__()
        for signal in self._signals:
            signal.deinit()
# end PhasedMotor child class

class NRF24L01():
    """This class acts as a wrapper for circuitpython-nrf24l01 module to remotely control a peripheral device using nRF24L01 radio transceivers"""
    def __init__(self, spi, pins, address=b'rfpi0'):
        if not type(pins, list) and len(pins) != 2:
            raise ValueError('pins parameter must be a list of length 2 (CE and CSN respectively)')
        if not type(spi, SPI):
            raise ValueError('spi parameter must be an object of type busio.SPI')
        pins[0] = digitalio.DigitalInOut(pins[0])
        pins[1] = digitalio.DigitalInOut(pins[1])
        self._rf = RF24(spi, pins[0], pins[1])
        self._rf.open_tx_pipe(address)
        # self._rf.what_happened(1)

    def go(self, cmd):
        """Assembles a bytearray to be used for transmitting commands over the air"""
        temp = struct.pack('bb', cmd[0], cmd[1])
        print('transmit', repr(cmd), 'returned:', self._rf.send(temp))


class USB():
    """
    This class acts as a wrapper to pyserial module for communicating to an external USB serial device. Specifically designed for an Arduino running custom code
    """
    def __init__(self, address='/dev/ttyUSB0', baud=-1):
        try:
            if baud < 0:
                self._ser = serial.Serial(address)
            else:
                self._ser = serial.Serial(address, baud)
            print('Successfully opened port', address, '@', baud, 'to Arduino device')
        except serial.SerialException:
            raise ValueError('unable to open serial arduino device @ port {}'.format(address))

    def go(self, cmd):
        """ assembles a encoded bytearray for outputting on the serial connection"""
        command = ' '
        for c in cmd:
            command += repr(c) + ' '
        command = bytes(command.encode('utf-8'))
        self._ser.write(command)
