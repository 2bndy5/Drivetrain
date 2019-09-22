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
    """This base class is meant be used as a parent to `BiMotor` and `PhasedMotor` classes of this
    module, but can be used for solonoids if needed. Solonoids, by nature, cannot be controlled
    analogously (cannot be any value other than `True` or `False`). Despite the fact that this class
    holds all the smoothing input algorithms for its child classes, the output values, when
    instantiated indepedently, are not actually smoothed. With that said, this class can be used to
    control up to 2 solonoids (see also `value` attribute for more details) as in the case of an
    actual locomotive train.

    :param list,tuple pins: A `list` or `tuple` of (`board` module's) pins that are used to drive the
        to the solonoid(s). The length of this `list`/`tuple` must be in range [1, 2] (any
        additional items/pins will be ignored).

    :param int ramp_time: This parameter is really a placeholder for the child classes
        `BiMotor` & `PhasedMotor` as it has no affect on objects instantiated with this base class.
        Changing this value has not been tested and will probably only slightly delay the
        solonoid(s) outputs.

    """
    def __init__(self, pins, ramp_time=0):
        if not pins:
            raise ValueError('The number of pins used must be at least 1')
        self._signals = []
        for i, pin in enumerate(pins):
            if i >= 2:
                break
            self._signals.append(digitalio.DigitalInOut(pin))
            self._signals[i].switch_to_output(False)
        # variables used to track acceleration
        self._init_speed = 0
        self._target_speed = 0
        self._start_smooth = 0
        self._end_smooth = 0
        self._smoothing_thread = None
        self._cancel_thread = False
        self._dt = ramp_time

    @property
    def is_cellerating(self):
        """This attribute contains a `bool` indicating if the motor's speed is in the midst of
            changing. (read-only)

        """
        if self._smoothing_thread is not None and self._target_speed != self.value:
            return True
        return False

    @property
    def ramp_time(self):
        """This attribute is the maximum amount of time (in milliseconds) used to smooth the input
        values. A negative value will be used as a positive number. Set this to ``0`` to
        disable all smoothing on the motor input values or just set the `value` atribute
        directly to bypass the smoothing algorithm.

        .. note:: Since the change in speed (target - initial) is also used to determine how much
            time will be used to smooth the input, this parameter's value will represent the maximum
            time it takes for the motor to go from full reverse to full forward and vice versa. If
            the motor is going from rest to either full reverse or full forward, then the time it
            takes to do that will be half of this parameter's value.

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
        delta_time = int(time.monotonic() * 1000) - self._start_smooth
        while delta_time < (self._end_smooth - self._start_smooth) and self._cancel_thread:
            delta_speed = (1 - cos(delta_time / float(self._end_smooth - self._start_smooth) * PI)) / 2
            self.value = (delta_speed * (self._target_speed - self._init_speed) + self._init_speed)
            delta_time = int(time.monotonic() * 1000) - self._start_smooth
        self.value = self._target_speed

    # let target_speed be the percentual target speed [-1,1]
    def cellerate(self, target_speed):
        """A function to smoothly accelerate/decelerate the motor to a specified target speed.

        :param int target_speed: The desired target speed in range of [-65535, 65535]. Any invalid
            inputs will be clamped to an `int` value in the proper range.

        """
        self._target_speed = max(-65535, min(65535, int(target_speed)))
        # integer of milliseconds
        self._start_smooth = int(time.monotonic() * 1000)
        self._init_speed = self.value
        delta_t = abs((self._target_speed - self._init_speed) / 131070)
        self._end_smooth = self._start_smooth + delta_t * self._dt
        self._cancel_thread = False
        self._stop_thread()
        self._smoothing_thread = Thread(target=self._smooth)
        self._cancel_thread = True
        self._smoothing_thread.start()

    @property
    def value(self):
        """This attribute contains the current output value of the solonoid(s) in range
        [-1, 1]. An invalid input value will be clamped to an `int` in the proper range.

        .. note:: Because this class is built to handle 2 pins (passed in the ``pins`` parameter
            to the constructor) and tailored for solonoids, any negative value will only energize
            the solonoid driven by the second pin . Any positive value will only energize the
            solonoid driven by the first pin. Alternatively, a ``0`` value will de-energize both
            solonoids.

        """
        return self._signals[0].value or self._signals[1].value if len(self._signals) > 1 else self._signals[0].value

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
        self._cancel_thread = False
        if self._smoothing_thread is not None:
            del self._smoothing_thread
        for signal in self._signals:
            signal.deinit()
        self._signals.clear()
# end Solonoid parent class


class BiMotor(Solonoid):
    """This class is meant be used for motors driver by driver boards that expect 2 PWM outputs.
    Each pin represent the controlling signal for the motor's speed in a single rotational direction.

    :param list,tuple pins: A `list` or `tuple` of (`board` module's) pins that are used to drive the
        motor. The length of this `list` or `tuple` must be in range [1, 2]; any additional
        items/pins will be ignored, and a `ValueError` exception is thrown if no pins are passed
        (an empty `tuple`/`list`). If only 1 pin is passed then the, motor will only rotate in 1
        direction depending on how the motor is connected to the motor driver.

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
        for signal in self._signals:
            signal.deinit()
        self._signals.clear()
        for pin in pins:
            if len(self._signals) == 2:
                break
            self._signals.append(pulseio.PWMOut(pin))

    @property
    def value(self):
        """This attribute contains the current output value of the solonoid(s) in range
        [-65535, 65535]. An invalid input value will be clamped to an `int` in the proper range.
        A negative value represents the motor's speed in reverse rotation. A positive value
        reprsents the motor's speed in forward rotation."""
        return self._signals[0].duty_cycle - (self._signals[1].duty_cycle if len(self._signals) > 1 else 0)

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


class PhasedMotor(Solonoid):
    """This class is meant be used for motors driven by driver boards that expect:

        * 1 PWM output (to control the motor's speed)
        * 1 digital output (to control the motor's rotational direction)

    :param list,tuple pins: A `list` or `tuple` of (`board` module's) pins that are used to drive the
        to the motor. The length of this `list`/`tuple` must be 2, otherwise a `ValueError`
        exception is thrown.

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
        self._signals[0].deinit()
        self._signals[1].deinit()
        self._signals.clear()
        self._signals.append(pulseio.PWMOut(pins[0]))
        if len(pins) >= 1:
            # save direction signal pin # & set coresponding signal value to true
            self._signals.append(digitalio.DigitalInOut(pins[1]))
            self._signals[1].switch_to_output(value=True)

    @property
    def value(self):
        """This attribute contains the current output value of the solonoid(s) in range
        [-65535, 65535]. An invalid input value will be clamped to an `int` in the proper range.
        A negative value represents the motor's speed in reverse rotation. A positive value
        reprsents the motor's speed in forward rotation."""
        return self._signals[0].duty_cycle * (1 if self._signals[1].value else -1)

    @value.setter
    def value(self, val):
        val = max(-65535, min(65535, int(val)))
        # going forward
        if val > 0:
            self._signals[0].duty_cycle = val
            if len(self._signals) > 1:
                self._signals[1].value = True
        # going backward
        elif val < 0:
            self._signals[0].duty_cycle = (val * -1)
            if len(self._signals) > 1:
                self._signals[1].value = False
        # otherwise stop
        else:
            self._signals[0].duty_cycle = 0
            if len(self._signals) > 1:
                self._signals[1] = True
# end PhasedMotor child class

class NRF24L01():
    """This class acts as a wrapper for circuitpython-nrf24l01 module to remotely control a
    peripheral device using nRF24L01 radio transceivers.

    :param ~busio.SPI spi: The object of the SPI bus used to connect to the nRF24L01 transceiver.

        .. note:: This object should be shared among other driver classes that connect to different
            devices on the same SPI bus (SCK, MOSI, & MISO pins)

    :param string address: This will be the RF address used to transmit to the receiving nRF24L01
        transceiver. For more information on this parameters usage, please read the documentation
        on the using the `circuitpython-nrf24l01 library
        <https://circuitpython-nrf24l01.rtfd.io/en/latest/api.html#circuitpython_nrf24l01.RF24.open_rx_pipe>`_

    """
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
        """Assembles a bytearray to be used for transmitting commands over the air.

        :param bytearray cmd: A bytearray of exactly 2 bytes in length. Future development
            may lift this restriction, but currently only a bytearray of 1 byte per axis (eg. x &
            y) is expected.

        """
        temp = struct.pack('bb', cmd[0], cmd[1])
        print('transmit', repr(cmd), 'returned:', self._rf.send(temp))


class USB():
    """
    This class acts as a wrapper to pyserial module for communicating to an external USB serial
        device. Specifically designed for an Arduino running custom code.

    :param string address: The serial port address of the external serial device.
    :param int baud: The specific baudrate to be used for the serial connection. If left
        unspecified, the serial will use a baudrate of 9600.

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
        """Assembles an encoded bytearray for outputting on the serial connection. The ``cmd``
            parameter can be any datatype you wish as it gets treated like a string using the
            builtin datatype's ``__repr__()``. If you use a custom datatype, make sure the datatype
            has a ``__repr__()`` override to return the datatype as a ``print()`` friendly string.

        """
        command = ' '
        for c in cmd:
            command += repr(c) + ' '
        command = bytes(command.encode('utf-8'))
        self._ser.write(command)
