"""PWMOut class as an alternative to circuitpython's pulseio.PWMOut due to lack of support on the
Raspberry Pi and the Jetson""" 
# pylint: disable=import-error
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
# GPIO.setwarnings(False) # unadvised due to thread priority

# example code
# GPIO.setup(12, GPIO.OUT)
# p = GPIO.PWM(12, 50)  # channel=12 frequency=50Hz
# p.start(0) # init duty_cycle of 0 (max of 100)
# p.ChangeFrequency(500)
# for dc in range(0, 101): # fade LED from 0 to 100% duty cycle
#     p.ChangeDutyCycle(dc)
#     time.sleep(0.1)
# p.stop()
# GPIO.cleanup(12) # this should be done on entry & exit of programs

class PWMOut:
    """A wrapper class to substitute the RPi.GPIO.PWM for pulseio.PWMOut

    :param ~microcontoller.Pin pin: The `board` module's pin to be used for PWM output
    :param int freq: The frewquency (in Hz) of the PWM output. Defaults to 500 Hz.
    :param int duty_cycle: The initial duty cycle of the PWM output. Ranges [0, 65535]
    """
    def __init__(self, pin, freq=500, duty_cycle=0):
        self._pin_number = int(repr(pin))
        GPIO.cleanup(self._pin_number) # make sure to deinit pin from any previous unresolved usage
        GPIO.setup(self._pin_number, GPIO.OUT)
        self._frequency = int(freq)
        self._pin = GPIO.PWM(self._pin_number, self._frequency)
        self._duty_cycle = int(duty_cycle)
        self._pin.start(self._duty_cycle * 655.35)

    @property
    def duty_cycle(self):
        """The `int` value in range [0, 65535] of the pin's current PWM duty cycle."""
        return self._duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, val):
        val = max(0, min(65535, int(val)))
        self._duty_cycle = val
        self._pin.ChangeDutyCycle(val * 655.35)

    @property
    def frequency(self):
        """The `int` value of the pin's current (approximated) PWM frequency."""
        return self._frequency
    
    @frequency.setter
    def frequency(self, val):
        self._frequency = val
        self._pin.ChangeFrequency(val)
    
    def deinit(self):
        self._pin.stop()
        GPIO.cleanup(self._pin_number) # make sure to deinit pin from any previous unresolved usage

    def __del__(self):
        self.deinit()
        del self._pin, self._pin_number, self._frequency, self._duty_cycle