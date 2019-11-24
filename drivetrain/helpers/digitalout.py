""" a wrapper for micropython's machine.Pin object """
# pylint: disable=import-error
from microcontroller

class DigitalInOut:
    """A class to control micropython's machine.Pin object like
    a circuitpython DigitalInOut object

    :param ~microcontroller.Pin pin: the digital pin alias.
    """
    def __init__(self, pin):
        self._pin = machine.Pin(pin, machine.Pin.IN)

    def __deinit__(self):
        self._pin.deinit()

    def switch_to_output(self, value=False):
        """ change pin into output """
        self._pin.init(machine.Pin.OUT, value=value)

    def switch_to_input(self):
        """ change pin into input """
        self._pin.init(machine.Pin.IN)

    @property
    def value(self):
        """ the value of the pin """
        return self._pin.value()

    @value.setter
    def value(self, val):
        self._pin.value(val)

    def __del__(self):
        self._pin.deinit()
        del self._pin
