"""A helper module that allows manipulating the Roboclaw device like a
`BiMotor` object compatible with `MotorPool` API"""
# pylint: disable=too-many-function-args
from .smoothing_input import SmoothMotor
from .motors import MotorPool

class RoboclawChannels(SmoothMotor):
    """A class to use one motor's set of terminals on a Roboclaw device.

    :param ~roboclaw.roboclaw.Roboclaw rc_bus: the main UART serial bus to be used as a default
        means of communicating to the Roboclaw(s). You need only instantiate the object for
        this parameter once if all roboclaws are attached to the same port.
        If using multiple USB ports, this parameter holds the object instantiated on
        only 1 port with the :py:attr:`~roboclaw.Roboclaw.address` attribute configured accrdingly.
    :param list address: A `list` of `int` addresses for each individual Roboclaw on the same
        UART serial bus. Address options are limited to range of [``0x80``, ``0x87``] and defaults
        to ``0x80`` if not specified. This is required & should not be changed dynamically.
    :param bool channel: The specific motor being controled on the Roboclaw device. `False`
        means Motor 1; `True` means Motor 2. This is required & should not be changed dynamically.
    :param int value: The initial duty cycle value to start the motor with. Defaults to ``0``. See
        also the `value` attribute.
    :param int ramp_time: The maximum amount of time (in milliseconds) used to smooth the input
        values. A negative value will be used as a positive number. Set this to ``0`` to
        disable all smoothing on the motor input values or just set the
        `value` attribute directly to bypass the smoothing algorithm.

        .. note:: Since the change in speed (target - initial) is also used to determine how much
            time will be used to smooth the input, this attribute's value will represent the
            maximum time it takes for the motor to go from full reverse to full forward and vice
            versa. If the motor is going from rest to either full reverse or full forward, then
            the time it takes to do that will be half of this attribute's value.
    """
    def __init__(self, rc_bus, address, channel, value=0, ramp_time=0):
        self._rc_bus = rc_bus
        self._value = value
        self.address = address
        self._channel = channel
        super(RoboclawChannels, self).__init__(ramp_time=ramp_time)

    @property
    def value(self):
        """This attribute contains the current output value of the Roboclaw Motor in range
        [-65535, 65535]. An invalid input value will be clamped to an `int` in the proper range.
        A negative value represents the motor's speed in reverse rotation. A positive value
        reprsents the motor's speed in forward rotation."""
        return self._value

    @value.setter
    def value(self, val):
        self._value = min(65535, max(-65535, int(val)))
        if self._channel is not None:
            if self._channel:
                self._rc_bus.duty_m1(int(self._value / 2))
            else:
                self._rc_bus.duty_m2(int(self._value / 2))

class RoboclawMotorPool(MotorPool):
    """A wrapper to `MotorPool` to intervept 2 motor commands and write them
    simultaneously instead of writing each command individually."""
    def __init__(self, rc_bus, address=None, ramp_time=0):
        super(RoboclawMotorPool, self).__init__()
        for addr in address:
            self._motors.append(RoboclawChannels(None, addr, None, ramp_time=ramp_time))
        self._rc_bus = rc_bus

    def go(self, cmds, smooth=None):
        """This function takes the input commands and passes them to the motors.

        :param list,tuple cmds: A `list` or `tuple` of values to be passed to the motors.
        :param bool smooth: This controls the motors' built-in algorithm that smooths input values
            over a period of time (in milliseconds) contained in the motors'
            :attr:`~drivetrain.smoothing_input.SmoothMotor.ramp_time` attribute. This can be
            disabled per motor by setting the
            :attr:`~drivetrain.smoothing_input.SmoothMotor.ramp_time` attribute to ``0``, thus
            the smoothing algorithm is automatically bypassed despite this parameter's value.
        """
        for i, cmd in enumerate(cmds):
            if i < len(self._motors):
                if smooth: # if input is getting smoothed
                    self._motors[i].cellerate(cmd)
                else:
                    self._motors[i].value = cmd
            if i % 2:
                self._rc_bus.duty_m1_m2(
                    int(cmds[i - 1] / 2),
                    int(cmd / 2),
                    address=self._motors[i].address)

    def sync(self):
        super().sync()
        for i in range(0, self._motors, 2):
            self._rc_bus.duty_m1_m2(
                int(self._motors[i].value / 2),
                int(self._motors[1 + i].value / 2),
                address=self._motors[i].address)
