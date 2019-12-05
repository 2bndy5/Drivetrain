"""A helper module that allows manipulating the Roboclaw object like a
`MotorPool` `list` of :py:class:`~drivetrain.motor.BiMotor` object"""
# pylint: disable=too-many-function-args
from .helpers.smoothing_input import SmoothMotor
from .motor import MotorPool

class RoboclawMotor(SmoothMotor):
    """A class to use one motor 's set of terminals on a Roboclaw object.

    :param roboclaw.Roboclaw rc_bus: the main UART serial bus to be used as a default means of
        communicating to the Roboclaw(s). You need only instantiate the object for
        this parameter once if all roboclaws are attached to the same port.
        If using multiple USB ports, this parameter holds the object instantiated on
        that port with the :py:attr:`~roboclaw.Roboclaw.address` attribute configured accrdingly.
    :param list address: A `list` of `int` addresses for each individual Roboclaw on the same
        UART serisl bus. Address options are limited to range of [``0x80``, ``0x87``] and degaults
        to ``0x80`` if not specified.
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
    def __init__(self, rc_bus, address, channel, value=0, ramp_time=2000):
        self._rc_bus = rc_bus
        self._value = value
        self.address = address
        self._channel = channel
        super(RoboclawMotor, self).__init__(ramp_time=ramp_time)

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
        if self._channel:
            self._rc_bus.duty_m1(int(self._value / 2))
        else:
            self._rc_bus.duty_m2(int(self._value / 2))

class RoboclawMotorPool(MotorPool):
    """This class uses a serial connection and 2 functions from the
    :py:class:`~roboclaw.RoboClaw` object to control all/any connected Roboclaws
    on the same serial UART port.

    :param roboclaw.Roboclaw rc_bus: the main UART serial bus to be used as a default means of
        communicating to the Roboclaw(s). You need only instantiate the object for
        this parameter once if all roboclaws are attached to the same port.
        If using multiple USB ports, this parameter holds the object instantiated on
        that port with the :py:attr:`~roboclaw.Roboclaw.address` attribute configured accrdingly.
    :param list address: A `list` of `int` addresses for each individual Roboclaw on the same
        UART serisl bus. Address options are limited to range of [``0x80``, ``0x87``] and degaults
        to ``0x80`` if not specified.
    :param int ramp_time: The maximum amount of time (in milliseconds) used to smooth the input
        values. A negative value will be used as a positive number. Set this to ``0`` to
        disable all smoothing on the motor input values or just set the
        :attr:`~RoboclawMotor.value` attribute directly to bypass the smoothing algorithm.

        .. note:: Since the change in speed (target - initial) is also used to determine how much
            time will be used to smooth the input, this attribute's value will represent the
            maximum time it takes for the motor to go from full reverse to full forward and vice
            versa. If the motor is going from rest to either full reverse or full forward, then
            the time it takes to do that will be half of this attribute's value.
    """
    def __init__(self, rc_bus, address=None, ramp_time=2000):
        self._rc_bus = rc_bus
        super(RoboclawMotorPool, self).__init__()
        for addr in address:
            for i in range(2):
                self._motors.append(RoboclawMotor(rc_bus, addr, 1 - (i % 2), ramp_time=ramp_time))

    @property
    def smooth(self):
        """Setting this attribute enables (`True`) or disables (`False`) the input smoothing
        alogrithms for all motors, if applicable, attached to the motorpool object."""
        return self._smooth

    @property
    def ramp_times(self):
        """This attribute returns all the `ramp_time` attributes of the motors attached to the
        motorpool object. (read-only)"""
        return [x.ramp_time for x in self._motors]

    @smooth.setter
    def smooth(self, enable):
        self._smooth = enable

    def go(self, cmds, smooth=None):
        """ takes controling output commands and passes them accordingly to the roboclaw(s) """
        if len(cmds) < len(self._motors):
            raise AttributeError("not enough commands for the number of motors")
        for i, motor in enumerate(self._motors):
            smooth = motor.ramp_time if smooth is None else smooth
            if smooth:
                motor.cellerate(min(65535, max(-65535, cmds[i])))
            else:
                motor.value = min(65535, max(-65535, cmds[i]))
            # if (i % 2): # have enough commands to write both at same time
            #     self._rc_bus.duty_m1_m2(
            #         int(self._motors[i - 1].value / 2),
            #         int(motor.value / 2),
            #         address=motor.address)

    def stop(self):
        """Stops both channels & de-initialize the serial object on its port for future use."""
        self._rc_bus.duty_m1_m2(0, 0)
        self._rc_bus.serial_obj.close()

    def __del__(self):
        self.stop()
