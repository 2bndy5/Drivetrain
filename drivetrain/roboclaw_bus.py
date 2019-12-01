"""A helper module that allows manipulating the Roboclaw object like a `list` of
:py:class:`~drivetrain.motor.BiMotor` object"""
# pylint: disable=too-many-function-args
from .helpers.smoothing_input import SmoothMotor, SmoothDrivetrain

class _RoboclawMotor(SmoothMotor):
    def __init(self, address, ramp_time, value):
        self._value = value
        self.address = address
        super(_RoboclawMotor, self).__init__(ramp_time=ramp_time)

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

class RoboclawMotorPool(SmoothDrivetrain):
    """This class uses a serial connection and 2 functions from the
    :py:class:`~roboclaw.RoboClaw` object to control all/any connected Roboclaws
    on the same serial UART port.

    :param roboclaw.Roboclaw rc_bus: the main UART serial bus to be used as a default means of
        communicating to the Roboclaw(s). You need only instantiate the object for
        this parameter once if all roboclaws are attached to the same port.
        If using multiple USB ports, this parameter holds the object instantiated on
        that port with the :py:attr:`~roboclaw.Roboclaw.address` attrubte configured accrdingly.
    :param int freq: The frewquency (in Hz) of the PWM output. Defaults to 500 Hz.
    :param int value: The initial duty cycle of the PWM output. Ranges [0, 65535]
    """
    def __init__(self, rc_bus, address=None, ramp_time=2, value=0, max_spped=100, smooth=True):
        if rc_bus is None:
            raise AttributeError('roboclaw has no serial bus designated by'
                                 ' parameter "rc_bus"')
        self._rc_bus = rc_bus
        super(RoboclawMotorPool, self).__init__()
        for addr in address:
            for _ in range(2):
                self._motors.append(_RoboclawMotor(addr, ramp_time, value))

    def __getitem__(self, index):
        assert index in range(len(self._motors) - 1)
        return self._motors[index]

    @property
    def smooth(self):
        """Setting this attribute enables (`True`) or disables (`False`) the input smoothing
        alogrithms for all motors, if applicable, attached to the motorpool object. When read from, this attribute
        returns all the `ramp_time` attributes of the motors attached to the motorpool object."""
        self._smooth = [x.ramp_time for x in self._motors]
        return self._smooth

    @smooth.setter
    def smooth(self, enable):
        self._smooth = enable

    def go(self, cmds, smooth=None):
        """ takes controling output commands and passes them accordingly to the roboclaw(s) """
        if len(cmds) < len(self._motors):
            raise AttributeError("not enough commands for the number of motors")
        for i, motor in enumerate(self._motors):
            smooth = bool(motor.ramp_time)
            if smooth:
                motor.cellerate(min(65535, max(-65535, cmds[i])))
            else:
                motor.value = min(65535, max(-65535, cmds[i]))

            # if (i % 2): # have enough commands to write both at same time
                # self._rc_bus.duty_m1_m2(int(self._motors[i - 1].value / 2), int(motor.value / 2), address=motor.address)

    def stop(self):
        """Stops both channels & de-initialize the serial object on its port for future use."""
        self._rc_bus.duty_m1_m2(0, 0)
        self._rc_bus.serial_obj.close()

    def __del__(self):
        self.stop()
