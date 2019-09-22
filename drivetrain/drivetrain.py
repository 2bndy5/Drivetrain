"""

Drivetrain Configuration Classes
================================

This module contains the necessary algorithms for utilizing different DC motor types in different
configurations for the raspberry pi. Currently only supporting the R2D2 (aliased here as `BiPed`)
& typical openRC (aliased here as `QuadPed`) configurations.

"""
# pylint: disable=arguments-differ,invalid-name

# from gpiozero import AngularServo
from .stepper import StepperMotor
from .motor import BiMotor, PhasedMotor, NRF24L01, USB


class _Drivetrain:
    """A base class that is only used for inheriting various types of drivetrain configurations."""
    def __init__(self, motors, max_speed):
        for i, m in enumerate(motors):
            if not type(m, (BiMotor, PhasedMotor, StepperMotor)):
                raise ValueError('unknown motor (index {}) of type {}'.format(i, type(m)))
        if not motors:
            raise ValueError('No motors were passed to the drivetrain.')
        self._motors = motors
        self._max_speed = max(0, min(max_speed if max_speed is not None else 0, 100))

    def _gogo(self, aux, init=2):
        """A helper function to the child classes to handle extra periphial motors attached to the
        `Drivetrain` object. This is only useful for motors that serve a specialized purpose
        other than propulsion.

        :param list,tuple aux: A list or tuple of `int` motor input values to be passed in
            corresponding order to the motors.

        :param int init: The index of the `list`/`tuple` of motor commends from which to start
            passing values to the corresponding motors.

        """
        if len(aux) > init:
            for i in range(init, len(aux)):
                if i < len(self._motors):
                    # print('motor[', i, '].value = ', aux[i] / 100.0, sep = '')
                    self._motors[i].value = aux[i] / 100.0
                else:
                    print(f'motor[{i}] not declared and/or installed')
    @property
    def is_cellerating(self):
        """This attribute contains a `bool` indicating if the drivetrain's motors' speed is in the
        midst of changing. (read-only)"""
        for m in self._motors:
            if m.is_cellerating:
                return True
        return False

    @property
    def max_speed(self):
        """This attribute determines a motor's top speed. Valid input values range [0,100]."""
        return self._max_speed

    @max_speed.setter
    def max_speed(self, val):
        self._max_speed = max(0, min(val if val is not None else 0, 100))

    def print(self):
        """Prints all the motors current values."""
        for i, m in self._motors:
            print(f'motor {i} value = {m.value}')

    def __del__(self):
        self._motors.clear()
        del self._motors
# end Drivetrain class

class BiPed(_Drivetrain):
    """A Drivetrain class meant to be used for motor configurations where propulsion and steering
    are shared tasks. For example: R2D2 has 2 motors (1 in each leg -- the front retractable
    wheel is free pivoting) for propulsion and steering.

    :param list motors: A `list` of motors that are to be controlled in concert. Each item in this
        `list` represents a single motor must be of type `Solonoid`, `BiMotor`, `PhasedMotor`, or
        `StepperMotor`. The first 2 motors in this `list` are used to propell and steer
        respectively.

    :param int max_speed: The maximum speed as a percentage in range [0,100] for the drivetrain's
        forward and backward motion. Defaults to 85%. This does not scale the motor speed's range,
        it just limits the top speed that the forward/backward motion can go.

    """
    def __init__(self, motors, max_speed=85):
        if len(motors) != 2:
            raise ValueError('The drivetrain requires 2 motors to operate.')
        super(BiPed, self).__init__(motors, max_speed)

    def go(self, cmds, smooth=True):
        """This function applies the user input to the motors' output according to drivetrain's motor
        configuration stated in the contructor documentation.

        :param list,tuple cmds: A `list` or `tuple` of input motor commands to be processed and
            passed to the motors. This list must have at least 2 items (input values), and any
            additional items will be ignored. A `list`/`tuple` of length less than 2 will throw a
            `ValueError` exception.

            .. important:: Ordering of the motor inputs contained in this list/tuple matters. They
                should correspond to the following order:

                1. forward/reverse magnitude in range [-100,100]
                2. left/right magnitude in range [-100,100]

        :param bool smooth: This controls the motors' built-in algorithm that smooths input values
            over a period of time (in milliseconds) contained in the motors'
            :attr:`~drivetrain.motor.BiMotor.ramp_time` attribute. This defaults to `True`.
            Optionally, if the :attr:`~drivetrain.motor.BiMotor.ramp_time` attribute is set to
            ``0`` then, the smoothing algorithm is automatically bypassed despite this parameter's
            value.

            .. note:: Assert this parameter (set as `True`) for robots with a rather high center of
                gravity or if some parts are poorly attached. The absence of properly smoothed
                acceleration/deceleration will likely make the robot fall over or loose parts
                become dislodged on sudden and drastic changes in speed.

        """
        if len(cmds) < 2:
            raise ValueError("the list/tuple of commands must be at least 2 items long")
        cmds[0] = max(-100, min(100, int(cmds[0])))
        cmds[1] = max(-100, min(100, int(cmds[1])))
        if cmds[1] > self._max_speed:
            cmds[1] = self._max_speed
        # assuming left/right axis is null (just going forward or backward)
        left = cmds[1]
        right = cmds[1]
        if abs(cmds[0]) == 100:
            # if forward/backward axis is null ("turning on a dime" functionality)
            if cmds[0] > self._max_speed:
                cmds[0] = self._max_speed
            right = cmds[0]
            left = cmds[0] * -1
        else:
            # if forward/backward axis is not null and left/right axis is not null
            offset = (100 - abs(cmds[0])) / 100.0
            if cmds[0] > 0:
                right *= offset
            elif cmds[0] < 0:
                left *= offset
        if smooth:
            self._motors[0].cellerate(left * 655.35)
            self._motors[1].cellerate(right * 655.35)
        else:
            self._motors[0].value = left * 655.35
            self._motors[1].value = right * 655.35
        self._gogo(cmds)
# end BiPed class


class QuadPed(_Drivetrain):
    """A Drivetrain class meant to be used for motor configurations where propulsion and steering
    are separate tasks. The first motor is used to steer, and the second motor is used to
    propell.

    :param list motors: A `list` of motors that are to be controlled in concert. Each item in this
        `list` represents a single motor and must be of type `Solonoid`, `BiMotor`, `PhasedMotor`,
        or `StepperMotor`. The first 2 motors in this `list` are used to propell and steer
        respectively.

    :param int max_speed: The maximum speed as a percentage in range [0,100] for the drivetrain's
        forward and backward motion. Defaults to 85%. This does not scale the motor speed's range,
        it just limits the top speed that the forward/backward motion can go.

    """
    def __init__(self, motors, max_speed=85):
        if len(motors) != 2:
            raise ValueError('The drivetrain requires 2 motors to operate.')
        super(QuadPed, self).__init__(motors, max_speed)

    def go(self, cmds, smooth=True):
        """This function applies the user input to motor output according to drivetrain's motor
        configuration.

        :param list,tuple cmds: A `list` or `tuple` of input motor commands to be passed to the
            motors. This `list` / `tuple` must have at least 2 items (input values), and any
            additional item(s) will be ignored. A `list`/`tuple` of length less than 2 will throw a
            `ValueError` exception.

            .. important:: Ordering of the motor inputs contained in this list/tuple matters. They
                should correspond to the following order:

                1. forward/reverse magnitude in range [-100,100]
                2. left/right magnitude in range [-100,100]

        :param bool smooth: This controls the motors' built-in algorithm that smooths input values
            over a period of time (in milliseconds) contained in the motors'
            :attr:`~drivetrain.motor.BiMotor.ramp_time` attribute. This defaults to `True`.
            Optionally, if the :attr:`~drivetrain.motor.BiMotor.ramp_time` attribute is set to
            ``0`` then, the smoothing algorithm is automatically bypassed despite this parameter's
            value.

            .. note:: Assert this parameter (set as `True`) for robots with a rather high center of
                gravity or if some parts are poorly attached. The absence of properly smoothed
                acceleration/deceleration will likely make the robot fall over or loose parts
                become dislodged on sudden and drastic changes in speed.

        """
        # make sure arguments are in their proper range
        if len(cmds) < 2:
            raise ValueError("the list/tuple of commands must be at least 2 items long")
        # make sure speeds are an integer (not decimal/float)
        cmds[0] = max(-100, min(100, int(cmds[0])))
        cmds[1] = max(-100, min(100, int(cmds[1])))
        if cmds[1] > self._max_speed:
            cmds[1] = self._max_speed
        if smooth:
            self._motors[0].cellerate(cmds[0] * 655.35)
            self._motors[1].cellerate(cmds[1] * 655.35)
        else:
            self._motors[0].value = cmds[0] * 655.35
            self._motors[1].value = cmds[1] * 655.35
        self._gogo(cmds)
# end QuadPed class

class External:
    """A class to be used for controlling drivetrains not physically attached to the host
    controller. Currently only supports USB (Serial) and nRF24L01 (an spi based radio transceiver)
    as interfaces.

    :param ~motor.USB,~motor.NRF24L01 interface: The specialized interface type used to remotely
        control the external drivetrain's motors.

    """
    def __init__(self, interface):
        if type(interface, (USB, NRF24L01)):
            self._interface = interface
        else:
            raise ValueError('The "External" drivetrain class only supports interfaces of type class USB or NRF24L01')
        self._last_cmds = []

    def go(self, cmds):
        """pass motor control commands to the appropriate interface (specified upon instantiation)."""
        self._interface.go(cmds)
