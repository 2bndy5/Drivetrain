
.. If you created a package, create one automodule per module in the package.

.. If your library file(s) are nested in a directory (e.g. /adafruit_foo/foo.py)
.. use this format as the module name: "adafruit_foo.foo"

Drivetrain Configurations
============================

.. important:: Other motor libraries have implemented the "DC braking" concept in which all coils of the motor are energized to lock the rotor in place using simultaneously opposing electromagnetic forces. Unlike other motor libraries, we DO NOT assume your motors' driver circuit contains flyback diodes to protect its transistors (even though they are practically required due to `Lenz's Law <https://en.wikipedia.org/wiki/Lenz%27s_law>`_). Therefore, passing a desired speed of ``0`` to any of the ``cellerate()`` or ``go()`` functions of the drivetrain and motor objects will effectively de-energize the coils in the motors.

Tank Drivetrain
----------------

.. autoclass:: drivetrain.drivetrain.Tank
    :members:
    :inherited-members:

Automotive Drivetrain
---------------------

.. autoclass:: drivetrain.drivetrain.Automotive
    :members:
    :inherited-members:

Locomotive Drivetrain
---------------------

.. autoclass:: drivetrain.drivetrain.Locomotive
    :members:

Mecanum Drivetrain
---------------------

.. autoclass:: drivetrain.drivetrain.Mecanum
    :members:
    :inherited-members:

Drivetrain Interfaces
======================

Buffer Mixin
------------

.. autoclass:: drivetrain.buffer_mixin.BufferMixin
    :members:

NRF24L01
---------

.. autoclass:: drivetrain.interfaces.NRF24L01
    :members:
    :show-inheritance:

.. autoclass:: drivetrain.interfaces.NRF24L01tx
    :members:
    :show-inheritance:

.. autoclass:: drivetrain.interfaces.NRF24L01rx
    :members:
    :show-inheritance:

USB
-----

.. autoclass:: drivetrain.interfaces.USB
    :members:
    :show-inheritance:

.. autoclass:: drivetrain.interfaces.USBtx
    :members:
    :show-inheritance:

.. autoclass:: drivetrain.interfaces.USBrx
    :members:
    :show-inheritance:


Motor Types
===================

Smoothing Algorithm
-------------------

.. autoclass:: drivetrain.smoothing_input.Smooth
    :members:

Solenoid
----------------

.. autoclass:: drivetrain.motor.Solenoid
    :members: value

BiMotor
----------------

.. autoclass:: drivetrain.motor.BiMotor
    :members:
    :show-inheritance:

PhasedMotor
----------------

.. autoclass:: drivetrain.motor.PhasedMotor
    :members:
    :show-inheritance:

StepperMotor
----------------

.. autoclass:: drivetrain.stepper.StepperMotor
    :members:

Roboclaw Bus object helper class
----------------------------------

.. automodule:: drivetrain.roboclaw_bus
    :members:
    :show-inheritance:

non-CircuitpythonPython Helpers
===============================

DigitalInOut For MicroPython
----------------------------

``from drivetrain.digitaio import DigitalInOut``

PWMOut For MicroPython & RPi.GPIO
----------------------------------

``from drivetrain.pwm import PWMOut``

UART Serial with context manager For MicroPython
-------------------------------------------------

This module contains a wrapper class for MicroPython's :py:class:`~machine.UART` or
CircuitPython's :py:class:`~busio.UART` class to work as a drop-in replacement to
:py:class:`~serial.Serial` object.

.. note:: This helper class does not expose all the pySerial API. It's tailored to this library only. That said, to use this:

    .. code-block:: python

        from drivetrain.usart_serial_ctx import SerialUART as UART
        serial_bus = UART()
        with serial_bus:
            serial_bus.read_until() # readline() with timeout
            serial_bus.in_waiting() # how many bytes in the RX buffer
            serial_bus.close() # same as UART.deinit()
        # exit ``with`` also calls machine.UART.deinit()
