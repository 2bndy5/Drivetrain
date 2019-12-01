
.. If you created a package, create one automodule per module in the package.

.. If your library file(s) are nested in a directory (e.g. /adafruit_foo/foo.py)
.. use this format as the module name: "adafruit_foo.foo"

Drivetrain Configurations
============================

.. important:: Other motor libraries have implemented the "DC braking" concept in which all coils of the motor are energized to lock the rotor in place using simultaneously opposing electromagnetic forces. Unlike other motor libraries, we DO NOT assume your motors' driver circuit contains flyback diodes to protect its transistors (even though they are practically required due to `Lenz's Law <https://en.wikipedia.org/wiki/Lenz%27s_law>`_). Therefore, passing a desired speed of ``0`` to any of the ``cellerate()`` or ``go()`` functions of the drivetrain and motor objects will effectively de-energize the coils in the motors.

Smoothing Algorithm
-------------------

.. autoclass:: drivetrain.helpers.smoothing_input.SmoothDrivetrain
    :members:

Tank Drivetrain
----------------

.. autoclass:: drivetrain.drivetrain.Tank
    :members:
    :show-inheritance:

Automotive Drivetrain
---------------------

.. autoclass:: drivetrain.drivetrain.Automotive
    :members:
    :show-inheritance:

Locomotive Drivetrain
---------------------

.. autoclass:: drivetrain.drivetrain.Locomotive
    :members:

Mecanum Drivetrain
---------------------

.. autoclass:: drivetrain.drivetrain.Mecanum
    :members:
    :show-inheritance:

Drivetrain Interfaces
======================

Buffer Mixin
------------

.. autoclass:: drivetrain.helpers.buffer_mixin.BufferMixin
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

.. autoclass:: drivetrain.helpers.smoothing_input.SmoothMotor
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

Aditional Helpers
===============================

DigitalInOut For MicroPython
----------------------------

.. automodule:: drivetrain.helpers.digi_io
    :members:

PWMOut For MicroPython & RPi.GPIO
----------------------------------

.. automodule:: drivetrain.helpers.pwm
    :members:

UART Serial for Non-PySerial supported platforms
--------------------------------------------------

.. automodule:: drivetrain.helpers.usart_serial_ctx
    :members:

Cylical Redundancy Checking
------------------------------

.. automodule:: drivetrain.helpers.data_manip
    :members:
