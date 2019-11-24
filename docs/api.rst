
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

NRF24L01
---------

.. autoclass:: drivetrain.interfaces.NRF24L01
    :members:

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

.. autoclass:: drivetrain.interfaces.USBtx
    :members:
    :show-inheritance:

.. autoclass:: drivetrain.interfaces.USBrx
    :members:
    :show-inheritance:

Motor Types
===================

Solenoid
----------------

.. autoclass:: drivetrain.motor.Solenoid
    :members: value

BiMotor
----------------

.. autoclass:: drivetrain.motor.BiMotor
    :members:
    :inherited-members:

PhasedMotor
----------------

.. autoclass:: drivetrain.motor.PhasedMotor
    :members:
    :inherited-members:

StepperMotor
----------------

.. autoclass:: drivetrain.stepper.StepperMotor
    :members:

.. non-CircuitpythonPython Helpers
.. ===============================

.. .. currentmodule:: drivetrain.helpers

.. DigitalInOut For MicroPython
.. ----------------------------

.. .. autoclass:: drivetrain.helpers.digitaio.DigitalInOut
..     :members:

.. PWMOut For MicroPython & RPi.GPIO
.. ----------------------------------

.. .. autoclass:: drivetrain.helpers.pwm.PWMOut
..     :members:

.. UART Serial with context manager For MicroPython
.. -------------------------------------------------

.. .. autoclass:: drivetrain.helpers.usart_serial_ctx.SerialUART
..     :members:

Smoothing Algorithm
-------------------

.. autoclass:: drivetrain.helpers.smoothing_input.Smooth
    :members:
