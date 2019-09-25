
.. If you created a package, create one automodule per module in the package.

.. If your library file(s) are nested in a directory (e.g. /adafruit_foo/foo.py)
.. use this format as the module name: "adafruit_foo.foo"


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

External Drivetrain
-------------------

.. autoclass:: drivetrain.drivetrain.External
    :members:

Solenoid
----------------

.. autoclass:: drivetrain.motor.Solenoid
    :members: value

BiMotor
----------------

.. important:: Other motor libraries have implemented the "DC braking" concept in which all coils of the motor are energized to lock the rotor in place using simultaneously opposing electromagnetic forces. Unlike other motor libraries, we DO NOT assume your motors' driver circuit contains a flyback diodes to protect its transistors (even though they are highly recommended due to `Lenz's Law <https://en.wikipedia.org/wiki/Lenz%27s_law>`_).  Therefore, passing a desired speed of ``0`` to the `cellerate()` or `go()` functions of the drivetrain and motor objects will effectively de-energize the coils in the motors.

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

nRF24L01 Interface
------------------

.. autoclass:: drivetrain.motor.NRF24L01
    :members:

USB Interface
------------------

.. autoclass:: drivetrain.motor.USB
    :members:
