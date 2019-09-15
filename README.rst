.. image:: https://readthedocs.org/projects/drivetrain/badge/?version=latest
    :target: https://drivetrain.readthedocs.io/en/latest/?badge=latest
    :alt: Documentation Status

Introduction
============

A collection of motor drivers classes and a special drivetrain class to coordinate the motors objects in generic configurations. This takes advatage of the threading module for smoothing motor input commands in background running threads. This was developed for & tested on the Raspberry PI.

Dependencies
============

This library requires the `digitalio` and `pulseio` modules from the `adafruit-blinka Library <https://pypi.org/project/Adafruit-Blinka/>`_

The stepper motor driver class was originally built atop the gpiozero base classes, and is due for an upgrade to depend on circuitpython's board and digitalio modules. All this means the library is still a work in progress, and `gpiozero library <https://pypi.org/project/gpiozero/>`_ is also currently required (will get dropped soon).

Installation
============

Currenty, there is no plan to deploy this library to pypi yet.

You can easily install this library to your Raspberry Pi in the terminal using the following commands:

.. code-block:: shell

    git clone http://github.com/DVC-Viking-Robotics/Drivetrain.git
    cd Drivetrain
    python3 setup.py install

Some cases may require the last command be prefixed with ``sudo`` or appended with ``--user``.

Installing this library should also automatically install the `adafruit-blinka library <https://pypi.org/project/Adafruit-Blinka/>`_, but if you run into an import error related to the digitalio or pulseio modules, make sure the `adafruit-blinka library <https://pypi.org/project/Adafruit-Blinka/>`_ is install via:

.. code-block:: shell

    pip3 install adfruit-blinka

Again, some cases may require the command be prefixed with ``sudo`` or appended with ``--user``.
