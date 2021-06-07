.. image:: https://readthedocs.org/projects/drivetrain/badge/?version=latest
    :target: https://drivetrain.readthedocs.io/en/latest/?badge=latest
    :alt: Documentation Status

Introduction
============

A collection of motor drivers classes and specialized drivetrain classes to coordinate the motors' objects in generic configurations. This takes advatage of the threading module for smoothing motor input commands in background running threads. This was developed for & tested on the Raspberry PI. For running this library on CicuitPython devices (that don't have access to the threading module) like the Adafruit ItsyBitsy M4, we have added a fallback function called "sync()" that should get called at least once in the application's main loop.

Dependencies
============

This library requires

    * the `digitalio` and `pulseio` modules from the `adafruit-blinka Library <https://pypi.org/project/Adafruit-Blinka/>`_
    * for serial communications: `pyserial <https://pypi.org/project/pyserial/>`_
    * and for using the nRF24L01 as an interface: `circuitpython-nrf24l01 <https://pypi.org/project/circuitpython-nrf24l01/>`_

Installation
============

Currenty, there is no plan to deploy this library to pypi yet.

You can easily install this library to your Raspberry Pi in the terminal using the following commands:

.. code-block:: shell

    git clone http://github.com/DVC-Viking-Robotics/Drivetrain.git
    cd Drivetrain
    python3 setup.py install

Some cases may require the last command be prefixed with ``sudo`` or appended with ``--user``.

Installing this library should also automatically install the dependencies listed above (platform permitting).

Try the Examples
================

Try out any of the simple test examples in the `examples <https://drivetrain.readthedocs.io/en/latest/examples.html>`_ to make sure everything (including pin connections & library installation) is setup correctly.
