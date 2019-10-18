"""
A colection of controlling interfaces for drivetrains (both external and internal).

"""

from serial import Serial
from busio import SPI
from digitalio import DigitalInOut
from circuitpython_nrf24l01 import RF24

class NRF24L01():
    """This class acts as a wrapper for circuitpython-nrf24l01 library for using a
    peripheral device with nRF24L01 radio transceivers. This is a base class to
    :class:`~drivetrain.interfaces.NRF24L01tx` and :class:`~drivetrain.interfaces.NRF24L01rx`
    classes.

    :param ~busio.SPI spi: The object of the SPI bus used to connect to the nRF24L01 transceiver.

        .. note:: This object should be shared among other driver classes that connect to different
            devices on the same SPI bus (SCK, MOSI, & MISO pins)

    :param list,tuple pins: A `list` or `tuple` of (`board` module's) digital output pins that
        are connected to the nRF24L01's CSN and CE pins respectively. The length of this `list`
        /`tuple` must be 2, otherwise a `ValueError` exception is thrown.

    :param bytearray address: This will be the RF address used to transmit to the receiving
        nRF24L01 transceiver. For more information on this parameter's usage, please read the
        documentation on the using the `circuitpython-nrf24l01 library
        <https://circuitpython-nrf24l01.rtfd.io/en/latest/api.html>`_
    """
    def __init__(self, spi, pins, address=b'rfpi0'):
        if not isinstance(pins, (list, tuple)) and len(pins) != 2:
            raise ValueError('pins parameter must be a list of length 2 (CE and CSN respectively)')
        if not isinstance(spi, SPI):
            raise ValueError('spi parameter must be an object of type busio.SPI')
        csn = DigitalInOut(pins[0]) # CSN
        ce = DigitalInOut(pins[1]) # CE
        self._rf = RF24(spi, csn, ce)
        self._rf.open_tx_pipe(address)
        self._rf.open_rx_pipe(1, address)
        # self._rf.what_happened(1) # prints radio condition
        self._prev_cmds = [None, None]

    @property
    def value(self):
        """The most previous list of commands that were processed by the drivetrain object"""
        return self._prev_cmds

class NRF24L01tx(NRF24L01):
    """This child class allows the remote controlling of an external drivetrain by transmitting
    commands to another MCU via the nRF24L01 transceiver. See also the :class:`~drivetrain.interfaces.NRF24L01` base class for
    details about instantiation."""
    def go(self, cmds):
        """Assembles a bytearray to be used for transmitting commands over the air to a receiving
        nRF24L01 transceiver.

        :param list,tuple cmds: A `list` or `tuple` of `int` commands to be sent over the air
            using the nRF24L01. This `list`/`tuple` can have any length (at least 1) as needed.
        """
        self._prev_cmds = cmds
        command = b''
        for cmd in cmds:
            command += bytes([cmd])
        self._rf.send(command)
        # success = self._rf.send(command)
        # print('transmit', repr(cmds), 'returned:', success)

class NRF24L01rx(NRF24L01):
    """This child class allows the external remote controlling of an internal drivetrain by
    receiving commands from another MCU via the nRF24L01 transceiver.
    
    :param :class:~drivetrain.drivetrain.Tank,:class:~drivetrain.drivetrain.Automotive, :class:~drivetrain.drivetrain.Locomotive drivetrain: The pre-instantiated drivetrain
        configuration object that is to be controlled.

    See also the :class:`~drivetrain.interfaces.NRF24L01` base class for details about instantiation.
    """
    def __init__(self, spi, pins, drivetrain, address=b'rfpi0'):
        self._d_train = drivetrain
        super(NRF24L01rx, self).__init__(spi, pins, address=b'rfpi0')
        self._rf.listen = True

    def sync(self):
        """Checks if there are new commands waiting in the nRF24L01's RX FIFO buffer to be
        processed by the drivetrain object (passed to the constructor upon instantiation)."""
        if self._rf.any():
            self.go(self._rf.recv())

    def go(self, cmds):
        """Assembles a list of drivetrain commands from the received bytearray via the nRF24L01
        transceiver.

        :param list,tuple cmds: A `list` or `tuple` of `int` commands to be sent the 
            drivetrain object (passed to the constructor upon instantiation). This `list`/`tuple`
            can have any length (at least 1) as needed.
        """
        self._prev_cmds = list(cmds)
        self._d_train.go(self.value)

class USB():
    """
    This base class acts as a wrapper to pyserial module for communicating to an external USB
    serial device. Specifically designed for an Arduino running custom code.

    :param string address: The serial port address of the external USB serial device.
    :param int baud: The specific baudrate to be used for the serial connection. If left
        unspecified, the default baudrate of 9600 is used.
    :param int timeout: Specific number of seconds till the threading
        :class:`~serial.Serial`'s :func:`~serial.Serial.read_until()` operation expires. Defaults
        to 1 second.
    """
    def __init__(self, address='/dev/ttyUSB0', baud=9600, timeout=1.0):
        self._ser = Serial(address=address, baud=baud, timeout=timeout)
        # print('Successfully opened port {} @ {} to Arduino device'.format(address, baud))
        self._ser.close()
        self._prev_cmds = [None, None]

    @property
    def value(self):
        """The most previous list of commands that were processed by the drivetrain object"""
        return self._prev_cmds

class USBtx(USB):
    """This child class allows the remote controlling of an external drivetrain by transmitting
    commands to another MCU via USB serial connection. See also the `USB` base class for
    details about instantiation."""
    def go(self, cmds):
        """Assembles a bytearray for outputting over the Serial connection.

        :param list,tuple cmds: A `list` or `tuple` of `int` commands to be sent over the Serial
            connection. This `list`/`tuple` can have any length (at least 1) as needed.
        """
        self._prev_cmds = cmds
        commands = b''
        for i, cmd in enumerate(cmds):
            commands += bytes([cmd])
            if i + 1 < len(cmds):
                commands += ' '
        with self._ser as usb:
            usb.write(commands)

class USBrx(USB):
    """This child class allows the remote controlling of an external drivetrain by receiving
    commands from another MCU via USB serial connection.
    
    :param :class:~drivetrain.drivetrain.Tank,:class:~drivetrain.drivetrain.Automotive,:class:~drivetrain.drivetrain.Locomotive drivetrain: The pre-instantiated drivetrain
        configuration object that is to be controlled.

    See also the :class:`~drivetrain.interfaces.USB` base class for details about instantiation.
    """
    def __init__(self, drivetrain, address='/dev/ttyUSB0', baud=9600):
        self._d_train = drivetrain
        super(USBrx, self).__init__(address=address, baud=baud)

    def sync(self):
        """Checks if there are new commands waiting in the USB serial device's input stream to be
        processed by the drivetrain object (passed to the constructor upon instantiation)."""
        commands = []
        with self._ser as usb:
            if usb.in_waiting:
                commands = usb.read_until().rsplit(' ')
        for i in range(len(commands)):
            commands[i] = int(commands[i])
        if len(commands):
            self.go(commands)

    def go(self, cmds):
        """Assembles a list of drivetrain commands from the received bytearray over the USB
        serial connection.

        :param list,tuple cmds: A `list` or `tuple` of `int` commands to be sent the 
            drivetrain object (passed to the constructor upon instantiation). This `list`/`tuple`
            can have any length (at least 1) as needed.
        """
        self._prev_cmds = cmds
        self._d_train.go(self.value)
