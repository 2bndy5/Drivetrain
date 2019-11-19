"""
A colection of controlling interfaces for drivetrains (both external and internal).

"""
PYSERIAL = True
try:
    from serial import Serial
except ImportError:
    PYSERIAL = False # not running on Win32 nor Linux
    try:
        from busio import UART as Serial
    except ImportError: # running on a MicroPython board
        from .uart_serial_ctx import SerialUART as Serial
from circuitpython_nrf24l01 import RF24

class NRF24L01():
    """This class acts as a wrapper for circuitpython-nrf24l01 library for using a
    peripheral device with nRF24L01 radio transceivers. This is a base class to
    :class:`~drivetrain.interfaces.NRF24L01tx` and :class:`~drivetrain.interfaces.NRF24L01rx`
    classes.

    :param ~circuitpython_nrf24l01.RF24 nrf24_object: The instantiated object of the nRF24L01
        transceiver radio.
    :param bytearray address: This will be the RF address used to transmit to the receiving
        nRF24L01 transceiver. For more information on this parameter's usage, please read the
        documentation on the using the `circuitpython-nrf24l01 library
        <https://circuitpython-nrf24l01.rtfd.io/en/latest/api.html>`_
    """
    def __init__(self, nrf24_object, address=b'rfpi0'):
        if not isinstance(nrf24_object, RF24):
            raise ValueError('spi parameter must be an object of type busio.SPI')
        self._rf = nrf24_object
        self._address = address
        self.address = address
        self._rf.listen = False
        # self._rf.what_happened(1) # prints radio condition
        self._prev_cmds = [None, None]

    @property
    def value(self):
        """The most previous list of commands that were processed by the drivetrain object"""
        return self._prev_cmds

    @property
    def address(self):
        return self._address

    @address.setter
    def address(self, address):
        self._address = address
        self._rf.open_tx_pipe(address)
        self._rf.open_rx_pipe(1, address)

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

    :param Tank,Automotive,Locomotive drivetrain: The
        pre-instantiated drivetrain configuration object that is to be controlled.

    See also the :class:`~drivetrain.interfaces.NRF24L01` base class for details about instantiation.
    """
    def __init__(self, nrf24_object, drivetrain, address=b'rfpi0'):
        self._d_train = drivetrain
        super(NRF24L01rx, self).__init__(nrf24_object, address=b'rfpi0')
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

    :param ~busio.UART,~serial.Serial,~machine.UART serial_object: The instantiated serial object to be used for
        the serial connection.
    """
    def __init__(self, serial_object):
        if not isinstance(serial_object, Serial):
            raise ValueError("serial_object not recognized or unsupported")
        self._ser = serial_object
        # print('Successfully opened port {} @ {} to Arduino device'.format(address, baud))
        if PYSERIAL:
            self._ser.close()
        else:
            self._ser.deinit()
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
        with self._ser:
            self._ser.write(commands)

class USBrx(USB):
    """This child class allows the remote controlling of an external drivetrain by receiving
    commands from another MCU via USB serial connection.

    :param Tank,Automotive,Locomotive drivetrain: The
        pre-instantiated drivetrain configuration object that is to be controlled.

    See also the :class:`~drivetrain.interfaces.USB` base class for details about instantiation.
    """
    def __init__(self, drivetrain, serial_object):
        self._d_train = drivetrain
        super(USBrx, self).__init__(serial_object=serial_object)

    def sync(self):
        """Checks if there are new commands waiting in the USB serial device's input stream to be
        processed by the drivetrain object (passed to the constructor upon instantiation)."""
        commands = []
        with self._ser:
            if self._ser.in_waiting:
                if PYSERIAL:
                    # pyserial object doesn't use internal timeout value for read_line()
                    commands = self._ser.read_until() # defaults to '\n' character
                else:
                    commands = self._ser.read_line()
                commands = ''.join([chr(b) for b in commands])
                commands = commands.rsplit(' ')
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
