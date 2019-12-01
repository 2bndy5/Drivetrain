"""
A colection of controlling interfaces for drivetrains (both external and internal).

"""
PYSERIAL = True
try:
    from serial import Serial as UART
except ImportError:
    PYSERIAL = False # not running on Win32 nor Linux
    from .helpers.usart_serial_ctx import SerialUART as UART
from circuitpython_nrf24l01 import RF24
from .helpers.buffer_mixin import BufferMixin

IS_TREADED = PYSERIAL

class NRF24L01(BufferMixin):
    """This class acts as a wrapper for circuitpython-nrf24l01 library for using a
    peripheral device with nRF24L01 radio transceivers. This is a base class to
    :class:`~drivetrain.interfaces.NRF24L01tx` and :class:`~drivetrain.interfaces.NRF24L01rx`
    classes.

    :param ~circuitpython_nrf24l01.rf24.RF24 nrf24_object: The instantiated object of the nRF24L01
        transceiver radio.
    :param bytearray address: This will be the RF address used to transmit/receive drivetrain
        commands via the nRF24L01 transceiver. For more information on this parameter's usage,
        please read the documentation on the using the
        :py:meth:`~circuitpython_nrf24l01.rf24.RF24.open_tx_pipe()`
    """
    def __init__(self, nrf24_object, address=b'rfpi0', cmd_template="ll"):
        if not isinstance(nrf24_object, RF24):
            raise ValueError('nRf24L01 object not recognized or supported.')
        self._rf = nrf24_object
        self._rf.listen = False
        # self._rf.what_happened(1) # prints radio condition
        super(NRF24L01, self).__init__(cmd_template=cmd_template, address=address)
        self.address = address

    @property
    def address(self):
        """This `bytearray` will be the RF address used to transmit/receive drivetrain
        commands via the nRF24L01 transceiver. For more information on this parameter's usage,
        please read the documentation on the using the
        :py:meth:`~circuitpython_nrf24l01.rf24.RF24.open_tx_pipe()`"""
        return self._address

    @address.setter
    def address(self, address):
        self._address = address
        self._rf.open_tx_pipe(address)
        self._rf.open_rx_pipe(1, address)

    @property
    def crc(self):
        """The amount of bytes to used as a CRC checksum. This defaults to the maximum of 2.
        See also the `crc <https://circuitpython-nrf24l01.readthedocs.io/en/stable/
        api.html#circuitpython_nrf24l01.rf24.circuitpython_nrf24l01.rf24.RF24.RF24.crc>`_ attribute.
        """
        return self._rf.crc

    @crc.setter
    def crc(self, val):
        assert 0 <= val <= 2
        self._rf.crc = val

class NRF24L01tx(NRF24L01):
    """This child class allows the remote controlling of an external drivetrain by transmitting
    commands to another MCU via the nRF24L01 transceiver. See also the :class:`~drivetrain.interfaces.NRF24L01` base class for
    details about instantiation."""
    def go(self, cmds):
        """Assembles a bytearray to be used for transmitting commands over the air to a receiving
        nRF24L01 transceiver.

        :param list,tuple cmds: A `list` or `tuple` of `int` commands to be sent over the air
            using the nRF24L01. This `list`/`tuple` must have a length equal to the number of
            characters in the :py:attr:`~drivetrain.interfaces.NRF24L01.cmd_template` string.
        """
        command = self._make_message(cmds)
        self._prev_cmds = cmds
        self._rf.send(command)
        # success = self._rf.send(command)
        # print('transmit', repr(cmds), 'returned:', success)

class NRF24L01rx(NRF24L01):
    """This child class allows the external remote controlling of an internal drivetrain by
    receiving commands from another MCU via the nRF24L01 transceiver.

    :param Tank,Automotive,Locomotive,Mecanum drivetrain: The
        pre-instantiated drivetrain configuration object that is to be controlled.

    See also the :class:`~drivetrain.interfaces.NRF24L01` base class for details about instantiation.
    """
    def __init__(self, nrf24_object, drivetrain, address=b'rfpi0', cmd_template='ll'):
        self._d_train = drivetrain
        super(NRF24L01rx, self).__init__(nrf24_object, address=address, cmd_template=cmd_template)
        self._rf.listen = True

    def sync(self):
        """Checks if there are new commands waiting in the nRF24L01's RX FIFO buffer to be
        processed by the drivetrain object (passed to the constructor upon instantiation).
        Any data that is waiting to be received is interpreted and passed to the drivetrain object."""
        if self._rf.any():
            rx = self._rf.recv()
            self.go(self._message_unpack(rx))
        if not IS_TREADED:
            self._d_train.sync()

    def go(self, cmds):
        """Assembles a list of drivetrain commands from the received bytearray via the nRF24L01
        transceiver.

        :param list,tuple cmds: A `list` or `tuple` of `int` commands to be sent the
            drivetrain object (passed to the constructor upon instantiation). This `list`/`tuple`
            must have a length equal to the number of characters in the
            :py:attr:`~drivetrain.interfaces.NRF24L01.cmd_template` string.
        """
        self._prev_cmds = list(cmds)
        self._d_train.go(self.value)

class USB(BufferMixin):
    """
    This base class acts as a wrapper to pyserial module for communicating to an external USB
    serial device. Specifically designed for an Arduino running custom code.

    :param busio.UART,serial.Serial,machine.UART serial_object: The instantiated serial object to
        be used for the serial connection.
    """
    def __init__(self, serial_object, address=None, cmd_template='ll'):
        if not isinstance(serial_object, UART):
            raise ValueError("serial_object not recognized or unsupported")
        self._ser = serial_object
        # print('Successfully opened port {} @ {} to Arduino device'.format(address, baud))
        self._ser.close()
        super(USB, self).__init__(cmd_template=cmd_template, address=address)

class USBtx(USB):
    """This child class allows the remote controlling of an external drivetrain by transmitting
    commands to another MCU via USB serial connection. See also the `USB` base class for
    details about instantiation."""
    def go(self, cmds):
        """Assembles a bytearray for outputting over the Serial connection.

        :param list,tuple cmds: A `list` or `tuple` of `int` commands to be sent over the Serial
            connection. This `list`/`tuple` must have a length equal to the number of characters
            in the :py:attr:`~drivetrain.interfaces.USB.cmd_template` string.
        """
        command = self._make_message(cmds)
        self._prev_cmds = cmds
        with self._ser:
            self._ser.write(command + b'\n') # terminate command w/ '\n' character for readline()

class USBrx(USB):
    """This child class allows the remote controlling of an external drivetrain by receiving
    commands from another MCU via USB serial connection.

    :param Tank,Automotive,Locomotive,Mecanum drivetrain: The
        pre-instantiated drivetrain configuration object that is to be controlled.

    See also the :class:`~drivetrain.interfaces.USB` base class for details about instantiation.
    """
    def __init__(self, drivetrain, serial_object, address=None, cmd_template='ll'):
        self._d_train = drivetrain
        super(USBrx, self).__init__(serial_object, cmd_template=cmd_template, address=address)

    def sync(self):
        """Checks if there are new commands waiting in the USB serial device's input stream to be
        processed by the drivetrain object (passed to the constructor upon instantiation).
        Any data that is waiting to be received is interpreted and passed to the drivetrain object."""
        rx = b''
        with self._ser:
            if self._ser.in_waiting:
                rx = self._ser.read_until() # defaults to '\n' character
        if rx:
            # ignore '\n' at the end
            # nl_trimmed = rx if rx[-1:][0] != 10 else rx[: -1]
            self.go(self._message_unpack(rx)) # ignore fmt + ';'
        if IS_TREADED:
            self._d_train.sync()

    def go(self, cmds):
        """Assembles a list of drivetrain commands from the received bytearray over the USB
        serial connection.

        :param list,tuple cmds: A `list` or `tuple` of `int` commands to be sent the
            drivetrain object (passed to the constructor upon instantiation). This `list`/`tuple`
            must have a length equal to the number of characters in the
            :py:attr:`~drivetrain.interfaces.USB.cmd_template` string.
        """
        self._prev_cmds = cmds
        self._d_train.go(cmds)
