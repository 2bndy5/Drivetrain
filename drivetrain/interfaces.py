"""
A colection of controlling interfaces for drivetrains (both external and internal).

"""
import struct
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

    :param ~circuitpython_nrf24l01.rf24.RF24 nrf24_object: The instantiated object of the nRF24L01
        transceiver radio.
    :param bytearray address: This will be the RF address used to transmit to the receiving
        nRF24L01 transceiver. For more information on this parameter's usage, please read the
        documentation on the using the `circuitpython-nrf24l01 library
        <https://circuitpython-nrf24l01.rtfd.io/en/latest/api.html>`_
    :param str cmd_template: This variable will be used as the `"fmt" (Format String of
        Characters) <https://docs.python.org/3.6/library/struct.html#format-strings>`_ parameter
        internally passed to the :py:func:`struct.pack()` and :py:func:`struct.unpack()` for
        transmiting and receiving drivetrain commands. The number of characters in this string must
        correspond to the number of commands in the ``cmds`` list passed to
        :py:meth:`~drivetrain.interfaces.NRF24L01rx.go()`.
    """
    def __init__(self, nrf24_object, address=b'rfpi0', cmd_template="ll"):
        if not isinstance(nrf24_object, RF24):
            raise ValueError('nRf24L01 object not recognized or supported.')
        self._rf = nrf24_object
        self._address = address
        self.address = address
        self._rf.listen = False
        # self._rf.what_happened(1) # prints radio condition
        self._prev_cmds = [None, None]
        self._fmt = cmd_template

    @property
    def cmd_template(self):
        """Use this attribute to change or check the format string used to pack or unpack
        drivetrain commands in `bytearray` form. Refer to `Format String and Format Characters
        <https://docs.python.org/3.6/library/struct.html#format-strings>`_  for allowed datatype
        aliases. The number of characters in this string must correspond to the number of commands
        in the ``cmds`` list passed to :py:meth:`~drivetrain.interfaces.NRF24L01rx.go()`."""
        return self._fmt

    @cmd_template.setter
    def cmd_template(self, fmt):
        self._fmt = fmt

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
            using the nRF24L01. This `list`/`tuple` must have a length equal to the number of
            characters in the :py:attr:`~drivetrain.interfaces.NRF24L01.cmd_template` string.
        """
        self._prev_cmds = cmds
        command = self._fmt.encode() + b';'
        for i, c in enumerate(self._fmt):
            try:
                command += struct.pack(c, cmds[i])
            except struct.error:
                raise ValueError("command argument, {cmds[i]}, not in range of datatype '{c}'. "
                                 "Refer to 'Format Characters' in python's struct docs for "
                                 "adequate datatype aliases.")
            except IndexError:
                raise ValueError("expected {} commands, but {} were given".format(
                    len(self._fmt), len(cmds)))
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
    def __init__(self, nrf24_object, drivetrain, address=b'rfpi0', cmd_template="ll"):
        self._d_train = drivetrain
        super(NRF24L01rx, self).__init__(nrf24_object, address=b'rfpi0', cmd_template=cmd_template)
        self._rf.listen = True

    def sync(self):
        """Checks if there are new commands waiting in the nRF24L01's RX FIFO buffer to be
        processed by the drivetrain object (passed to the constructor upon instantiation).
        Any data that is waiting to be received is interpreted and passed to the drivetrain object."""
        if self._rf.any():
            rx = self._rf.recv()
            fmt = b''
            for i, byte in enumerate(rx):
                if byte == 59:
                    fmt = str(rx[:i], encoding='utf-8')
                    break
            self.go(list(struct.unpack(fmt, rx[len(fmt) + 1:])))

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

class USB():
    """
    This base class acts as a wrapper to pyserial module for communicating to an external USB
    serial device. Specifically designed for an Arduino running custom code.

    :param busio.UART,serial.Serial,machine.UART serial_object: The instantiated serial object to
        be used for the serial connection.
    :param str cmd_template: This variable will be used as the `"fmt" (Format String of
        Characters) <https://docs.python.org/3.6/library/struct.html#format-strings>`_ parameter
        internally passed to the :py:func:`struct.pack()` and :py:func:`struct.unpack()` for
        transmiting and receiving drivetrain commands. The number of characters in this string must
        correspond to the number of commands in the ``cmds`` list passed to
        :py:meth:`~drivetrain.interfaces.USBrx.go()`.
    """
    def __init__(self, serial_object, cmd_template="ll"):
        if not isinstance(serial_object, Serial):
            raise ValueError("serial_object not recognized or unsupported")
        self._ser = serial_object
        # print('Successfully opened port {} @ {} to Arduino device'.format(address, baud))
        if PYSERIAL:
            self._ser.close()
        else:
            self._ser.deinit()
        self._prev_cmds = [None, None]
        self._fmt = cmd_template

    @property
    def cmd_template(self):
        """Use this `str` attribute to change or check the format string used to pack or unpack
        drivetrain commands in `bytearray` form. Refer to `Format String and Format Characters
        <https://docs.python.org/3.6/library/struct.html#format-strings>`_  for allowed datatype
        aliases. The number of characters in this string must correspond to the number of commands
        in the ``cmds`` list passed to :py:meth:`~drivetrain.interfaces.USBrx.go()`."""
        return self._fmt

    @cmd_template.setter
    def cmd_template(self, fmt):
        self._fmt = fmt

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
            connection. This `list`/`tuple` must have a length equal to the number of characters
            in the :py:attr:`~drivetrain.interfaces.USB.cmd_template` string.
        """
        self._prev_cmds = cmds
        command = self._fmt.encode() + b';'
        for i, c in enumerate(self._fmt):
            try:
                command += struct.pack(c, cmds[i])
            except struct.error:
                raise ValueError("command argument, {cmds[i]}, not in range of datatype {i}. "
                                 "Refer to 'Format Characters' in python's struct docs for "
                                 "adequate datatype aliases.")
            except IndexError:
                raise ValueError("expected {} commands, but {} were given".format(
                    len(self._fmt), len(cmds)))
        with self._ser:
            self._ser.write(command + b'\n') # terminate command w/ '\n' character for readline()

class USBrx(USB):
    """This child class allows the remote controlling of an external drivetrain by receiving
    commands from another MCU via USB serial connection.

    :param Tank,Automotive,Locomotive drivetrain: The
        pre-instantiated drivetrain configuration object that is to be controlled.

    See also the :class:`~drivetrain.interfaces.USB` base class for details about instantiation.
    """
    def __init__(self, drivetrain, serial_object, cmd_template="ll"):
        self._d_train = drivetrain
        super(USBrx, self).__init__(serial_object=serial_object, cmd_template=cmd_template)

    def sync(self):
        """Checks if there are new commands waiting in the USB serial device's input stream to be
        processed by the drivetrain object (passed to the constructor upon instantiation).
        Any data that is waiting to be received is interpreted and passed to the drivetrain object."""
        rx = b''
        with self._ser:
            if self._ser.in_waiting:
                if PYSERIAL:
                    # pyserial object doesn't use internal timeout value for read_line()
                    rx = self._ser.read_until() # defaults to '\n' character
                else:
                    rx = self._ser.read_line()
        if rx:
            fmt = ''
            for i, byte in enumerate(rx):
                if byte == 59:
                    fmt = str(rx[:i], encoding='utf-8')
                    break
            self.go(list(struct.unpack(fmt, rx[len(fmt) + 1 : -1]))) # ignore fmt & '\n

    def go(self, cmds):
        """Assembles a list of drivetrain commands from the received bytearray over the USB
        serial connection.

        :param list,tuple cmds: A `list` or `tuple` of `int` commands to be sent the
            drivetrain object (passed to the constructor upon instantiation). This `list`/`tuple`
            must have a length equal to the number of characters in the
            :py:attr:`~drivetrain.interfaces.USB.cmd_template` string.
        """
        self._prev_cmds = cmds
        self._d_train.go(self.value)
