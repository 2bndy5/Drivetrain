"""
A colection of controlling interfaces for drivetrains (both external and internal).

"""

from serial import Serial, SerialException
from busio import SPI
from digitalio import DigitalInOut
from circuitpython_nrf24l01 import RF24

class NRF24L01():
    """This class acts as a wrapper for circuitpython-nrf24l01 module to remotely control a
    peripheral device using nRF24L01 radio transceivers.

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
        return self._prev_cmds

class NRF24L01tx(NRF24L01):
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
        print('transmit', repr(cmds), 'returned:', self._rf.send(command))

class NRF24L01rx(NRF24L01):
    def __init__(self, spi, pins, drivetrain, address=b'rfpi0'):
        self._d_train = drivetrain
        super(NRF24L01rx, self).__init__(spi, pins, address=b'rfpi0')
        self._rf.listen = True

    def sync(self):
        if self._rf.any():
            self.go(self._rf.recv())

    def go(self, cmds):
        self._prev_cmds = list(cmds)
        self._d_train.go(self.value)

class USB():
    """
    This class acts as a wrapper to pyserial module for communicating to an external USB serial
    device. Specifically designed for an Arduino running custom code.

    :param string address: The serial port address of the external serial device.
    :param int baud: The specific baudrate to be used for the serial connection. If left
        unspecified, the serial library will assume a baudrate of 9600.

    """
    def __init__(self, address='/dev/ttyUSB0', baud=-1):
        if baud < 0:
            self._ser = Serial(address)
        else:
            self._ser = Serial(address, baud)
        # print('Successfully opened port {} @ {} to Arduino device'.format(address, baud))
        self._ser.close()
        self._prev_cmds = [None, None]

    @property
    def value(self):
        return self._prev_cmds

class USBtx(USB):
    def go(self, cmds):
        """Assembles an encoded bytearray for outputting over the Serial connection.

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
    def __init__(self, drivetrain, address='/dev/ttyUSB0', baud=-1):
        self._d_train = drivetrain
        super(USBrx, self).__init__(address=address, baud=baud)

    def sync(self):
        commands = []
        with self._ser as usb:
            if usb.in_waiting:
                commands = usb.readline().rsplit(' ')
        for i in range(len(commands)):
            commands[i] = int(commands[i])
        if len(commands):
            self.go(commands)

    def go(self, cmds):
        self._prev_cmds = cmds
        self._d_train.go(self.value)
