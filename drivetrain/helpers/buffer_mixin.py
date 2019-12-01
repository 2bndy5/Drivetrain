import struct

class BufferMixin:
    """A Mixin class to abstract using the ``fmt`` parameter to internal calls of `struct`
    methods.

    :param str cmd_template: This variable will be used as the `"fmt" (Format String of
        Characters) <https://docs.python.org/3.6/library/struct.html#format-strings>`_ parameter
        internally passed to the :py:func:`struct.pack()` and :py:func:`struct.unpack()` for
        transmiting and receiving drivetrain commands. The number of characters in this string must
        correspond to the number of commands in the ``cmds`` list passed to base object's
        ``go()`` method.
    """
    def __init__(self, cmd_template=None, address=None):
        self._fmt = cmd_template
        self._address = address
        self._prev_cmds = [None, None]
        self._crc = 0

    @property
    def value(self):
        """The most previous list of commands that were processed by the drivetrain object"""
        return self._prev_cmds

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
    def address(self):
        """This address used to transmit/receive drivetrain commands via the bus type invoked by
        the parent class. If this parameter is used, please read the documentation on the using the
        parent class's overriden attribute.
        """
        return self._address

    @address.setter
    def address(self, address):
        self._address = address

    @property
    def crc(self):
        """The amount of bytes to used as a CRC checksum. If this parameter is used, please read
        the documentation on the using the parent class's overriden attribute.
        """
        return self._crc

    @crc.setter
    def crc(self, val):
        assert 0 <= val <= 2
        self._crc = val

    def _make_message(self, cmds):
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
        return command

    def _message_unpack(self, message):
        fmt = ''
        for i, byte in enumerate(message):
            if byte == 59: # found ';' now convert all prior chars to str & break
                # str() doesn't support "encoding" kwarg in circuitpython
                fmt = ''.join(chr(c) for c in message[:i])
                break
        return list(struct.unpack(fmt, message[len(fmt) + 1:]))
