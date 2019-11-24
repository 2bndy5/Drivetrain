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
    def __init__(self, cmd_template):
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
