class Stepper:

    def reset_0_angle(self):
        """A calibrating function that will reset the motor's zero angle to its current position."""
        self._steps = 0

    def tick(self):
        """This function should be used only once per main loop iteration. It will trigger stepping operations on the motor if needed."""
        # direction is automatically detirmined toward the shortest route. *see _is_cw
        if self.steps != self._target_pos:
            # print(self._steps, '!=', self._target_pos)
            # iterate self._steps
            self._step()
            # write to pins
            self._write()
            # wait a certain amount of time based on motor speed
            # self.delay()

    @property
    def is_changing(self):
        """This attribute will return a `bool` indicating if the motor is in the midst of moving. (read-only)"""
        return self._target_pos != self.steps if self._target_pos is not None else False


    @angle.setter
    def angle(self, val):
        """
        Rotate motor to specified angle w/ respect to SPR
        All input angle is valid since it is wrapped to range [0,360]. *see _wrap_it()
        """
        # _wrap_it angle to constraints of [0,360] degrees
        if val is None:
            self.reset_0_angle()
        else:
            self._target_pos = math.ceil(self._wrap_it(360, 0, val) / self.dps)
            # print('targetPos =', self._target_pos, 'curr_pos =', self.steps)
            self.tick()
