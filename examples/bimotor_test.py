"""
A simple test of the BiMotor class

This iterates through a list of motor commands
and prints the ellapsed time taken to acheive each command
"""
# pylint: disable=invalid-name
import time
import board
from drivetrain import BiMotor

motor = BiMotor([board.D22, board.D13], ramp_time=2000)
Value = [-25, 25, -100, 100, 0]
for test in Value:
    # send input instructions
    # NOTE we convert the percentage value to range [-65535, 65535]
    motor.cellerate(test * 655.35)
    start = time.monotonic()
    t = start
    # do a no delay wait for at most 3 seconds
    while motor.is_cellerating and t < start + 3:
        t = time.monotonic()
    print('test result {} took {} seconds'.format(motor.value, t - start))
