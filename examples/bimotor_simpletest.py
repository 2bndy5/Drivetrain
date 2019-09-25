"""
A simple test of the BiMotor class

This iterates through a list of motor commands
and prints the ellapsed time taken to acheive each command
"""
# pylint: disable=invalid-name
import time
import board
from drivetrain.motor import BiMotor

motor = BiMotor([board.D22, board.D13], ramp_time=2000)
Value = [-25, 25, -100, 100, 0]
for test in Value:
    # send input instructions
    # NOTE we convert the percentage value to range [-65535, 65535]
    motor.cellerate = test * 655.35
    start = time.monotonic()
    t = start
    # do a no delay wait for at least 3 seconds
    while t < start + 3:
        pass
    print(f'test command {test * 655.35} took {time.monotonic()-start} seconds')
