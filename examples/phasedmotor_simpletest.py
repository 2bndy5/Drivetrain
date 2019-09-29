"""
A simple test of the PhasedMotor class

This iterates through a list of motor commands
and prints the ellapsed time taken to acheive each command
"""
# pylint: disable=invalid-name
import time
import board
from drivetrain.motor import PhasedMotor

motor = PhasedMotor([board.D17, board.D18], ramp_time=2000)
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
    print(f'test result {test * 655.35} took {t - start} seconds')
