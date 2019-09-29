"""
A simple test of the Tank drivetrain class.

This iterates through a list of drivetrain commands
and tallies up the ellapsed time taken to acheive each set of commands
as well as the ellapsed time taken for each motor to acheive each individual command
"""
# pylint: disable=invalid-name
import time
import board
from drivetrain.drivetrain import Tank, BiMotor

mymotors = [BiMotor([board.D22, board.D13], ramp_time=2000),
            BiMotor([board.D17, board.D18], ramp_time=2000)]
d = Tank(mymotors, max_speed=100)
testInput = [[100, 0],
             [-100, 0],
             [0, 0],
             [0, 100],
             [0, -100],
             [0, 0]]
for test in testInput:
    # use the list `end` to keep track of each motor's ellapsed time
    end = []
    # NOTE we convert a percentage to range of an 32 bit int
    for t_val in test:
        t_val = t_val * 655.35
    for m in mymotors:
        # end timer for motor[i] = end[i]
        end.append(None)
    d.go(test)  # send input commands
    # unanimous start of all timmers
    start = time.monotonic()
    t = start
    # do a no delay wait for at least 3 seconds
    while d.is_cellerating or t < start + 3:
        t = time.monotonic()
        for j, m in enumerate(mymotors):
            if not m.is_cellerating and end[j] is None:
                end[j] = t

    print(f'test commands {repr(test)} took {time.monotonic() - start} seconds')
    for j, m in enumerate(mymotors):
        if end[j] is not None:
            print(f'motor {j} acheived {m.value} in {end[j]-start} seconds')
        else:
            print(f"motor {j} didn't finish cellerating and a has value of {m.value}")
    print(' ') # for clearer print statement grouping
