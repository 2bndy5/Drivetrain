"""
A simple test of the Automotive drivetrain class.

This iterates through a list of drivetrain commands
and tallies up the ellapsed time taken to acheive each set of commands
as well as the ellapsed time taken for each motor to acheive each individual command
"""
# pylint: disable=invalid-name
import time
import board
from drivetrain.drivetrain import Automotive, PhasedMotor

mymotors = [PhasedMotor([board.D22, board.D13], ramp_time=2000),
            PhasedMotor([board.D17, board.D18], ramp_time=2000)]
d = Automotive(mymotors)
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
    for i, t_val in enumerate(test):
        test[i] = t_val * 655.35
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

    print('test commands {} took {} seconds'.format(repr(test), t - start))
    for j, m in enumerate(mymotors):
        if end[j] is not None:
            print('motor {} acheived {} in {} seconds'.format(j, m.value, end[j]-start))
        else:
            print("motor {} didn't finish cellerating and a has value of {}".format(j, m.value))
    print(' ') # for clearer print statement grouping
