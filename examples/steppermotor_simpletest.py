"""A simple test of the StepperMotor class driving a 5V microstepper"""
# pylint: disable=invalid-name
import time
import board
from drivetrain.stepper import StepperMotor

motor = StepperMotor([board.D13, board.D12, board.D11, board.D10])
Steps = [-256, 256, 0] # 1024, 2048, 4096]
Angle = [-15, 15, 0] # 180, 360]
Value = [-25, 25, 0] # -50, 100, 0]
for test in Value:
    motor.value = test # send input instructions
    # do a no delay wait for at least 2 seconds
    start = time.monotonic()
    t = start
    end = None
    while motor.is_changing or t < start + 2:
        t = time.monotonic()
        if not motor.is_changing and end is None:
            end = t
            print(repr(motor))
            print('value acheived in', end-start, 'seconds')
        # elif motor.is_changing:
        #     print(repr(motor))
