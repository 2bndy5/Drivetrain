"""
Example of library usage receiving commands via an
nRF24L01 transceiver to control a Mecanum drivetrain.
"""
import board
from digitalio import DigitalInOut as Dio
from circuitpython_nrf24l01 import RF24
from drivetrain import Mecanum, BiMotor, NRF24L01rx

# instantiate transceiver radio on the SPI bus
nrf = RF24(board.SPI(), Dio(board.D5), Dio(board.D4))

# instantiate motors for a Mecanum drivetrain in the following order
# Front-Right, Rear-Right, Rear-Left, Front-Left
motors = [
    BiMotor([board.RX, board.TX]),
    BiMotor([board.D13, board.D12]),
    BiMotor([board.D11, board.D10]),
    BiMotor([board.D2, board.D7])
    ]
# NOTE there are no more PWM pins available

# instantiate receiving object for a Mecanum drivetrain
d = NRF24L01rx(nrf, Mecanum(motors))

while True: # this runs forever
    d.sync()
# doing a keyboard interupt will most likely leave the SPI bus in an
# undesirable state. You must do a hard-reset of the circuitoython MCU to
# reset the SPI bus for continued use. This code assumes power is lost on exit.
