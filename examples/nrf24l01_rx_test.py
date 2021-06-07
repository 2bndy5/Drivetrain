"""
Example of library usage receiving commands via an
nRF24L01 transceiver to control a Mecanum drivetrain.
"""
import board
from digitalio import DigitalInOut
from circuitpython_nrf24l01.rf24 import RF24
from drivetrain import Mecanum, BiMotor, NRF24L01rx

# instantiate transceiver radio on the SPI bus
nrf = RF24(board.SPI(), DigitalInOut(board.D5), DigitalInOut(board.D4))

# instantiate motors for a Mecanum drivetrain in the following order
# Front-Right, Rear-Right, Rear-Left, Front-Left
motors = [
    BiMotor(
        plus_pin=PWMOut(board.RX),
        neg_pin=PWMOut(board.TX)),
    BiMotor(
        plus_pin=PWMOut(board.D13),
        neg_pin=PWMOut(board.D12)),
    BiMotor(
        plus_pin=PWMOut(board.D11),
        neg_pin=PWMOut(board.D10)),
    BiMotor(
        plus_pin=PWMOut(board.D2),
        neg_pin=PWMOut(board.D7))
    ]

# instantiate receiving object for a Mecanum drivetrain
d = NRF24L01rx(nrf, Mecanum(motors), address=b"1Node")

while True: # this runs forever
    d.sync()
# doing a keyboard interupt will most likely leave the SPI bus in an
# undesirable state. You must do a hard-reset of the circuitoython MCU to
# reset the SPI bus for continued use. This code assumes power is lost on exit.
