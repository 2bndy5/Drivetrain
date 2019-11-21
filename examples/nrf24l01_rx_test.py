import board
from digitalio import DigitalInOut as Dio
from circuitpython_nrf24l01 import RF24
from drivetrain import Mecanum, BiMotor, NRF24L01rx

SPI_BUS = board.SPI()
nrf = RF24(SPI_BUS, Dio(board.D5), Dio(board.D4))

# Front-Right, Rear-Right, Rear-Left, Front-Left
motors = [
    BiMotor([board.RX, board.TX]),
    BiMotor([board.D13, board.D12]),
    BiMotor([board.D11, board.D10]),
    BiMotor([board.D2, board.D7])
    ]

d = NRF24L01rx(nrf, Mecanum(motors))

while True:
    d.sync()
