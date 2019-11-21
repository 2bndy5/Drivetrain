"""to simplify importing objects across all modules"""
from .drivetrain import Drivetrain, Tank, Automotive, Locomotive, Mecanum
from .motor import Solenoid, BiMotor, PhasedMotor
from .interfaces import NRF24L01tx, NRF24L01rx, USBtx, USBrx
from .stepper import StepperMotor
__all__ = [
    'Drivetrain',
    'Tank',
    'Automotive',
    'Locomotive',
    'Mecanum',
    'Solenoid',
    'BiMotor',
    'PhasedMotor',
    'StepperMotor',
    'NRF24L01tx',
    'NRF24L01rx',
    'USBtx',
    'USBrx'
]
