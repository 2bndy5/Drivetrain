"""to simplify importing objects across all modules"""
from .motor import Solenoid, BiMotor, PhasedMotor
from .stepper import StepperMotor
from .drivetrain import Drivetrain, Tank, Automotive, Locomotive, Mecanum
from .interfaces import NRF24L01tx, NRF24L01rx, USBtx, USBrx
from .smoothing_input import Smooth
from drivetrain.buffer_mixin import BufferMixin

__all__ = [
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
    'USBrx',
    'Smooth',
    'BufferMixin'
]
