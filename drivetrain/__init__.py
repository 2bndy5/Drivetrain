"""to simplify importing objects across all modules"""
from .drivetrain import Tank, Automotive, Locomotive, External
from .motor import Solenoid, BiMotor, PhasedMotor, NRF24L01, USB
from .stepper import StepperMotor
__all__ = ['Tank', 'Automotive', 'Locomotive', 'External', 'Solenoid', 'BiMotor', 'PhasedMotor', 'StepperMotor', 'NRF24L01', 'USB']
